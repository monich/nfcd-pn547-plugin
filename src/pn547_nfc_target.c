/*
 * Copyright (C) 2019 Jolla Ltd.
 * Copyright (C) 2019 Open Mobile Platform LLC.
 * Copyright (C) 2019 Slava Monich <slava.monich@jolla.com>
 *
 * You may use this file under the terms of the BSD license as follows:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. Neither the names of the copyright holders nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation
 * are those of the authors and should not be interpreted as representing
 * any official policies, either expressed or implied.
 */

#include "pn547_plugin_p.h"
#include "pn547_log.h"

#include <nci_core.h>

#include <nfc_tag.h>
#include <nfc_target_impl.h>

#define T2T_CMD_READ (0x30)

enum {
    EVENT_DATA_PACKET,
    EVENT_COUNT
};

typedef NfcTargetClass Pn547NfcTargetClass;
typedef struct pn547_nfc_target Pn547NfcTarget;

typedef struct pn547_nfc_target_presence_check {
    Pn547NfcTargetPresenseCheckFunc done;
    void* user_data;
} Pn547NfcTargetPresenceCheck;

typedef
guint
(*Pn547NfcTargetPresenceCheckFunc)(
    Pn547NfcTarget* self,
    Pn547NfcTargetPresenceCheck* check);

struct pn547_nfc_target {
    NfcTarget target;
    NciCore* nci;
    NCI_RF_INTERFACE rf_intf;
    gulong event_id[EVENT_COUNT];
    guint send_in_progress;
    gboolean transmit_in_progress;
    GBytes* pending_reply; /* Reply arrived before send has completed */
    Pn547NfcTargetPresenceCheckFunc presence_check_fn;
};

GType pn547_nfc_target_get_type(void);
G_DEFINE_TYPE(Pn547NfcTarget, pn547_nfc_target, NFC_TYPE_TARGET)
#define PN547_NFC_TYPE_TARGET (pn547_nfc_target_get_type())
#define PN547_NFC_TARGET(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), \
        PN547_NFC_TYPE_TARGET, Pn547NfcTarget))

static
Pn547NfcTargetPresenceCheck*
pn547_nfc_target_presence_check_new(
    Pn547NfcTargetPresenseCheckFunc fn,
    void* user_data)
{
    Pn547NfcTargetPresenceCheck* check =
        g_slice_new(Pn547NfcTargetPresenceCheck);

    check->done = fn;
    check->user_data = user_data;
    return check;
}

static
void
pn547_nfc_target_presence_check_free(
    Pn547NfcTargetPresenceCheck* check)
{
    g_slice_free(Pn547NfcTargetPresenceCheck, check);
}

static
void
pn547_nfc_target_presence_check_free1(
    gpointer data)
{
    pn547_nfc_target_presence_check_free(data);
}

static
void
pn547_nfc_target_cancel_send(
    Pn547NfcTarget* self)
{
    if (self->send_in_progress) {
        nci_core_cancel(self->nci, self->send_in_progress);
        self->send_in_progress = 0;
        if (self->pending_reply) {
            g_bytes_unref(self->pending_reply);
            self->pending_reply = NULL;
        }
    }
}

static
void
pn547_nfc_target_drop_nci(
    Pn547NfcTarget* self)
{
    pn547_nfc_target_cancel_send(self);
    nci_core_remove_all_handlers(self->nci, self->event_id);
    self->nci = NULL;
}

static
void
pn547_nfc_target_finish_transmit(
    Pn547NfcTarget* self,
    const guint8* payload,
    guint len)
{
    NfcTarget* target = &self->target;

    self->transmit_in_progress = FALSE;
    if (len > 0) {
        if (self->rf_intf == NCI_RF_INTERFACE_FRAME) {
            const guint8 status = payload[len - 1];

            /*
             * 8.2 Frame RF Interface
             * 8.2.1.2 Data from RF to the DH
             */
            if (status == NCI_STATUS_OK) {
                nfc_target_transmit_done(target, NFC_TRANSMIT_STATUS_OK,
                    payload, len - 1);
                return;
            }
            GDEBUG("Transmission status 0x%02x", status);
        } else if (self->rf_intf == NCI_RF_INTERFACE_ISO_DEP) {
            /*
             * 8.3 ISO-DEP RF Interface
             * 8.3.1.2 Data from RF to the DH
             */
            nfc_target_transmit_done(target, NFC_TRANSMIT_STATUS_OK,
                payload, len);
            return;
        }
    }
    nfc_target_transmit_done(target, NFC_TRANSMIT_STATUS_ERROR, NULL, 0);
}

static
void
pn547_nfc_target_data_sent(
    NciCore* nci,
    gboolean success,
    void* user_data)
{
    Pn547NfcTarget* self = PN547_NFC_TARGET(user_data);

    GASSERT(self->send_in_progress);
    self->send_in_progress = 0;

    if (self->pending_reply) {
        gsize len;
        GBytes* reply = self->pending_reply;
        const guint8* payload = g_bytes_get_data(reply, &len);

        /* We have been waiting for this send to complete */
        GDEBUG("Send completed");
        self->pending_reply = NULL;
        pn547_nfc_target_finish_transmit(self, payload, len);
        g_bytes_unref(reply);
    }
}

static
void
pn547_nfc_target_data_packet_handler(
    NciCore* nci,
    guint8 cid,
    const void* data,
    guint len,
    void* user_data)
{
    Pn547NfcTarget* self = PN547_NFC_TARGET(user_data);

    if (cid == NCI_STATIC_RF_CONN_ID && self->transmit_in_progress &&
        !self->pending_reply) {
        if (G_UNLIKELY(self->send_in_progress)) {
            /*
             * Due to multi-threaded nature of pn547 driver and services,
             * incoming reply transactions sometimes get handled before
             * send completion callback has been invoked. Postpone transfer
             * completion until then.
             */
            GDEBUG("Waiting for send to complete");
            self->pending_reply = g_bytes_new(data, len);
        } else {
            pn547_nfc_target_finish_transmit(self, data, len);
        }
    } else {
        GDEBUG("Unhandled data packet, cid=0x%02x %u byte(s)", cid, len);
    }
}

static
void
pn547_nfc_target_presence_check_complete(
    NfcTarget* target,
    NFC_TRANSMIT_STATUS status,
    const void* data,
    guint len,
    void* user_data)
{
    Pn547NfcTargetPresenceCheck* check = user_data;

    check->done(target, status == NFC_TRANSMIT_STATUS_OK, check->user_data);
}

static
guint
pn547_nfc_target_presence_check_t2(
    Pn547NfcTarget* self,
    Pn547NfcTargetPresenceCheck* check)
{
    static const guint8 cmd_data[] = { T2T_CMD_READ, 0x00 };

    return nfc_target_transmit(&self->target, cmd_data, sizeof(cmd_data),
        NULL, pn547_nfc_target_presence_check_complete,
        pn547_nfc_target_presence_check_free1, check);
}

static
guint
pn547_nfc_target_presence_check_t4(
    Pn547NfcTarget* self,
    Pn547NfcTargetPresenceCheck* check)
{
    return nfc_target_transmit(&self->target, NULL, 0,
        NULL, pn547_nfc_target_presence_check_complete,
        pn547_nfc_target_presence_check_free1, check);
}

/*==========================================================================*
 * Interface
 *==========================================================================*/

NfcTarget*
pn547_nfc_target_new(
    const NciIntfActivationNtf* ntf,
    NciCore* nci)
{
     Pn547NfcTarget* self = g_object_new(PN547_NFC_TYPE_TARGET, NULL);
     NfcTarget* target = &self->target;

     switch (ntf->mode) {
     case NCI_MODE_PASSIVE_POLL_A:
     case NCI_MODE_ACTIVE_POLL_A:
     case NCI_MODE_PASSIVE_LISTEN_A:
     case NCI_MODE_ACTIVE_LISTEN_A:
         target->technology = NFC_TECHNOLOGY_A;
         break;
     case NCI_MODE_PASSIVE_POLL_B:
     case NCI_MODE_PASSIVE_LISTEN_B:
         target->technology = NFC_TECHNOLOGY_B;
         break;
     case NCI_MODE_PASSIVE_POLL_F:
     case NCI_MODE_PASSIVE_LISTEN_F:
     case NCI_MODE_ACTIVE_LISTEN_F:
         target->technology = NFC_TECHNOLOGY_F;
         break;
     default:
         break;
     }

     switch (ntf->protocol) {
     case NCI_PROTOCOL_T1T:
         target->protocol = NFC_PROTOCOL_T1_TAG;
         break;
     case NCI_PROTOCOL_T2T:
         target->protocol = NFC_PROTOCOL_T2_TAG;
         self->presence_check_fn = pn547_nfc_target_presence_check_t2;
         break;
     case NCI_PROTOCOL_T3T:
         target->protocol = NFC_PROTOCOL_T3_TAG;
         break;
     case NCI_PROTOCOL_ISO_DEP:
         self->presence_check_fn = pn547_nfc_target_presence_check_t4;
         switch (target->technology) {
         case NFC_TECHNOLOGY_A:
             target->protocol = NFC_PROTOCOL_T4A_TAG;
             break;
         case NFC_TECHNOLOGY_B:
             target->protocol = NFC_PROTOCOL_T4B_TAG;
             break;
         default:
             GDEBUG("Unexpected ISO_DEP technology");
             break;
         }
         break;
     case NCI_PROTOCOL_NFC_DEP:
         target->protocol = NFC_PROTOCOL_NFC_DEP;
         break;
     default:
         GDEBUG("Unexpected protocol 0x%02x", ntf->protocol);
         break;
     }

     self->rf_intf = ntf->rf_intf;
     self->nci = nci;
     self->event_id[EVENT_DATA_PACKET] = nci_core_add_data_packet_handler(nci,
         pn547_nfc_target_data_packet_handler, self);
     return target;
}

guint
pn547_nfc_target_presence_check(
    NfcTarget* target,
    Pn547NfcTargetPresenseCheckFunc fn,
    void* user_data)
{
    if (G_LIKELY(target)) {
        Pn547NfcTarget* self = PN547_NFC_TARGET(target);

        if (self && self->presence_check_fn) {
            Pn547NfcTargetPresenceCheck* check =
                pn547_nfc_target_presence_check_new(fn, user_data);
            const guint id = self->presence_check_fn(self, check);

            if (id) {
                return id;
            }
            pn547_nfc_target_presence_check_free(check);
        }
    }
    return 0;
}

/*==========================================================================*
 * Methods
 *==========================================================================*/

static
gboolean
pn547_nfc_target_transmit(
    NfcTarget* target,
    const void* data,
    guint len)
{
    Pn547NfcTarget* self = PN547_NFC_TARGET(target);

    GASSERT(!self->send_in_progress);
    GASSERT(!self->transmit_in_progress);
    if (self->nci) {
        GBytes* bytes = g_bytes_new(data, len);

        self->send_in_progress = nci_core_send_data_msg(self->nci,
            NCI_STATIC_RF_CONN_ID, bytes, pn547_nfc_target_data_sent,
            NULL, self);
        g_bytes_unref(bytes);
        if (self->send_in_progress) {
            self->transmit_in_progress = TRUE;
            return TRUE;
        }
    }
    return FALSE;
}

static
void
pn547_nfc_target_cancel_transmit(
    NfcTarget* target)
{
    Pn547NfcTarget* self = PN547_NFC_TARGET(target);

    self->transmit_in_progress = FALSE;
    pn547_nfc_target_cancel_send(self);
}

static
void
pn547_nfc_target_deactivate(
    NfcTarget* target)
{
    Pn547NfcTarget* self = PN547_NFC_TARGET(target);

    nci_core_set_state(self->nci, NCI_RFST_IDLE);
}

static
void
pn547_nfc_target_gone(
    NfcTarget* target)
{
    pn547_nfc_target_drop_nci(PN547_NFC_TARGET(target));
    NFC_TARGET_CLASS(pn547_nfc_target_parent_class)->gone(target);
}

/*==========================================================================*
 * Internals
 *==========================================================================*/

static
void
pn547_nfc_target_init(
    Pn547NfcTarget* self)
{
}

static
void
pn547_nfc_target_finalize(
    GObject* object)
{
    Pn547NfcTarget* self = PN547_NFC_TARGET(object);

    pn547_nfc_target_drop_nci(self);
    G_OBJECT_CLASS(pn547_nfc_target_parent_class)->finalize(object);
}

static
void
pn547_nfc_target_class_init(
    NfcTargetClass* klass)
{
    G_OBJECT_CLASS(klass)->finalize = pn547_nfc_target_finalize;
    klass->deactivate = pn547_nfc_target_deactivate;
    klass->transmit = pn547_nfc_target_transmit;
    klass->cancel_transmit = pn547_nfc_target_cancel_transmit;
    klass->gone = pn547_nfc_target_gone;
}

/*
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
