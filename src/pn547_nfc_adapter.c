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
#include "pn547_io.h"

#include <nfc_adapter_impl.h>
#include <nfc_target_impl.h>
#include <nfc_tag_t2.h>
#include <nfc_tag_t4.h>

#include <nci_core.h>
#include <nci_hal.h>

typedef struct pn547_nfc_adapter Pn547NfcAdapter;
typedef NfcAdapterClass Pn547NfcAdapterClass;

/* NCI core events */
enum {
    CORE_EVENT_CURRENT_STATE,
    CORE_EVENT_NEXT_STATE,
    CORE_EVENT_INTF_ACTIVATED,
    CORE_EVENT_COUNT
};

struct pn547_nfc_adapter {
    NfcAdapter adapter;
    NfcTarget* target;
    NciCore* nci;
    gulong nci_event_id[CORE_EVENT_COUNT];
    Pn547HalIo* io;
    gboolean need_power;
    gboolean power_on;
    gboolean power_switch_pending;
    NFC_MODE desired_mode;
    NFC_MODE current_mode;
    gboolean mode_change_pending;
    guint mode_check_id;
    guint presence_check_id;
    guint presence_check_timer;
};

G_DEFINE_TYPE(Pn547NfcAdapter, pn547_nfc_adapter, NFC_TYPE_ADAPTER)
#define PN547_NFC_TYPE_ADAPTER (pn547_nfc_adapter_get_type())
#define PN547_NFC_ADAPTER(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), \
        PN547_NFC_TYPE_ADAPTER, Pn547NfcAdapter))
#define SUPER_CLASS pn547_nfc_adapter_parent_class

#define PRESENCE_CHECK_PERIOD_MS (250)

/*==========================================================================*
 * Implementation
 *==========================================================================*/

static
void
pn547_nfc_adapter_drop_target(
    Pn547NfcAdapter* self)
{
    NfcTarget* target = self->target;

    if (target) {
        self->target = NULL;
        if (self->presence_check_timer) {
            g_source_remove(self->presence_check_timer);
            self->presence_check_timer = 0;
        }
        if (self->presence_check_id) {
            nci_core_cancel(self->nci, self->presence_check_id);
            self->presence_check_id = 0;
        }
        GINFO("Target is gone");
        nfc_target_gone(target);
        nfc_target_unref(target);
    }
}

static
void
pn547_nfc_adapter_presence_check_done(
    NfcTarget* target,
    gboolean ok,
    void* user_data)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(user_data);

    GDEBUG("Presence check %s", ok ? "ok" : "failed");
    self->presence_check_id = 0;
    if (!ok) {
        nci_core_set_state(self->nci, NCI_RFST_DISCOVERY);
    }
}

static
gboolean
pn547_nfc_adapter_presence_check_timer(
    gpointer user_data)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(user_data);

    if (!self->presence_check_id && !self->target->sequence) {
        self->presence_check_id = pn547_nfc_target_presence_check(self->target,
            pn547_nfc_adapter_presence_check_done, self);
        if (!self->presence_check_id) {
            GDEBUG("Failed to start presence check");
            self->presence_check_timer = 0;
            nci_core_set_state(self->nci, NCI_RFST_DISCOVERY);
            return G_SOURCE_REMOVE;
        }
    } else {
        GDEBUG("Skipped presence check");
    }
    return G_SOURCE_CONTINUE;
}

static
void
pn547_nfc_adapter_mode_check(
    Pn547NfcAdapter* self)
{
    NciCore* nci = self->nci;
    const NFC_MODE mode = (nci->current_state > NCI_RFST_IDLE) ?
        NFC_MODE_READER_WRITER : NFC_MODE_NONE;

    if (self->mode_change_pending) {
        if (mode == self->desired_mode) {
            self->mode_change_pending = FALSE;
            self->current_mode = mode;
            nfc_adapter_mode_notify(&self->adapter, mode, TRUE);
        }
    } else if (self->current_mode != mode) {
        self->current_mode = mode;
        nfc_adapter_mode_notify(&self->adapter, mode, FALSE);
    }
}

gboolean
pn547_nfc_adapter_mode_check_cb(
    gpointer user_data)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(user_data);

    self->mode_check_id = 0;
    pn547_nfc_adapter_mode_check(self);
    return G_SOURCE_REMOVE;
}

static
void
pn547_nfc_adapter_schedule_mode_check(
    Pn547NfcAdapter* self)
{
    if (!self->mode_check_id) {
        self->mode_check_id = g_idle_add(pn547_nfc_adapter_mode_check_cb, self);
    }
}

static
gboolean
pn547_nfc_adapter_can_power_off(
    Pn547NfcAdapter* self)
{
    return (self->nci->current_state <= NCI_RFST_IDLE);
}

static
void
pn547_nfc_adapter_state_check(
    Pn547NfcAdapter* self)
{
    if (self->power_on && !self->need_power &&
        pn547_nfc_adapter_can_power_off(self)) {
        pn547_io_set_power(self->io, FALSE);
        self->power_on = FALSE;
        if (self->power_switch_pending) {
            self->power_switch_pending = FALSE;
            nfc_adapter_power_notify(&self->adapter, FALSE, TRUE);
        } else {
            nfc_adapter_power_notify(&self->adapter, FALSE, FALSE);
        }
    }
    pn547_nfc_adapter_mode_check(self);
}

static
const NfcParamPollA*
pn547_nfc_adapter_convert_poll_a(
    NfcParamPollA* dest,
    const NciModeParamPollA* src)
{
    dest->sel_res = src->sel_res;
    dest->nfcid1.bytes = src->nfcid1;
    dest->nfcid1.size = src->nfcid1_len;
    return dest;
}

static
const NfcParamPollB*
pn547_nfc_adapter_convert_poll_b(
    NfcParamPollB* dest,
    const NciModeParamPollB* src)
{
    dest->fsc = src->fsc;
    dest->nfcid0.bytes = src->nfcid0;
    dest->nfcid0.size = sizeof(src->nfcid0);
    return dest;
}

static
const NfcParamIsoDepPollA*
pn547_nfc_adapter_convert_iso_dep_poll_a(
    NfcParamIsoDepPollA* dest,
    const NciActivationParamIsoDepPollA* src)
{
    dest->fsc = src->fsc;
    dest->t1 = src->t1;
    return dest;
}

static
void
pn547_nfc_adapter_nci_intf_activated(
    NciCore* nci,
    const NciIntfActivationNtf* ntf,
    void* user_data)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(user_data);
    const NciModeParam* mp = ntf->mode_param;
    NfcTag* tag = NULL;

    /* Drop the previous target, if any */
    pn547_nfc_adapter_drop_target(self);

    /* Register the new tag */
    self->target = pn547_nfc_target_new(ntf, nci);

    /* Figure out what kind of target we are dealing with */
    if (mp) {
        NfcParamPollA poll_a;
        NfcParamPollB poll_b;

        switch (ntf->mode) {
        case NCI_MODE_PASSIVE_POLL_A:
        case NCI_MODE_ACTIVE_POLL_A:
            switch (ntf->rf_intf) {
            case NCI_RF_INTERFACE_FRAME:
                /* Type 2 Tag */
                tag = nfc_adapter_add_tag_t2(&self->adapter, self->target,
                    pn547_nfc_adapter_convert_poll_a(&poll_a, &mp->poll_a));
                break;
            case NCI_RF_INTERFACE_ISO_DEP:
                /* ISO-DEP Type 4A */
                if (ntf->activation_param) {
                    const NciActivationParam* ap = ntf->activation_param;
                    NfcParamIsoDepPollA iso_dep_poll_a;

                    tag = nfc_adapter_add_tag_t4a(&self->adapter, self->target,
                        pn547_nfc_adapter_convert_poll_a(&poll_a, &mp->poll_a),
                        pn547_nfc_adapter_convert_iso_dep_poll_a
                            (&iso_dep_poll_a, &ap->iso_dep_poll_a));
                }
                break;
            case NCI_RF_INTERFACE_NFCEE_DIRECT:
            case NCI_RF_INTERFACE_NFC_DEP:
                break;
            }
            break;
        case NCI_MODE_PASSIVE_POLL_B:
            switch (ntf->rf_intf) {
            case NCI_RF_INTERFACE_ISO_DEP:
                /* ISO-DEP Type 4B */
                tag = nfc_adapter_add_tag_t4b(&self->adapter, self->target,
                    pn547_nfc_adapter_convert_poll_b(&poll_b, &mp->poll_b),
                    NULL);
                break;
            case NCI_RF_INTERFACE_FRAME:
            case NCI_RF_INTERFACE_NFCEE_DIRECT:
            case NCI_RF_INTERFACE_NFC_DEP:
                break;
            }
            break;
        case NCI_MODE_PASSIVE_POLL_F:
        case NCI_MODE_ACTIVE_POLL_F:
        case NCI_MODE_PASSIVE_POLL_15693:
        case NCI_MODE_PASSIVE_LISTEN_A:
        case NCI_MODE_PASSIVE_LISTEN_B:
        case NCI_MODE_PASSIVE_LISTEN_F:
        case NCI_MODE_ACTIVE_LISTEN_A:
        case NCI_MODE_ACTIVE_LISTEN_F:
        case NCI_MODE_PASSIVE_LISTEN_15693:
            break;
        }
    }

    if (!tag) {
        nfc_adapter_add_other_tag(&self->adapter, self->target);
    }

    /* Start periodic presence checks */
    self->presence_check_timer = g_timeout_add(PRESENCE_CHECK_PERIOD_MS,
        pn547_nfc_adapter_presence_check_timer, self);
}

static
void
pn547_nfc_adapter_nci_next_state_changed(
    NciCore* nci,
    void* user_data)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(user_data);

    if (nci->next_state != NCI_RFST_POLL_ACTIVE) {
        pn547_nfc_adapter_drop_target(self);
        if (nci->next_state == NCI_STATE_ERROR && self->power_on) {
            GDEBUG("Resetting the chip");
            pn547_io_set_power(self->io, FALSE);
            pn547_io_set_power(self->io, TRUE);
        }
    }
    pn547_nfc_adapter_state_check(self);
}

static
void
pn547_nfc_adapter_nci_current_state_changed(
    NciCore* nci,
    void* user_data)
{
    pn547_nfc_adapter_state_check(PN547_NFC_ADAPTER(user_data));
}

/*==========================================================================*
 * Interface
 *==========================================================================*/

NfcAdapter*
pn547_nfc_adapter_new(
    const char* dev)
{
    Pn547HalIo* io = pn547_io_new(dev);

    if (io) {
        Pn547NfcAdapter* self = g_object_new(PN547_NFC_TYPE_ADAPTER, NULL);

        self->io = io;
        self->nci = nci_core_new(&io->hal_io);
        self->nci_event_id[CORE_EVENT_CURRENT_STATE] =
            nci_core_add_current_state_changed_handler(self->nci,
                pn547_nfc_adapter_nci_current_state_changed, self);
        self->nci_event_id[CORE_EVENT_NEXT_STATE] =
            nci_core_add_next_state_changed_handler(self->nci,
                pn547_nfc_adapter_nci_next_state_changed, self);
        self->nci_event_id[CORE_EVENT_INTF_ACTIVATED] =
            nci_core_add_intf_activated_handler(self->nci,
                pn547_nfc_adapter_nci_intf_activated, self);

        return &self->adapter;
    }
    return NULL;
}

/*==========================================================================*
 * Methods
 *==========================================================================*/

static
gboolean
pn547_nfc_adapter_submit_power_request(
    NfcAdapter* adapter,
    gboolean on)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(adapter);
    NciCore* nci = self->nci;

    GASSERT(!self->power_switch_pending);
    self->need_power = on;
    if (on) {
        if (self->power_on) {
            GDEBUG("Power is already on");
            nci_core_set_state(nci, NCI_RFST_IDLE);
            /* Power stays on, we are done */
            nfc_adapter_power_notify(&self->adapter, TRUE, TRUE);
        } else if (pn547_io_set_power(self->io, TRUE)) {
            self->power_on = TRUE;
            nci_core_restart(self->nci);
            nfc_adapter_power_notify(&self->adapter, TRUE, TRUE);
        }
    } else {
        if (self->power_on) {
            if (pn547_nfc_adapter_can_power_off(self)) {
                pn547_io_set_power(self->io, FALSE);
                self->power_on = FALSE;
                nfc_adapter_power_notify(&self->adapter, FALSE, TRUE);
            } else {
                GDEBUG("Waiting for NCI state machine to become idle");
                nci_core_set_state(nci, NCI_RFST_IDLE);
                self->power_switch_pending =
                    (nci->current_state != NCI_RFST_IDLE &&
                    nci->next_state == NCI_RFST_IDLE);
            }
        } else {
            GDEBUG("Power is already off");
            /* Power stays off, we are done */
            nfc_adapter_power_notify(&self->adapter, FALSE, TRUE);
        }
    }
    return self->power_switch_pending;
}

static
void
pn547_nfc_adapter_cancel_power_request(
    NfcAdapter* adapter)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(adapter);

    self->need_power = self->power_on;
    self->power_switch_pending = FALSE;
}

static
gboolean
pn547_nfc_adapter_submit_mode_request(
    NfcAdapter* adapter,
    NFC_MODE mode)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(adapter);

    self->desired_mode = mode;
    self->mode_change_pending = TRUE;
    nci_core_set_state(self->nci, NCI_RFST_DISCOVERY);
    pn547_nfc_adapter_schedule_mode_check(self);
    return TRUE;
}

static
void
pn547_nfc_adapter_cancel_mode_request(
    NfcAdapter* adapter)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(adapter);

    self->mode_change_pending = FALSE;
    pn547_nfc_adapter_schedule_mode_check(self);
}

/*==========================================================================*
 * Internals
 *==========================================================================*/

static
void
pn547_nfc_adapter_init(
    Pn547NfcAdapter* self)
{
    NfcAdapter* adapter = &self->adapter;

    adapter->supported_modes = NFC_MODE_READER_WRITER;
    adapter->supported_tags = NFC_TAG_TYPE_FELICA |
        NFC_TAG_TYPE_MIFARE_CLASSIC | NFC_TAG_TYPE_MIFARE_ULTRALIGHT;
    adapter->supported_protocols =  NFC_PROTOCOL_T2_TAG |
        NFC_PROTOCOL_T4A_TAG | NFC_PROTOCOL_T4B_TAG | NFC_PROTOCOL_NFC_DEP;
}

static
void
pn547_nfc_adapter_dispose(
    GObject* object)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(object);

    pn547_nfc_adapter_drop_target(self);
    G_OBJECT_CLASS(SUPER_CLASS)->dispose(object);
}

static
void
pn547_nfc_adapter_finalize(
    GObject* object)
{
    Pn547NfcAdapter* self = PN547_NFC_ADAPTER(object);

    nci_core_remove_all_handlers(self->nci, self->nci_event_id);
    nci_core_free(self->nci);
    pn547_io_free(self->io);
    G_OBJECT_CLASS(SUPER_CLASS)->finalize(object);
}

static
void
pn547_nfc_adapter_class_init(
    NfcAdapterClass* klass)
{
    GObjectClass* object_class = G_OBJECT_CLASS(klass);

    klass->submit_power_request = pn547_nfc_adapter_submit_power_request;
    klass->cancel_power_request = pn547_nfc_adapter_cancel_power_request;
    klass->submit_mode_request = pn547_nfc_adapter_submit_mode_request;
    klass->cancel_mode_request = pn547_nfc_adapter_cancel_mode_request;
    object_class->dispose = pn547_nfc_adapter_dispose;
    object_class->finalize = pn547_nfc_adapter_finalize;
}

/*
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
