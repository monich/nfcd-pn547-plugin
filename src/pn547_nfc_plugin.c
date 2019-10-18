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

#include "plugin.h"

#include "pn547_plugin_p.h"
#include "pn547_log.h"

#include <nfc_adapter.h>
#include <nfc_manager.h>
#include <nfc_plugin_impl.h>

#include <nci_types.h>

GLOG_MODULE_DEFINE("pn547");

#define PN547_DEVICE  "/dev/pn544" /* Yes, pn544! */

typedef NfcPluginClass Pn547NfcPluginClass;
typedef struct pn547_nfc_plugin {
    NfcPlugin parent;
    NfcManager* manager;
    NfcAdapter* adapter;
} Pn547NfcPlugin;

G_DEFINE_TYPE(Pn547NfcPlugin, pn547_nfc_plugin, NFC_TYPE_PLUGIN)
#define PN547_TYPE_PLUGIN (pn547_nfc_plugin_get_type())
#define PN547_NFC_PLUGIN(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), \
        PN547_TYPE_PLUGIN, Pn547NfcPlugin))

static
gboolean
pn547_nfc_plugin_start(
    NfcPlugin* plugin,
    NfcManager* manager)
{
    Pn547NfcPlugin* self = PN547_NFC_PLUGIN(plugin);

    GVERBOSE("Starting");
    self->manager = nfc_manager_ref(manager);
    self->adapter = pn547_nfc_adapter_new(PN547_DEVICE);
    nfc_manager_add_adapter(self->manager, self->adapter);
    return TRUE;
}

static
void
pn547_nfc_plugin_stop(
    NfcPlugin* plugin)
{
    Pn547NfcPlugin* self = PN547_NFC_PLUGIN(plugin);

    GVERBOSE("Stopping");
    if (self->adapter) {
        nfc_manager_remove_adapter(self->manager, self->adapter->name);
        nfc_adapter_unref(self->adapter);
        self->adapter = NULL;
    }
    nfc_manager_unref(self->manager);
    self->manager = NULL;
}

static
void
pn547_nfc_plugin_init(
    Pn547NfcPlugin* self)
{
}

static
void
pn547_nfc_plugin_class_init(
    NfcPluginClass* klass)
{
    klass->start = pn547_nfc_plugin_start;
    klass->stop = pn547_nfc_plugin_stop;
}

static
NfcPlugin*
pn547_nfc_plugin_create(
    void)
{
    GDEBUG("Plugin loaded");
    return g_object_new(PN547_TYPE_PLUGIN, NULL);
}

static GLogModule* const pn547_nfc_plugin_logs[] = {
    &GLOG_MODULE_NAME,
    &pn547_hexdump_log,
    &NCI_LOG_MODULE,
    NULL
};

NFC_PLUGIN_DEFINE2(pn547, "pn547 integration", pn547_nfc_plugin_create,
    pn547_nfc_plugin_logs, 0)

/*
 * Local Variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
