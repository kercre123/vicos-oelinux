/*
 * Copyright (C) 2016 The Linux Foundation. All rights reserved
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_INCLUDE_BT_VENDOR_H
#define ANDROID_INCLUDE_BT_VENDOR_H

#include <hardware/bluetooth.h>

__BEGIN_DECLS

#define BT_PROFILE_VENDOR_ID "vendor"
#define BT_PROFILE_WIPOWER_VENDOR_ID "wipower"

/** Bluetooth Device Name */
typedef struct {
    uint8_t name[249];
} __attribute__((packed))bt_lename_t;

typedef struct{
    int len;
    void *val;
}btvendor_lename_t;

/** Callback when bredr cleanup is done.
 */
typedef void (*  btvendor_bredr_cleanup_callback)(bool status);

/** Callback to notify ssr cleanup to app.
 */
typedef void (*  btvendor_ssr_cleanup_callback)(void);


/** Bluetooth ACL connection state changed with reason callback */
typedef void (*btvendor_acl_state_changed_with_reason_callback)(bt_status_t status,
                                                       bt_bdaddr_t *remote_bd_addr,
                                                       bt_acl_state_t state,
                                                       uint8_t reason,
                                                       uint8_t transport_type);
/** BT-Vendor callback structure. */
typedef struct {
    /** set to sizeof(BtVendorCallbacks) */
    size_t      size;
    btvendor_bredr_cleanup_callback  bredr_cleanup_cb;
    btvendor_ssr_cleanup_callback    ssr_cleanup_cb;
    btvendor_acl_state_changed_with_reason_callback acl_state_changed_with_reason_cb;
} btvendor_callbacks_t;

/** Represents the standard BT-Vendor interface.
 */
typedef struct {

    /** set to sizeof(BtVendorInterface) */
    size_t          size;

    /**
     * Register the BtVendor callbacks
     */
    bt_status_t (*init)( btvendor_callbacks_t* callbacks );

    /** Does SSR cleanup */
    void (*ssrcleanup)(void);

    /** Does BREDR cleanup */
    void (*bredrcleanup)(void);

    /** Closes the interface. */
    void  (*cleanup)( void );

    void (*setLeBtName)(btvendor_lename_t* name);

    /** Sets the Scan Mode with parameter to ignore LE Scan mode*/
    void (*setScanMode)(bt_scan_mode_t mode, bool ignoreLeScanModes);

} btvendor_interface_t;

__END_DECLS

#endif /* ANDROID_INCLUDE_BT_VENDOR_H */

