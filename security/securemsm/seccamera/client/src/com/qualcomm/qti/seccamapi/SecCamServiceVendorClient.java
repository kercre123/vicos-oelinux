// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All Rights Reserved.
// Confidential and Proprietary - Qualcomm Technologies, Inc.
package com.qualcomm.qti.seccamapi;

import android.os.Bundle;
import android.os.Message;
import android.os.Messenger;
import android.util.Log;

public class SecCamServiceVendorClient {

    private final static String LOG_TAG = "SECCAM-SERVICE-VENDOR-CLIENT";

    // Vendor specific command IDs
    public static final int MSG_VENDOR_EXCHANGE_TIMESTAMP = 2000;
    public static final int MSG_VENDOR_TEST_COMMAND = 2001;

    public static boolean handleVendorMessage(Message msg) {
        Log.d(LOG_TAG, "handleVendorMessage");
        boolean ret = false;

        switch (msg.what) {
            case MSG_VENDOR_EXCHANGE_TIMESTAMP: {
                Bundle in_bundle = msg.getData();
                long result = in_bundle.getLong("tzTimestamp");
                Log.d(LOG_TAG, "handleVendorMessage - TA timestamp recieved: " + result);
                ret = true;
                break;
            }
            // Test command intendent for test purposes only
            case MSG_VENDOR_TEST_COMMAND: {
                Bundle in_bundle = msg.getData();
                int result = in_bundle.getInt("result");
                Log.d(LOG_TAG, "handleVendorMessage - Test command recieved: " + result);
                ret = true;
                break;
            }
            default:
                Log.d(LOG_TAG, "recieved unhandled command ID");
        }

        return ret;
    }
}
