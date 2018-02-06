// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All Rights Reserved.
// Confidential and Proprietary - Qualcomm Technologies, Inc.
package com.qualcomm.qti.seccamservice;

import android.os.Bundle;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;

public class SecCamServiceVendorHandler {

    private final static String LOG_TAG = "SECCAM-SERVICE-VENDOR-HANDLER";

    public static final int JNI_OK = 0;
    public static final int JNI_EPERM = -1;
    public static final int JNI_EINVAL = -22;
    public static final int JNI_ETIMEDOUT = -110;
    private static final String TZ_APP_NAME = "seccamdemo64";

    private static final int MSG_VENDOR_EXCHANGE_TIMESTAMP = 2000;
    private static final int MSG_VENDOR_TEST_COMMAND = 2001;

    //=========================================================================
    // JNI API
    //=========================================================================
    private static native long exchangeTimestampWithTA(long timestamp);
    private static native int handleTestCommand(byte[] byteArray);
    private static native int startTzAppSession(String j_app_name);
    private static native int shutdownTzAppSession();

    static {
        System.loadLibrary("seccam_vendor");
    }

    //=========================================================================
    //
    //=========================================================================

    static void handleVendorMessage_MSG_VENDOR_EXCHANGE_TIMESTAMP(Message msg) {
        int ret = JNI_OK;

        Log.d(LOG_TAG, "::handleVendorMessage_MSG_VENDOR_EXCHANGE_TIMESTAMP");
        Messenger activityMessenger = msg.replyTo;
        Message replyMsg = Message.obtain();
        replyMsg.what = MSG_VENDOR_EXCHANGE_TIMESTAMP;
        Bundle in_bundle = msg.getData();
        long hlosTimestamp = in_bundle.getLong("hlosTimestamp");

        // In order to initate the session and recieve the tz app handle in the jni,
        // startTzAppSession must be called before any vendor commands are sent to the TA.
        // This needs to be done only once, in the beginning of the session.
        ret = startTzAppSession(TZ_APP_NAME);
        if (JNI_OK != ret) {
            Log.d(LOG_TAG, "::handleVendorMessage_MSG_VENDOR_EXCHANGE_TIMESTAMP startTzAppSession failed " + ret);
            return;
        }

        // Call the JNI function to handle the timestamp exchange command
        long tzTimestamp = exchangeTimestampWithTA(hlosTimestamp);

        // shutdownTzAppSession needs to be called once, when ending the session.
        // after it is called, no additional vendor commands may be sent to the TA.
        ret = shutdownTzAppSession();
        if (JNI_OK != ret) {
            Log.d(LOG_TAG, "::handleVendorMessage_MSG_VENDOR_EXCHANGE_TIMESTAMP shutdownTzAppSession failed " + ret);
            return;
        }
        Bundle out_bundle = new Bundle();
        out_bundle.putLong("tzTimestamp", tzTimestamp);
        replyMsg.setData(out_bundle);
        try {
            activityMessenger.send(replyMsg);
        } catch (RemoteException e) {
            e.printStackTrace();
        }
    }

    static void handleVendorMessage_MSG_VENDOR_TEST_COMMAND(Message msg) {
        int ret = JNI_OK;

        Log.d(LOG_TAG, "::handleVendorMessage_MSG_VENDOR_TEST_COMMAND");
        Messenger activityMessenger = msg.replyTo;
        Message replyMsg = Message.obtain();
        replyMsg.what = MSG_VENDOR_TEST_COMMAND;
        Bundle in_bundle = msg.getData();
        byte[] byteArray = in_bundle.getByteArray("paramsArray");

        ret = startTzAppSession(TZ_APP_NAME);
        if (JNI_OK != ret) {
            Log.d(LOG_TAG, "::handleVendorMessage_MSG_VENDOR_TEST_COMMAND startTzAppSession failed " + ret);
            return;
        }

        int result = handleTestCommand(byteArray);
        if (JNI_OK != ret) {
            Log.d(LOG_TAG, "::handleVendorMessage_MSG_VENDOR_TEST_COMMAND handleTestCommand failed " + ret);
            return;
        }

        ret = shutdownTzAppSession();
        if (JNI_OK != ret) {
            Log.d(LOG_TAG, "::handleVendorMessage_MSG_VENDOR_TEST_COMMAND shutdownTzAppSession failed " + ret);
            return;
        }
        Bundle out_bundle = new Bundle();
        out_bundle.putInt("result", result);
        replyMsg.setData(out_bundle);
        try {
            activityMessenger.send(replyMsg);
        } catch (RemoteException e) {
            e.printStackTrace();
        }
    }

    static boolean handleVendorMessage(Message msg) {
        Log.d(LOG_TAG, "::handleVendorMessage");
        boolean ret = false;

        switch(msg.what) {
            case MSG_VENDOR_EXCHANGE_TIMESTAMP: {
                handleVendorMessage_MSG_VENDOR_EXCHANGE_TIMESTAMP(msg);
                ret = true;
                break;
            }
            case MSG_VENDOR_TEST_COMMAND: {
                handleVendorMessage_MSG_VENDOR_TEST_COMMAND(msg);
                ret = true;
                break;
            }
            default:
                ret = false;
        }

        return ret;
    }
}
