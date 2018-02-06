/*
 *    Copyright (c) 2014 Qualcomm Technologies, Inc. All Rights Reserved.
 *    Qualcomm Technologies Proprietary and Confidential.
 *
 */
package com.qualcomm.qti.vnwboot;

import android.util.Log;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Messenger;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.ServiceManager;
import android.util.Log;

import java.lang.SecurityException;



public class VnwBootReceiver extends BroadcastReceiver {
    private static final String TAG = "vnw-boot-receiver";
    @Override
    public void onReceive(Context context, Intent intent) {
        Log.e(TAG, "entered onReceive");
        if (intent.getAction().equals("android.intent.action.BOOT_COMPLETED"))  {
            Log.v(TAG, "onReceive BOOT_COMPLETED");
            //Intent i = new Intent(context,VehicleService.class);
            try {
                Intent vehicleServiceIntent = new Intent("com.qualcomm.qti.ivi.VehicleService");
                context.startService(vehicleServiceIntent);
                Intent distractionServiceIntent =
                        new Intent("com.qualcomm.qti.driver.distraction.DriverDistractionService");
                context.startService(distractionServiceIntent);
            } catch(SecurityException srvexception) {
                srvexception.printStackTrace();
            }
        }
    }
}
