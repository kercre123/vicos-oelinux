/*
 *    Copyright (c) 2015 Qualcomm Technologies, Inc. All Rights Reserved.
 *    Qualcomm Technologies Proprietary and Confidential.
 *
 */

package com.qualcomm.qti.driver.distraction;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;

import com.qualcomm.qti.VehicleFrameworkNotificationType;
import com.qualcomm.qti.VehicleInterfaceSignals;
import com.qualcomm.qti.VehicleManager;
import com.qualcomm.qti.VehicleSignalConstants;

import java.util.List;
import java.util.Vector;
/**
 * DistractionNotifier allows applications to register or deregister for distraction callbacks.
 * It communicates with distraction service internally before triggering the callbacks.
 */

public final class DistractionNotifier {
    private static final String TAG = "DistractionNotifier";
    private IDriverDistractionService mDistractionService;
    private Messenger mServiceRespReceiverMessenger;
    private Vector<DriverDistractionNotification> mClientsDistractionCallbacks;
    private static DistractionNotifier mMyself;
    private boolean mBound;
    private Context mContext;

    public interface DriverDistractionNotification {
        public void onNotify(String status);
        public void onErrorNotify(boolean isRestart);
    }

    public static DistractionNotifier getInstance(Context context) {
        if (mMyself == null) {
            mMyself = new DistractionNotifier(context);
        }
        return mMyself;
    }
    public void registerCallback(DriverDistractionNotification notification) {
        if ((mClientsDistractionCallbacks != null) && (notification != null) ) {
            mClientsDistractionCallbacks.add(notification);
        }
    }
    public void deregisterCallback(DriverDistractionNotification notification) {
        if ((mClientsDistractionCallbacks != null) && (notification != null) ) {
            mClientsDistractionCallbacks.remove(notification);
        }
    }
    /**
    * Constructor for DistractionNotifier;
    */
    private DistractionNotifier(Context context) {
        mBound = false;
        mContext = context;
        mClientsDistractionCallbacks = new Vector();
        Intent i = new Intent("com.qualcomm.qti.driver.distraction.DriverDistractionService");
        PackageManager pm = mContext.getPackageManager();
        List<ResolveInfo> resolveInfoList = pm.queryIntentServices(i, 0);
        if (resolveInfoList == null) {
            Log.e(TAG, "Error querying IntentService..");
        } else {
            ResolveInfo resolveInfoNode = resolveInfoList.get(0);
            ComponentName component = new ComponentName(resolveInfoNode.serviceInfo.packageName,
                    resolveInfoNode.serviceInfo.name);
            i.setComponent(component);
            mContext.bindService(i, mConnection, Context.BIND_AUTO_CREATE);
        }
    }
    /**
    * service connection object to monitor service status.
    */
    private ServiceConnection mConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder service) {
            Log.v(TAG, "onServiceConnected");
            mDistractionService = IDriverDistractionService.Stub.asInterface(service);
            mBound = true;
            if (mDistractionService != null) {
                mServiceRespReceiverMessenger = new Messenger(
                        new DriverDistractionServiceRespHandler());
                try {
                    mDistractionService.registerMessenger(mServiceRespReceiverMessenger);
                } catch (RemoteException rt) {
                    rt.printStackTrace();
                }
            }
        }
        public void onServiceDisconnected(ComponentName className) {
            Log.v(TAG, "onServiceDisconnected");
            mDistractionService = null;
            mBound = false;
            mMyself = null;
            notifyFatalError();
        }
    };
    /**
    * Handler for service response.
    */
    private class DriverDistractionServiceRespHandler extends Handler {
        @Override
        public void handleMessage(Message msg) {
            if ((msg != null) && (mClientsDistractionCallbacks != null)) {
                Bundle distractionInfo = msg.getData();
                String status = distractionInfo.getString("distraction");
                if (mClientsDistractionCallbacks.size() > 0) {
                    for (DriverDistractionNotification element : mClientsDistractionCallbacks) {
                        element.onNotify(status);
                    }
                }
            }
        }
        private static final String TAG = "DriverDistractionServiceRespHandler";
    }
    /**
    * Notify the client about the fatal error and provide the hint to clean up and restart.
    */
    private void notifyFatalError() {
        if (mClientsDistractionCallbacks != null) {
            if (mClientsDistractionCallbacks.size() > 0) {
                for (DriverDistractionNotification element : mClientsDistractionCallbacks) {
                    element.onErrorNotify(true);
                }
            }
        }
    }
}
