/*
 *    Copyright (c) 2014-2015 Qualcomm Technologies, Inc. All Rights Reserved.
 *    Qualcomm Technologies Proprietary and Confidential.
 *
 */

package com.qualcomm.qti;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.util.Log;

import com.qualcomm.qti.ivi.aidl.IVehicleService;

import java.util.List;
import java.util.Vector;

/**
 * VehicleManager provides set of APIs to be used by application developer to
 * query various data items related to the vehicle.
 */

public final class VehicleManager {
    private VehicleManagerCallback mClientCallback;
    private boolean mBound = false;
    private static final String TAG = "VehicleManager";
    private static VehicleManager mMyInstance = null;
    private IVehicleService mVehicleService;
    private Context mClientContext;
    private VehicleInterfaceData mVehicleInterfaceData;
    private Vector mCallbacksVector;

    /**
     * VehicleManager is singleton. Static asynchronous API to get the instance
     * of VehicleManager.
     * @param mContext points to the Context to be used by VehicleManager.
     * @param callback Callback provided by the caller to get the notification
     *            of when VehicleManager handle is created.
     * @return true if no error occurs in triggering asynchronous creation else
     *         returns false. Please note that this API is work in progress, and
     *         there might be few changes going forward.
     */
    public static boolean getInstance(Context mContext, VehicleManagerCallback callback) {
        boolean bret = false;
        if (mMyInstance == null) {
            mMyInstance = new VehicleManager(mContext);
        }
        mMyInstance.addCallback(callback);
        if (mMyInstance.mBound) {
            callback.handleVehicleManagerCreationStatus(mMyInstance);
            bret = true;
        }
        return bret;
    }

    /**
     * Returns VehicleInterfaceData handle; Please note that this API is work in
     * progress, and there might be few changes going forward.
     */
    public VehicleInterfaceData getInterfaceHandle() {
        if (mVehicleService != null) {
            if (mVehicleInterfaceData == null) {
                mVehicleInterfaceData = new VehicleInterfaceData(mVehicleService,
                        this.mClientContext);
            }
        }
        return mVehicleInterfaceData;
    }

    /**
     * An Interface used by VehicleManager to notify the
     * caller about the status of creation.
     * @param handle VehicleManager handle. If an error occurs, handle would be null.
     * Please note that this interface is work in progress, and
     * there might be few changes going forward.
     */
    public interface VehicleManagerCallback {
        public void handleVehicleManagerCreationStatus(VehicleManager handle);
    }

    // store the callback provided by caller
    private void addCallback(VehicleManagerCallback callback) {
        if (mCallbacksVector != null) {
            mCallbacksVector.add(callback);
        }
    }

    /**
     * VehicleManager constructor.
     */
    private VehicleManager(Context mContext) {
        mVehicleService = null;
        mClientContext = mContext;
        mBound = false;
        mCallbacksVector = new Vector();
        Intent i = new Intent("com.qualcomm.qti.ivi.VehicleService");
        PackageManager pm = mContext.getPackageManager();
        List<ResolveInfo> resolveInfoList = pm.queryIntentServices(i, 0);
        if (resolveInfoList == null) {
            Log.e(TAG, "Error querying IntentService for com.qualcomm.qti.ivi.VehicleService");
        } else {
            ResolveInfo resolveInfoNode = resolveInfoList.get(0);
            ComponentName component = new ComponentName(resolveInfoNode.serviceInfo.packageName,
                                                        resolveInfoNode.serviceInfo.name);
            i.setComponent(component);
            mClientContext.bindService(i, mConnection, Context.BIND_AUTO_CREATE);
        }
    }

    /**
     * service connection object to monitor service status.
     */
    private ServiceConnection mConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName className, IBinder service) {
            Log.v(TAG, "onServiceConnected");
            mVehicleService = IVehicleService.Stub.asInterface(service);
            mBound = true;
            if (mCallbacksVector != null) {
                for (Object element : mCallbacksVector) {
                    ((VehicleManagerCallback)element)
                    .handleVehicleManagerCreationStatus(mMyInstance);
                }
            } else {
                Log.e(TAG, "mCallbacksVector is NULL.Could not notify service connection");
            }
        }
        public void onServiceDisconnected(ComponentName className) {
            Log.v(TAG, "onServiceDisconnected");
            mVehicleService = null;
            mBound = false;
            if (mVehicleInterfaceData != null) {
                mVehicleInterfaceData.onVehicleServiceDisconnected();
                mVehicleInterfaceData = null;
            }
        }
    };
};
