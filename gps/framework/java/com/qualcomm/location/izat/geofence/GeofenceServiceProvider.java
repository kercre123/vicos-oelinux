/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015 - 2016 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

package com.qualcomm.location.izat.geofence;

import android.content.Intent;
import android.content.Context;
import android.content.IntentFilter;
import android.content.BroadcastReceiver;
import android.location.Location;
import android.os.IBinder;
import android.os.Binder;
import android.os.Parcelable;
import android.os.RemoteCallbackList;
import android.os.RemoteException;
import android.os.Bundle;
import android.app.ActivityManager;
import android.app.ActivityManager.RunningAppProcessInfo;

import android.util.Log;

import android.app.PendingIntent;

import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Map;

import com.qti.geofence.*;

public class GeofenceServiceProvider {
    private static final String TAG = "GeofenceServiceProvider";
    private static final boolean DEBUG_DBG = Log.isLoggable(TAG, Log.DEBUG);
    private static final int GEOFENCE_RESULT_SUCCESS = 0;
    private static final int GEOFENCE_RESULT_ERROR_TOO_MANY_GEOFENCES = -100;
    private static final int GEOFENCE_RESULT_ERROR_ID_EXISTS = -101;
    private static final int GEOFENCE_RESULT_ERROR_ID_UNKNOWN = -102;
    private static final int GEOFENCE_RESULT_ERROR_INVALID_TRANSITION = -103;
    private static final int GEOFENCE_RESULT_ERROR_GENERIC = -149;

    private static final int GEOFENCE_REQUEST_TYPE_ADD = 1;
    private static final int GEOFENCE_REQUEST_TYPE_REMOVE = 2;
    private static final int GEOFENCE_REQUEST_TYPE_PAUSE = 3;
    private static final int GEOFENCE_REQUEST_TYPE_RESUME = 4;
    private static int sGeofenceId = 1;
    private static final Object sCallBacksLock = new Object();
    private PendingIntent mPendingIntent;
    private final Context mContext;

    private class ClientData {
        private IGeofenceCallback mCallback;
        private PendingIntent mPendingIntent;
        private Map<Integer, GeofenceData> mGeofenceMap;

        ClientData(IGeofenceCallback callback) {
            mCallback = callback;
            mGeofenceMap = new HashMap<Integer, GeofenceData> ();
        }

        ClientData(PendingIntent notifyIntent) {
            mPendingIntent = notifyIntent;
            mGeofenceMap = new HashMap<Integer, GeofenceData> ();
        }
    }

    private Map<Integer, String> mPackagePerProcess;
    private Map<String, ClientData> mClientDataPerPackage;

    private RemoteCallbackList<IGeofenceCallback> mGeofenceCallbacks
        = new RemoteCallbackList<IGeofenceCallback>();

    public static GeofenceServiceProvider sInstance = null;
    public static GeofenceServiceProvider getInstance(Context ctx) {
        if (sInstance == null) {
            sInstance = new GeofenceServiceProvider(ctx);
        }
        return sInstance;
    }

    public GeofenceServiceProvider(Context ctx) {
        if (DEBUG_DBG)
            Log.d(TAG, "GeofenceServiceProvider construction");

        mContext = ctx;
        mPackagePerProcess = new HashMap<Integer, String>();
        mClientDataPerPackage = new HashMap<String , ClientData>();
        native_geofence_init();

        PackageRemovedReceiver pkgRemovedReceiver = new PackageRemovedReceiver();
        IntentFilter filter = new IntentFilter(Intent.ACTION_PACKAGE_REMOVED);
        filter.addDataScheme("package");
        ctx.registerReceiver(pkgRemovedReceiver, filter);
    }

    /* Remote binder */
    private final IGeofenceService.Stub mBinder = new IGeofenceService.Stub() {

        public void registerCallback(final IGeofenceCallback callback) {
            if (callback == null) {
                Log.e(TAG, "callback is null");
                return;
            }
            synchronized (sCallBacksLock) {
                String callingPackage = getPackageName(Binder.getCallingPid());
                if (DEBUG_DBG)
                    Log.d(TAG, "calling package:" + callingPackage);
                if (mClientDataPerPackage.containsKey(callingPackage) == false) {
                    ClientData clData = new ClientData(callback);
                    mClientDataPerPackage.put(callingPackage, clData);
                } else {
                    ClientData clData = mClientDataPerPackage.get(callingPackage);
                    if (clData.mCallback != null) {
                        mGeofenceCallbacks.unregister(clData.mCallback);
                    }
                    clData.mCallback = callback;
                }
                mGeofenceCallbacks.register(callback);
            }
            try {
                callback.asBinder().linkToDeath(new IBinder.DeathRecipient() {
                    @Override
                    public void binderDied() {
                        synchronized (sCallBacksLock) {
                            ClientData clData = null;
                            String ownerPackage = null;
                            for (Map.Entry<String, ClientData> entry :
                                    mClientDataPerPackage.entrySet()) {
                                clData = entry.getValue();
                                if (clData.mCallback == callback) {
                                    ownerPackage = entry.getKey();
                                    clData.mCallback = null;
                                    break;
                                }
                            }
                            if ((clData != null) && (clData.mPendingIntent == null)) {
                                if (DEBUG_DBG)
                                    Log.d(TAG, "Client died:" + ownerPackage +
                                        " remove all geofences");
                                removeAllGeofences(ownerPackage);
                            } else {
                                if (DEBUG_DBG)
                                    Log.d(TAG, "Client died:" + ownerPackage +
                                        " notify on breach");
                            }
                            mGeofenceCallbacks.unregister(callback);
                        }
                    }
                }, 0);
            } catch (RemoteException e) {
                throw new RuntimeException("Failed unregister geofence cb", e);
            }
        }

        public void unregisterCallback(IGeofenceCallback callback) {
            if (callback == null) {
                Log.e(TAG, "callback is null");
                return;
            }
            synchronized (sCallBacksLock) {
                String ownerPackage = getPackageName(Binder.getCallingPid());
                ClientData clData = mClientDataPerPackage.get(ownerPackage);
                if (clData != null){
                    clData.mCallback  = null;
                    if (clData.mPendingIntent == null) {
                        removeAllGeofences(ownerPackage);
                    }
                }
                mGeofenceCallbacks.unregister(callback);
            }
        }

        public void registerPendingIntent(final PendingIntent notifyIntent) {
            if (notifyIntent == null) {
                Log.e(TAG,
                    "in registerPendingIntent() notifyIntent is null");
            }

            String callingPackage = notifyIntent.getCreatorPackage();
            if (DEBUG_DBG) {
                Log.d(TAG,
                    "registerPendingIntent() for package:" + callingPackage);
            }
            if (mClientDataPerPackage.containsKey(callingPackage) == false) {
                ClientData clData = new ClientData(notifyIntent);
                mClientDataPerPackage.put(callingPackage, clData);
            } else {
                  ClientData clData = mClientDataPerPackage.get(callingPackage);
                  clData.mPendingIntent = notifyIntent;
            }
        }

        public void unregisterPendingIntent(PendingIntent notifyIntent) {
            if (notifyIntent == null) {
                Log.e(TAG,
                    "in unregisterPendingIntent() notifyIntent is null");
            }


            String ownerPackage = notifyIntent.getCreatorPackage();
            if (DEBUG_DBG) {
                Log.d(TAG,
                    "unregisterPendingIntent() for package:" + ownerPackage);
            }
            if (mClientDataPerPackage.containsKey(ownerPackage)) {
                ClientData clData = mClientDataPerPackage.get(ownerPackage);
                if (clData != null) {
                    clData.mPendingIntent = null;
                    if (clData.mCallback == null) {
                        removeAllGeofences(ownerPackage);
                    }
                }
            }
        }

        public int addGeofence(double latitude,
                               double longitude,
                               double radius, // in meters
                               int transitionTypes,
                               int responsiveness, // in milliseconds
                               int confidence,
                               int dwellTime, // in seconds
                               int dwellTimeMask) {

            final int geofenceId = sGeofenceId++;

            String callingPackage = getPackageName(Binder.getCallingPid());
            if (mClientDataPerPackage.containsKey(callingPackage)) {
                ClientData clData = mClientDataPerPackage.get(callingPackage);
                GeofenceData gfData = new GeofenceData(responsiveness, latitude, longitude,
                    radius, transitionTypes, confidence, dwellTimeMask, dwellTime,
                    null, null, geofenceId);
                clData.mGeofenceMap.put(geofenceId, gfData);
            }

            if (DEBUG_DBG) {
                Log.d(TAG, "in IGeofenceService.Stub(): addGeofence()" +
                           "; Calling package is " + callingPackage +
                           "; geofenceId is " + geofenceId +
                           "; latitude is " + latitude +
                           "; longitude is " + longitude +
                           "; radius is " + radius +
                           "; transitionTypes is " + transitionTypes +
                           "; responsiveness is " + responsiveness +
                           "; confidence is " + confidence +
                           "; dwellTime is " + dwellTime +
                           "; dwellTimeMask is " + dwellTimeMask);
            }
            native_add_geofence(geofenceId, latitude, longitude, radius,
                                transitionTypes, responsiveness, confidence,
                                dwellTime, dwellTimeMask);
            return geofenceId;
        }

        public int addGeofenceObj(GeofenceData gfData) {
            if (gfData == null) {
                Log.e(TAG,
                    "in addGeofence() gfData is null");
            }

            double latitude = gfData.getLatitude();
            double longitude = gfData.getLongitude();
            double radius = gfData.getRadius();
            int transitionTypes = gfData.getTransitionType().getValue();
            int responsiveness = gfData.getNotifyResponsiveness();
            int confidence = gfData.getConfidence().getValue();
            int dwellTime = gfData.getDwellTime();
            int dwellTimeMask = gfData.getDwellType().getValue();
            Object appBundleData = gfData.getAppBundleData();
            String appTextData = gfData.getAppTextData();

            final int geofenceId = sGeofenceId++;

            String callingPackage = getPackageName(Binder.getCallingPid());
            if (mClientDataPerPackage.containsKey(callingPackage)) {
                ClientData clData = mClientDataPerPackage.get(callingPackage);
                // geofence Id is provided by service, hence set it here.
                gfData.setGeofenceId(geofenceId);
                clData.mGeofenceMap.put(geofenceId, gfData);
            }

            if (DEBUG_DBG) {
                Log.d(TAG, "in IGeofenceService.Stub(): addGeofence()" +
                           "; Calling package is " + callingPackage +
                           "; geofenceId is " + geofenceId +
                           "; latitude is " + latitude +
                           "; longitude is " + longitude +
                           "; radius is " + radius +
                           "; transitionTypes is " + transitionTypes +
                           "; responsiveness is " + responsiveness +
                           "; confidence is " + confidence +
                           "; dwellTime is " + dwellTime +
                           "; dwellTimeMask is " + dwellTimeMask +
                           "; appTextData is " + appTextData);
            }
            native_add_geofence(geofenceId, latitude, longitude, radius,
                                transitionTypes, responsiveness, confidence,
                                dwellTime, dwellTimeMask);
            return geofenceId;
        }

        public void removeGeofence(int geofenceId) {
            String callingPackage = getPackageName(Binder.getCallingPid());
            if (DEBUG_DBG) {
                Log.d(TAG, "in IGeofenceService.Stub(): removeGeofence()" +
                           "; Calling package is " + callingPackage +
                           "; geofenceId is " + geofenceId);
            }

            if (mClientDataPerPackage.containsKey(callingPackage)) {
                ClientData clData = mClientDataPerPackage.get(callingPackage);
                clData.mGeofenceMap.remove(Integer.valueOf(geofenceId));
            }

            native_remove_geofence(geofenceId);
        }

        public void updateGeofence(int geofenceId,
                                   int transitionTypes,
                                   int notifyResponsiveness) {
            String callingPackage = getPackageName(Binder.getCallingPid());
            if (DEBUG_DBG) {
                Log.d(TAG, "in IGeofenceService.Stub(): updateGeofence()" +
                           "; Calling package is " + callingPackage +
                           "; geofenceId is " + geofenceId);
            }

            if (mClientDataPerPackage.containsKey(callingPackage)) {
                ClientData clData = mClientDataPerPackage.get(callingPackage);
                if (clData.mGeofenceMap.containsKey(geofenceId)) {
                    GeofenceData gfData =
                        clData.mGeofenceMap.get(Integer.valueOf(geofenceId));
                    gfData.setTransitionType(transitionTypes);
                    gfData.setNotifyResponsiveness(notifyResponsiveness);
                }
            }

            native_update_geofence(geofenceId,
                                   transitionTypes,
                                   notifyResponsiveness);
        }

        public void pauseGeofence(int geofenceId) {
            if (DEBUG_DBG) {
                Log.d(TAG, "in IGeofenceService.Stub(): pauseGeofence()" +
                           "; geofenceId is " + geofenceId);
            }
            native_pause_geofence(geofenceId);
        }

        public void resumeGeofence(int geofenceId) {
            String callingPackage = getPackageName(Binder.getCallingPid());
            if (DEBUG_DBG) {
                Log.d(TAG, "in IGeofenceService.Stub(): resumeGeofence()" +
                           "; Calling package is " + callingPackage +
                           "; geofenceId is " + geofenceId);
            }

            if (mClientDataPerPackage.containsKey(callingPackage)) {
                ClientData clData = mClientDataPerPackage.get(callingPackage);
                if (clData.mGeofenceMap.containsKey(geofenceId)) {
                    GeofenceData gfData =
                        clData.mGeofenceMap.get(Integer.valueOf(geofenceId));

                        GeofenceData.GeofenceTransitionTypes transitionTypes;
                        transitionTypes = gfData.getTransitionType();

                        // resume the geofence with the original transition type
                        native_resume_geofence(geofenceId, transitionTypes.getValue());
                }
            }
        }

        public void recoverGeofences(List<GeofenceData> gfList) {
             String callingPackage = getPackageName(Binder.getCallingPid());
             if (DEBUG_DBG) {
                Log.d(TAG, "in IGeofenceService.Stub(): recoverGeofences()" +
                           "; Calling package is " + callingPackage);
             }

             if (mClientDataPerPackage.containsKey(callingPackage)) {
                ClientData clData = mClientDataPerPackage.get(callingPackage);
                gfList.addAll(clData.mGeofenceMap.values());
             }
        }
    };

    private void reportGeofenceTransition(int geofenceId,
                                          int transition,
                                          Location location) {
        if (DEBUG_DBG)
            Log.d(TAG, "reportGeofenceTransition id : " + geofenceId +
                       "; transition : " + transition);
        synchronized (sCallBacksLock) {
            for (Map.Entry<String, ClientData> entry :
                                    mClientDataPerPackage.entrySet()) {
                ClientData clData = entry.getValue();
                if (clData.mGeofenceMap.containsKey(geofenceId)) {
                    if (DEBUG_DBG) {
                        Log.d(TAG, "Sending breach event to: " +
                            entry.getKey());
                    }

                    try {
                        if (clData.mPendingIntent != null) {
                            Intent pdIntent = new Intent();
                            GeofenceData gfData = clData.mGeofenceMap.get(geofenceId);
                            String gfCtxStringData = gfData.getAppTextData();
                            if (gfCtxStringData != null) {
                                pdIntent.putExtra("context-data", gfCtxStringData);
                            } else {
                                Bundle gfCtxBundleData = gfData.getAppBundleData();
                                if (gfCtxBundleData != null) {
                                    pdIntent.putExtra("context-data", gfCtxBundleData);
                                }
                            }
                            pdIntent.putExtra("transition-location", location);
                            pdIntent.putExtra("transition-event", transition);

                            clData.mPendingIntent.send(mContext, 0, pdIntent);
                        }
                    } catch (PendingIntent.CanceledException e) {
                        clData.mPendingIntent = null;
                    }

                    try {
                        if (clData.mCallback != null) {
                            clData.mCallback.onTransitionEvent(geofenceId,
                                                               transition,
                                                               location);
                        }
                    } catch (RemoteException e) {
                        clData.mCallback = null;
                    }

                    if ((clData.mPendingIntent == null) &&
                        (clData.mCallback == null)) {
                        removeAllGeofences(entry.getKey());
                    }

                    break;
                }
            };
        }
    }

    private void reportGeofenceRequestStatus(int requestType,
                                             int geofenceId,
                                             int status) {
        if (DEBUG_DBG)
            Log.d(TAG, "reportGeofenceRequestStatus requestType: " +
                       requestType +"; id : " + geofenceId +
                       "; status : " + status);
        synchronized (sCallBacksLock) {
            for (ClientData clData: mClientDataPerPackage.values()) {
                if (clData.mGeofenceMap.containsKey(geofenceId)) {
                    try {
                        clData.mCallback.onRequestResultReturned(geofenceId,
                                                                 requestType,
                                                                 status);
                        if ((GEOFENCE_REQUEST_TYPE_ADD == requestType) &&
                            (GEOFENCE_RESULT_SUCCESS != status)) {
                            clData.mGeofenceMap.remove(Integer.valueOf(geofenceId));
                        }
                    } catch (RemoteException e) {
                        // do nothing
                    }
                }
            };
        }
    }

    private void reportGeofenceStatus(int status,
                                      Location location) {
        if (DEBUG_DBG)
            Log.d(TAG, "reportGeofenceStatus - status : " + status);
        synchronized (sCallBacksLock) {
            int index = mGeofenceCallbacks.beginBroadcast();
            for (int i = 0; i < index; i++) {
                try {
                    mGeofenceCallbacks.getBroadcastItem(i).onEngineReportStatus(status, location);
                } catch (RemoteException e) {
                    // do nothing
                }
            }
            mGeofenceCallbacks.finishBroadcast();
        }
    }

    private void removeAllGeofences(String ownerPackage)
    {
        Log.d(TAG, "removing all geofences for package: " + ownerPackage);
        if (mClientDataPerPackage.containsKey(ownerPackage)) {

            ClientData clData = mClientDataPerPackage.get(ownerPackage);
            for(Integer key : clData.mGeofenceMap.keySet()) {
                native_remove_geofence(key);
            }
            clData.mGeofenceMap.clear();
        }
        mClientDataPerPackage.remove(ownerPackage);
    }

    public IGeofenceService getGeofenceBinder() {
        return mBinder;
    }

    private String getPackageName(int pid)
    {
        String packageName = mPackagePerProcess.get(pid);
        if (packageName == null) {
            ActivityManager am = (ActivityManager) mContext.getSystemService(Context.ACTIVITY_SERVICE);
            List<RunningAppProcessInfo> infos = am.getRunningAppProcesses();
            if (infos != null && infos.size() > 0) {
                for(RunningAppProcessInfo info : infos) {
                    if(info.pid == pid) {
                        // NOTE: Use the first package name found in that process.
                        mPackagePerProcess.put(pid, info.pkgList[0]);
                        return info.pkgList[0];
                    }
                }
            }
        }
        return packageName;
    }

    public class PackageRemovedReceiver extends BroadcastReceiver {
        @Override
        public void onReceive(Context conext, Intent intent) {
            synchronized (sCallBacksLock)  {
                 Log.d(TAG, "Package uninstalled, removing its geofences: " +
                     intent.getData().getSchemeSpecificPart());
                removeAllGeofences( intent.getData().getSchemeSpecificPart());
            }
        }
    }

    static {
        native_geofence_class_init();
    }

    private static native void native_geofence_class_init();
    private native void native_geofence_init();
    private native void native_add_geofence(int geofenceId,
                                            double latitude,
                                            double longitude,
                                            double radius,
                                            int transitionTypes,
                                            int notificationResponsivenes,
                                            int confidence,
                                            int dwellTime,
                                            int dwellTimeMask);
    private native void native_remove_geofence(int geofenceId);
    private native void native_update_geofence(int geofenceId,
                                               int transitionTypes,
                                               int notifyResponsiveness);
    private native void native_pause_geofence(int geofenceId);
    private native void native_resume_geofence(int geofenceId,
                                               int transitionTypes);
}
