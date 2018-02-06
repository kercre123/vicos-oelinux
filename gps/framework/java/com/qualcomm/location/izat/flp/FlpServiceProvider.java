/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

package com.qualcomm.location.izat.flp;

import android.app.Service;
import android.content.Intent;
import android.content.Context;
import android.location.Location;
import android.os.IBinder;
import android.os.RemoteCallbackList;
import android.os.RemoteException;
import android.os.ServiceManager;
import android.util.Log;
import android.util.Pair;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

import com.qti.flp.*;

public class FlpServiceProvider {
    private static final String TAG = "FlpServiceProvider";
    private static final boolean VERBOSE_DBG = Log.isLoggable(TAG, Log.VERBOSE);
    private static final int FLP_RESULT_SUCCESS = 0;
    private static final int FLP_RESULT_ERROR = -1;
    private static final int LOCATION_REPORT_ON_FULL_INDICATION = 1;
    private static final int LOCATION_REPORT_ON_FIX_INDICATION = 2;
    private static final int LOCATION_REPORT_ON_QUERY = 4;
    private static final int LOCATION_REPORT_ON_INDICATIONS = 8; // for legacy
    private static final int FLP_PASSIVE_LISTENER_HW_ID = -1;
    private static final int FEATURE_BIT_DISTANCE_BASED_BATCHING_IS_SUPPORTED = 8;
    private static final Object sCallBacksLock = new Object();
    private static final Object sStatusCallbacksLock = new Object();
    private final Context mContext;

    private static final int FLP_SESSION_BACKGROUND = 1;
    private static final int FLP_SESSION_FOREGROUND = 2;
    private static final int FLP_SESSION_PASSIVE = 4;

    private static final int FLP_BG_NOTIFICATION_ROUTINE = 1;
    private static final int FLP_BG_NOTIFICATION_OUTDOOR_TRIP = 3;

    private static final int BATCHING_STATUS_TRIP_COMPLETED = 0;
    private static final int BATCHING_STATUS_POSITION_AVAILABE = 1;
    private static final int BATCHING_STATUS_POSITION_UNAVAILABLE = 2;

    private static final int BATCHING_MODE_ROUTINE = 0;
    private static final int BATCHING_MODE_OUTDOOR_TRIP = 1;

    private class BgSessionData {
        private int mBgNotificationType;
        private long mSessionStartTime;

        BgSessionData() {
            mBgNotificationType = FLP_BG_NOTIFICATION_ROUTINE;
            mSessionStartTime = 0;
        }
   }

    private Queue<Pair<ILocationCallback,Long>> mQueryCbQueue
            = new LinkedList<Pair<ILocationCallback,Long>>();
    private RemoteCallbackList<ILocationCallback> mCallbacksForBg
            = new RemoteCallbackList<ILocationCallback>();
    private RemoteCallbackList<ILocationCallback> mCallbacksForFg
            = new RemoteCallbackList<ILocationCallback>();
    private RemoteCallbackList<ILocationCallback> mCallbacksForPassive
            = new RemoteCallbackList<ILocationCallback>();
    private RemoteCallbackList<IMaxPowerAllocatedCallback> mMaxPowerCallbacks
            = new RemoteCallbackList<IMaxPowerAllocatedCallback>();
    private RemoteCallbackList<ISessionStatusCallback> mCallbacksForStatus
            = new RemoteCallbackList<ISessionStatusCallback>();
    private Map<Integer, BgSessionData> mBgSessionMap;

    public static FlpServiceProvider sInstance = null;
    public static FlpServiceProvider getInstance(Context ctx) {
        if (sInstance == null) {
            sInstance = new FlpServiceProvider(ctx);
        }
        return sInstance;
    }
    private int mFlpFeaturMasks = -1;

    public FlpServiceProvider(Context ctx) {
        if (VERBOSE_DBG) {
            Log.d(TAG, "FlpServiceProvider construction");
        }
        mContext = ctx;
        mBgSessionMap = new HashMap<Integer, BgSessionData>();
        if (native_flp_init() != FLP_RESULT_SUCCESS) {
            Log.e(TAG, "native flp init failed");
        }
    }

    /* Remote binder */
    private final IFlpService.Stub mBinder = new IFlpService.Stub() {
        public void registerCallback(final int sessionType,
                                     final int id,
                                     final ILocationCallback cb,
                                     final long sessionStartTime) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): registerCallback()," +
                           " sessionType is " + sessionType + "; id is " + id +
                           "; sessionStartTime is " + sessionStartTime +
                           "; cb:" + cb);
            }
            if (cb != null) {
                if (mCallbacksForBg != null &&
                    mCallbacksForFg != null &&
                    mCallbacksForPassive != null) {
                    switch (sessionType) {
                        case FLP_SESSION_BACKGROUND:
                            synchronized (sCallBacksLock) {
                                mCallbacksForBg.register(cb, id);

                                BgSessionData bgSessData;
                                if (mBgSessionMap.containsKey(id)) {
                                    bgSessData = mBgSessionMap.get(id);
                                } else {
                                    bgSessData = new BgSessionData();
                                }

                                bgSessData.mSessionStartTime = sessionStartTime;
                                mBgSessionMap.put(id, bgSessData);
                            }
                            try {
                                cb.asBinder().linkToDeath(new IBinder.DeathRecipient() {
                                    @Override
                                    public void binderDied() {
                                        synchronized(sCallBacksLock) {
                                            mCallbacksForBg.unregister(cb);
                                            mBgSessionMap.remove(id);
                                            stopFlpSession(id);
                                        }
                                    }
                                }, 0);
                            } catch (RemoteException e) {
                                throw new RuntimeException("Failed clean up flp sessions", e);
                            }
                            break;
                        case FLP_SESSION_FOREGROUND:
                            synchronized (sCallBacksLock) {
                                mCallbacksForFg.register(cb, sessionStartTime);
                            }
                            try {
                                cb.asBinder().linkToDeath(new IBinder.DeathRecipient() {
                                    @Override
                                    public void binderDied() {
                                        synchronized(sCallBacksLock) {
                                            mCallbacksForFg.unregister(cb);
                                            stopFlpSession(id);
                                        }
                                    }
                                }, 0);
                            } catch (RemoteException e) {
                                throw new RuntimeException("Failed clean up flp sessions", e);
                            }
                            break;
                        case FLP_SESSION_PASSIVE:
                            synchronized (sCallBacksLock) {
                                mCallbacksForPassive.register(cb, sessionStartTime);
                            }
                            try {
                                cb.asBinder().linkToDeath(new IBinder.DeathRecipient() {
                                    @Override
                                    public void binderDied() {
                                        synchronized(sCallBacksLock) {
                                            mCallbacksForPassive.unregister(cb);
                                            if (id != FLP_PASSIVE_LISTENER_HW_ID) {
                                                stopFlpSession(id);
                                            }
                                        }
                                    }
                                }, 0);
                            } catch (RemoteException e) {
                                throw new RuntimeException("Failed unregister passive cb", e);
                            }
                            break;
                        default:
                            Log.e(TAG, "unknown sessionType");
                            break;
                    }
                } else {
                    Log.e(TAG, "one of the callback list is not created");
                }
            } else {
                Log.e(TAG, "cb is null");
            }
        }

        public void unregisterCallback(final int sessionType,
                                       final ILocationCallback cb) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): unregisterCallback(): cb:" + cb);
            }
            if (cb != null) {
                if (mCallbacksForBg != null &&
                    mCallbacksForFg != null &&
                    mCallbacksForPassive != null) {
                    synchronized (sCallBacksLock) {
                        switch (sessionType) {
                            case FLP_SESSION_BACKGROUND:
                                mCallbacksForBg.unregister(cb);
                                break;
                            case FLP_SESSION_FOREGROUND:
                                mCallbacksForFg.unregister(cb);
                                break;
                            case FLP_SESSION_PASSIVE:
                                mCallbacksForPassive.unregister(cb);
                                break;
                            default:
                                Log.e(TAG, "unknown sessionType");
                                break;
                        }
                    }
                } else {
                    Log.e(TAG, "one of the callback list is not created");
                }
            } else {
                Log.e(TAG, "cb is null");
            }
        }

        public void registerForSessionStatus(final int id, final ISessionStatusCallback cb) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): registerForSessionStatus() cb:" + cb);
            }
            if (cb != null) {
                if (mCallbacksForStatus != null) {
                    synchronized (sStatusCallbacksLock) {
                        mCallbacksForStatus.register(cb, id);
                    }
                    try {
                        cb.asBinder().linkToDeath(new IBinder.DeathRecipient() {
                            @Override
                            public void binderDied() {
                                synchronized(sStatusCallbacksLock) {
                                    mCallbacksForStatus.unregister(cb);
                                }
                            }
                        }, 0);
                    } catch (RemoteException e) {
                        throw new RuntimeException("Failed clean up flp sessions", e);
                    }
                } else {
                      Log.e(TAG, "cb is null");
                }
            }
        }

        public void unregisterForSessionStatus(final ISessionStatusCallback cb) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in FlpService.Stub unregisterForSessionStatus() cb = : " + cb);
            }
            if (cb != null) {
                if (mCallbacksForStatus != null) {
                    synchronized(sStatusCallbacksLock) {
                        mCallbacksForStatus.unregister(cb);
                    }
                }
            } else {
                Log.e(TAG, "cb is null");
            }
        }

        public int getAllSupportedFeatures() {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): getAllSupportedFeatures()");
            }
            if (mFlpFeaturMasks == -1) {
                mFlpFeaturMasks = native_flp_get_all_supported_features();
            }
            return mFlpFeaturMasks;
        }

        public int startFlpSession(int id,
                                   int flags,
                                   long period_ms,
                                   int distance_interval_mps,
                                   long trip_distance_m) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): startFlpSession()" +
                           "; id is " + id +
                           "; period_ms is " + period_ms +
                           "; flags is " + flags +
                           "; distance interval is " + distance_interval_mps +
                           "; trip distance is" + trip_distance_m);
            }

            // BgSessionMap only cares about BG sessions i.e. routine / trip
            synchronized(sCallBacksLock) {
                if ((flags == FLP_BG_NOTIFICATION_ROUTINE) ||
                    (flags == FLP_BG_NOTIFICATION_OUTDOOR_TRIP)) {
                    BgSessionData bgSessData = mBgSessionMap.get(id);

                    if (bgSessData != null) {
                        bgSessData.mBgNotificationType = flags;
                    } else {
                        Log.e(TAG, "No registered callback for this session id.");
                    }
                }
            }

            long period_ns =
                (period_ms > (Long.MAX_VALUE/1000000)) ? Long.MAX_VALUE : period_ms*1000000;
            return native_flp_start_session(id,
                                            flags,
                                            period_ns,
                                            distance_interval_mps);
        }

        public int updateFlpSession(int id,
                                    int flags,
                                    long period_ms,
                                    int distance_interval_mps,
                                    long trip_distance_m) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): updateFlpSession()" +
                           "; id is " + id +
                           "; period_ms is " + period_ms +
                           "; flags is " + flags +
                           "; distance_ms is " + distance_interval_mps +
                           "; trip distance " + trip_distance_m);
            }

            // BgSessionMap only cares about BG sessions i.e. routine / trip
            synchronized(sCallBacksLock) {
                BgSessionData bgSessData = mBgSessionMap.get(id);

                if ((flags == FLP_BG_NOTIFICATION_ROUTINE) ||
                    (flags == FLP_BG_NOTIFICATION_OUTDOOR_TRIP)) {
                    if (bgSessData != null) {
                        bgSessData.mBgNotificationType = flags;
                    } else {
                        // may be the update is happening from a foreground session,
                        // hence callback will be registered after the update call
                        bgSessData = new BgSessionData();
                        bgSessData.mBgNotificationType = flags;
                        mBgSessionMap.put(id, bgSessData);
                    }
                }
            }

            long period_ns =
                (period_ms > (Long.MAX_VALUE/1000000)) ? Long.MAX_VALUE : period_ms*1000000;
            return native_flp_update_session(id,
                                             flags,
                                             period_ns,
                                             distance_interval_mps);
        }

        public int stopFlpSession(int id) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): stopFlpSession(); id is " + id);
            }

            synchronized(sCallBacksLock) {
                mBgSessionMap.remove(id);
            }

            return native_flp_stop_session(id);
        }

        public int pullLocations(final ILocationCallback cb,
                                 final long sessionStartTime,
                                 final int id) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): pullLocations(), sessionStartTime is "
                           + sessionStartTime);
            }
            if (cb == null) {
                Log.e(TAG, "in IFlpService.Stub(): cb is null.");
                return FLP_RESULT_ERROR;
            }
            synchronized (sCallBacksLock) {
                // save the cb
                mQueryCbQueue.add(new Pair<ILocationCallback,Long>(cb, sessionStartTime));
            }
            return native_flp_get_all_locations();
        }
    };

    /* Remote binder */
    private final ITestService.Stub mTestingBinder = new ITestService.Stub() {
        public void deleteAidingData(long flags) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): deleteAidingData(). Flags: " + flags);
            }
            native_flp_delete_aiding_data(flags);
        }

        public void registerMaxPowerChangeCallback(final IMaxPowerAllocatedCallback cb) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): registerMaxPowerChangeCallback()");
            }
            if (cb != null) {
                if (mMaxPowerCallbacks != null) {
                    mMaxPowerCallbacks.register(cb);
                    try {
                        cb.asBinder().linkToDeath(new IBinder.DeathRecipient() {
                            @Override
                            public void binderDied() {
                                mMaxPowerCallbacks.unregister(cb);
                            }
                        }, 0);
                        cb.onMaxPowerAllocatedChanged(native_flp_get_max_power_allocated_in_mw());
                    } catch (RemoteException e) {
                        throw new RuntimeException("Failed clean up", e);
                    }
                } else {
                    Log.e(TAG, "mMaxPowerCallbacks is null");
                }
            } else {
                Log.e(TAG, "cb is null");
            }
        }

        public void unregisterMaxPowerChangeCallback(IMaxPowerAllocatedCallback cb) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in IFlpService.Stub(): unregisterMaxPowerChangeCallback()");
            }
            if (cb != null) {
                if (mMaxPowerCallbacks != null) {
                    mMaxPowerCallbacks.unregister(cb);
                } else {
                    Log.e(TAG, "mMaxPowerCallbacks is null");
                }
            } else {
                Log.e(TAG, "cb is null");
            }
        }
    };

    private void onLocationReport(Location[] locations, int reportTrigger, int batchingMode) {
        if (VERBOSE_DBG) {
            Log.d(TAG, "entering onLocationReport() reportTrigger is " + reportTrigger +
                       "; Batching Mode is " + batchingMode +
                       "; and the first timestamp is " + locations[0].getTime());
        }
        if (reportTrigger == LOCATION_REPORT_ON_FULL_INDICATION) {
            // Broadcast to all batching callbacks the new value.
            synchronized (sCallBacksLock) {
                int index = mCallbacksForBg.beginBroadcast();
                for (int i = 0; i < index; i++) {
                    try {
                        int sessionId = (Integer) mCallbacksForBg.getBroadcastCookie(i);
                        BgSessionData bgSessData = mBgSessionMap.get(sessionId);
                        if (bgSessData == null) {
                            Log.e(TAG, "No Background session data found for id:" + index);
                            continue;
                        }
                        long sessionStartTime = (long) bgSessData.mSessionStartTime;

                        if (VERBOSE_DBG) {
                            Log.d(TAG, "in the mCallbacksForBg loop : " + i +
                                "; timestamp is " + sessionStartTime +
                                "; notification Type is " + bgSessData.mBgNotificationType);
                        }
                        if (((bgSessData.mBgNotificationType == FLP_BG_NOTIFICATION_ROUTINE) &&
                            (batchingMode == BATCHING_MODE_ROUTINE)) ||
                            (bgSessData.mBgNotificationType == FLP_BG_NOTIFICATION_OUTDOOR_TRIP) &&
                            (batchingMode == BATCHING_MODE_OUTDOOR_TRIP)) {
                            if (sessionStartTime<=locations[0].getTime()) {
                                // return the whole batch
                                if (VERBOSE_DBG) {
                                    Log.d(TAG, "return whole batch");
                                }
                                mCallbacksForBg.getBroadcastItem(i).onLocationAvailable(locations);
                            } else if (sessionStartTime>locations[locations.length-1].getTime()) {
                                if (VERBOSE_DBG) {
                                    Log.d(TAG, "no need to return");
                                }
                            } else {
                                // find the offset
                                int offset = getOffset(sessionStartTime, locations);
                                Location[] newResult = new Location[locations.length - offset];
                                System.arraycopy(locations,
                                                 offset,
                                                 newResult,
                                                 0,
                                                 locations.length - offset);
                                mCallbacksForBg.getBroadcastItem(i).onLocationAvailable(newResult);
                            }
                        }
                    } catch (RemoteException e) {
                    // The RemoteCallbackList will take care of removing
                    // the dead object.
                    }
                }
                mCallbacksForBg.finishBroadcast();
            }
         } else if (reportTrigger == LOCATION_REPORT_ON_FIX_INDICATION) {
            // Broadcast to all tracking callbacks the new value.
            synchronized (sCallBacksLock) {
                int index = mCallbacksForFg.beginBroadcast();
                for (int i = 0; i < index; i++) {
                    try {
                        long sessionStartTime = (long) mCallbacksForFg.getBroadcastCookie(i);
                        if (VERBOSE_DBG) {
                            Log.d(TAG, "in the mCallbacksForFg loop : " + i
                                       + "; cd timestamp is" + sessionStartTime);
                        }
                        if (sessionStartTime<=locations[0].getTime()) {
                            // return the whole batch
                            if (VERBOSE_DBG) {
                                Log.d(TAG, "return whole batch");
                            }
                            mCallbacksForFg.getBroadcastItem(i).onLocationAvailable(locations);
                        } else if (sessionStartTime>locations[locations.length-1].getTime()) {
                            if (VERBOSE_DBG) {
                                Log.d(TAG, "no need to return");
                            }
                        } else {
                            // find the offset
                            int offset = getOffset(sessionStartTime, locations);
                            Location[] newResult = new Location[locations.length - offset];
                            System.arraycopy(locations,
                                             offset,
                                             newResult,
                                             0,
                                             locations.length - offset);
                            mCallbacksForFg.getBroadcastItem(i).onLocationAvailable(newResult);
                        }
                    } catch (RemoteException e) {
                        // The RemoteCallbackList will take care of removing
                        // the dead object.
                    }
                }
                mCallbacksForFg.finishBroadcast();
            }
        } else if (reportTrigger == LOCATION_REPORT_ON_QUERY) {
            synchronized (sCallBacksLock) {
                if (!mQueryCbQueue.isEmpty()) {
                    Pair<ILocationCallback,Long> cbPair = mQueryCbQueue.remove();
                    if (cbPair != null) {
                        try {
                            // check the timestamp, find the offset
                            ILocationCallback callback = cbPair.first;
                            long sessionStartTime = cbPair.second;
                            if (VERBOSE_DBG) {
                                Log.d(TAG, "calling callback for" +
                                           " pulling, sessionStartTime is " +
                                            sessionStartTime);
                            }
                            if (sessionStartTime<=locations[0].getTime()) {
                                // return the whole batch
                                if (VERBOSE_DBG) {
                                    Log.d(TAG, "return whole batch");
                                }
                                callback.onLocationAvailable(locations);
                            } else if (sessionStartTime>locations[locations.length-1].getTime()) {
                                if (VERBOSE_DBG) {
                                    Log.d(TAG, "no need to return");
                                }
                            } else {
                                int offset = getOffset(sessionStartTime, locations);
                                Location[] newResult = new Location[locations.length - offset];
                                System.arraycopy(locations,
                                                 offset,
                                                 newResult,
                                                 0,
                                                 locations.length - offset);
                                callback.onLocationAvailable(newResult);
                            }
                            // update the timestamp of the callback
                            if (mCallbacksForBg.unregister(callback)) {
                                mCallbacksForBg.register(callback, System.currentTimeMillis());
                            }
                            if (mCallbacksForFg.unregister(callback)) {
                                mCallbacksForFg.register(callback, System.currentTimeMillis());
                            }
                            if (mCallbacksForPassive.unregister(callback)) {
                                mCallbacksForPassive.register(callback, System.currentTimeMillis());
                            }
                        } catch (RemoteException e) {
                            // The RemoteCallbackList will take care of removing
                            // the dead object.
                        }
                    }
                } else {
                    Log.e(TAG, "no available callback on query");
                }
            }
        } else if (reportTrigger == LOCATION_REPORT_ON_INDICATIONS) {
            /*
               For legacy behaviour, the callback are in the passive
               listeners already, so do nothing here.
            */
        } else {
            Log.e(TAG, "unknown reportTrigger");
        }

        // passive listeners
        if (mCallbacksForPassive.getRegisteredCallbackCount() > 0) {
            // Broadcast to all passive listeners
            synchronized (sCallBacksLock) {
                int index = mCallbacksForPassive.beginBroadcast();
                for (int i = 0; i < index; i++) {
                    try {
                        long sessionStartTime = (long) mCallbacksForPassive.getBroadcastCookie(i);
                        if (VERBOSE_DBG) {
                            Log.d(TAG, "in the mCallbacksForPassive loop : " + i
                                       + "; cd timestamp is" + sessionStartTime);
                        }
                        if (sessionStartTime<=locations[0].getTime()) {
                            // return the whole batch
                            if (VERBOSE_DBG) {
                                Log.d(TAG, "return whole batch");
                            }
                            mCallbacksForPassive.getBroadcastItem(i).onLocationAvailable(locations);
                        } else if (sessionStartTime>locations[locations.length-1].getTime()) {
                            if (VERBOSE_DBG) {
                                Log.d(TAG, "no need to return");
                            }
                        } else {
                            // find the offset
                            int offset = getOffset(sessionStartTime, locations);
                            Location[] newResult = new Location[locations.length - offset];
                            System.arraycopy(locations,
                                             offset,
                                             newResult,
                                             0,
                                             locations.length - offset);
                            mCallbacksForPassive.getBroadcastItem(i).onLocationAvailable(newResult);
                        }
                    } catch (RemoteException e) {
                        // The RemoteCallbackList will take care of removing
                        // the dead object.
                    }
                }
                mCallbacksForPassive.finishBroadcast();
            }
        }
    }

    private void onBatchingStatusCb(int batchingStatus, int[] completedTripClientIds) {
        if (VERBOSE_DBG) {
            Log.d(TAG, "entering onBatchingStatusCb batchingStatus :" + batchingStatus);
        }
        synchronized(sStatusCallbacksLock) {
            List<Integer> completedTripClientList = null;

            if (completedTripClientIds != null) {
                completedTripClientList = new ArrayList<Integer>(completedTripClientIds.length);
                for (int index = 0; index < completedTripClientIds.length; index++) {
                    completedTripClientList.add(completedTripClientIds[index]);
                }
            }

            int broadcastCount = mCallbacksForStatus.beginBroadcast();
            for (int broadcastIndex = 0; broadcastIndex < broadcastCount; broadcastIndex++) {
                try {
                    if (batchingStatus != BATCHING_STATUS_TRIP_COMPLETED) {
                        mCallbacksForStatus.getBroadcastItem(broadcastIndex).onBatchingStatusCb(
                                batchingStatus);
                    } else {
                        int sessionId =
                                (int) mCallbacksForStatus.getBroadcastCookie(broadcastIndex);
                        if ((completedTripClientList != null) &&
                                (completedTripClientList.contains(sessionId))) {
                            mCallbacksForStatus.getBroadcastItem(broadcastIndex).onBatchingStatusCb(
                                    batchingStatus);
                        }
                    }
                } catch(RemoteException e) {
                  // The RemoteCallbackList will take care of removing
                  // the dead object.
                }
            }
            mCallbacksForStatus.finishBroadcast();
        }
    }

    private void onMaxPowerAllocatedChanged(double power_mW) {
        if (VERBOSE_DBG) {
            Log.d(TAG, "entering onMaxPowerAllocatedChanged() power: " + power_mW);
        }
        // Broadcast to all clients the new value.
        int index = mMaxPowerCallbacks.beginBroadcast();
        for (int i = 0; i < index; i++) {
            if (VERBOSE_DBG) {
                Log.d(TAG, "in the mMaxPowerCallbacks loop : " + i);
            }
            try {
                IMaxPowerAllocatedCallback cb =
                    mMaxPowerCallbacks.getBroadcastItem(i);
                cb.onMaxPowerAllocatedChanged(power_mW);
            } catch (RemoteException e) {
                // The RemoteCallbackList will take care of removing
                // the dead object.
            }
        }
        mMaxPowerCallbacks.finishBroadcast();
    }

    private int getOffset (long sessionStartTime, Location[] locations) {
        int offset = -1, left = 0, right = locations.length-1;
        while(left <= right) {
            int mid = (left+right)/2;
            // found the exactly match
            if (sessionStartTime ==
                locations[mid].getTime()) {
                offset = mid;
                break;
            }
            if (sessionStartTime >
                locations[mid].getTime()) {
                left = mid + 1;
            }
            if (sessionStartTime <
                locations[mid].getTime()) {
                right = mid - 1;
            }
        }
        if (offset == -1) {
            offset = left;
        }

        if (VERBOSE_DBG) {
           Log.d(TAG, "offset : " + offset);
        }
        return offset;
    }

    public IFlpService getFlpBinder() {
        return mBinder;
    }

    public ITestService getFlpTestingBinder() {
        return mTestingBinder;
    }

    static {
        native_flp_class_init();
    }

    private static native void native_flp_class_init();
    private native int native_flp_init();
    private native int native_flp_get_all_supported_features();
    private native int native_flp_start_session(int id,
                                                int flags,
                                                long period_ns,
                                                int distance_interval_mps);
    private native int native_flp_update_session(int id,
                                                 int flags,
                                                 long period_ns,
                                                 int distance_interval_mps);
    private native int native_flp_stop_session(int id);
    private native int native_flp_get_all_locations();
    private native void native_flp_delete_aiding_data(long flags);
    private native double native_flp_get_max_power_allocated_in_mw();
}
