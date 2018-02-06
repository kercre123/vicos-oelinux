/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

package com.qualcomm.location.izat.debugreport;

import android.content.Context;
import android.os.IBinder;
import android.os.Binder;
import android.os.Parcelable;
import android.os.RemoteCallbackList;
import android.os.RemoteException;
import android.os.Bundle;
import android.app.ActivityManager;
import android.app.ActivityManager.RunningAppProcessInfo;

import android.util.Log;
import java.lang.IndexOutOfBoundsException;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import com.qti.debugreport.*;

public class DebugReportService {
    private static final String TAG = "DebugReportService";
    private static final boolean DEBUG_DBG = Log.isLoggable(TAG, Log.DEBUG);

    private static final Object sCallBacksLock = new Object();
    private RemoteCallbackList<IDebugReportCallback> mDebugReportCallbacks
        = new RemoteCallbackList<IDebugReportCallback>();
    private final Context mContext;
    Timer mDebugReportTimer;

    private class ClientData {
        private IDebugReportCallback mCallback;
        private boolean mReportPeriodic;

        public ClientData(IDebugReportCallback callback) {
            mCallback = callback;
            mReportPeriodic = false;
        }
    }

    private Map<Integer, String> mPackagePerProcess;
    private Map<String, ClientData> mClientDataPerPackage;

    // DebugReport Data classes
    ArrayList<IZatEphmerisDebugReport> mListOfEphmerisReports =
            new ArrayList<IZatEphmerisDebugReport>();
    ArrayList<IZatFixStatusDebugReport> mListOfFixStatusReports =
            new ArrayList<IZatFixStatusDebugReport>();
    ArrayList<IZatLocationReport> mListOfBestLocationReports =
            new ArrayList<IZatLocationReport>();
    ArrayList<IZatLocationReport> mListOfEPIReports =
            new ArrayList<IZatLocationReport>();
    ArrayList<IZatGpsTimeDebugReport> mListGpsTimeOfReports =
            new ArrayList<IZatGpsTimeDebugReport>();
    ArrayList<IZatXoStateDebugReport> mListXoStateOfReports =
            new ArrayList<IZatXoStateDebugReport>();
    ArrayList<IZatRfStateDebugReport> mListRfStateOfReports =
            new ArrayList<IZatRfStateDebugReport>();
    ArrayList<IZatErrorRecoveryReport> mListOfErrorRecoveries =
            new ArrayList<IZatErrorRecoveryReport>();
    ArrayList<IZatPDRDebugReport> mListOfPDRReports =
            new ArrayList<IZatPDRDebugReport>();
    ArrayList<IZatSVHealthDebugReport> mListOfSVHealthReports =
            new ArrayList<IZatSVHealthDebugReport>();
    ArrayList<IZatXTRADebugReport> mListOfXTRAReports =
            new ArrayList<IZatXTRADebugReport>();

    public static DebugReportService sInstance = null;
    public static DebugReportService getInstance(Context ctx) {
        if (sInstance == null) {
            sInstance = new DebugReportService(ctx);
        }
        return sInstance;
    }

    public DebugReportService(Context ctx) {
        if (DEBUG_DBG) {
            Log.d(TAG, "DebugReportService construction");
        }

        mContext = ctx;
        mPackagePerProcess = new HashMap<Integer, String>();
        mClientDataPerPackage = new HashMap<String , ClientData>();

        native_debugreport_init();
    }

    /* Remote binder */
    private final IDebugReportService.Stub mBinder = new IDebugReportService.Stub() {

        public void registerForDebugReporting(final IDebugReportCallback callback) {
            if (callback == null) {
                Log.e(TAG, "callback is null");
                return;
            }

            synchronized (sCallBacksLock) {
                String callingPackage = getPackageName(Binder.getCallingPid());
                if (DEBUG_DBG) {
                    Log.d(TAG, "registerForDebugReporting: " + callingPackage);
                }

                if (mClientDataPerPackage.containsKey(callingPackage) == false) {
                    ClientData clData = new ClientData(callback);
                    mClientDataPerPackage.put(callingPackage, clData);
                } else {
                    ClientData clData = mClientDataPerPackage.get(callingPackage);
                    if (clData.mCallback != null) {
                        mDebugReportCallbacks.unregister(clData.mCallback);
                    }
                    clData.mCallback = callback;
                }

                mDebugReportCallbacks.register(callback);
            }


            try {
                callback.asBinder().linkToDeath(new IBinder.DeathRecipient() {
                    @Override
                    public void binderDied() {
                        synchronized (sCallBacksLock) {
                            mDebugReportCallbacks.unregister(callback);

                             ClientData clData = null;
                             String ownerPackage = null;
                            for (Map.Entry<String, ClientData> entry :
                                mClientDataPerPackage.entrySet()) {
                                clData = entry.getValue();
                                ownerPackage = entry.getKey();
                                if (clData.mCallback == callback) {
                                    if (DEBUG_DBG) {
                                        Log.d(TAG, "Package died: " + ownerPackage);
                                    }
                                    break;
                                }
                            }

                            mClientDataPerPackage.remove(ownerPackage);
                            checkOnPeriodicReporting();
                        }
                    }
                }, 0);
            } catch (RemoteException e) {
                throw new RuntimeException("Failed unregister debug report cb", e);
            }
        }

        public void unregisterForDebugReporting(IDebugReportCallback callback) {
            if (callback == null) {
                Log.e(TAG, "callback is null");
                return;
            }

            synchronized (sCallBacksLock) {
                String callingPackage = getPackageName(Binder.getCallingPid());
                 if (DEBUG_DBG) {
                    Log.d(TAG, "unregisterForDebugReporting: " + callingPackage);
                }

                mClientDataPerPackage.remove(callingPackage);
                mDebugReportCallbacks.unregister(callback);
                checkOnPeriodicReporting();
            }
        }

        public Bundle getDebugReport() {
            String callingPackage = getPackageName(Binder.getCallingPid());
            if (DEBUG_DBG) {
                Log.d(TAG, "getDebugReport: " + callingPackage);
            }

            synchronized(sCallBacksLock) {
                mListOfEphmerisReports.clear();
                mListOfFixStatusReports.clear();
                mListOfEPIReports.clear();
                mListOfBestLocationReports.clear();
                mListGpsTimeOfReports.clear();
                mListXoStateOfReports.clear();
                mListRfStateOfReports.clear();
                mListOfErrorRecoveries.clear();
                mListOfPDRReports.clear();
                mListOfSVHealthReports.clear();
                mListOfXTRAReports.clear();

                native_debugreport_getReport(30, mListOfEphmerisReports,
                                                 mListOfFixStatusReports,
                                                 mListOfEPIReports,
                                                 mListOfBestLocationReports,
                                                 mListGpsTimeOfReports,
                                                 mListXoStateOfReports,
                                                 mListRfStateOfReports,
                                                 mListOfErrorRecoveries,
                                                 mListOfPDRReports,
                                                 mListOfSVHealthReports,
                                                 mListOfXTRAReports);

                Bundle bundleDebugReportObj = new Bundle();
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_EPH_STATUS_KEY,
                        mListOfEphmerisReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_FIX_STATUS_KEY,
                        mListOfFixStatusReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_EXTERNAL_POSITION_INJECTION_KEY,
                        mListOfEPIReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_BEST_POSITION_KEY,
                        mListOfBestLocationReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_GPS_TIME_KEY,
                        mListGpsTimeOfReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_RF_STATE_KEY,
                        mListRfStateOfReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_XO_STATE_KEY,
                        mListXoStateOfReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_LAST_ERROR_RECOVERIES_KEY,
                        mListOfErrorRecoveries);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_PDR_INFO_KEY,
                        mListOfPDRReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_SV_HEALTH_KEY,
                        mListOfSVHealthReports);
                bundleDebugReportObj.putParcelableArrayList(
                        IZatDebugConstants.IZAT_DEBUG_XTRA_STATUS_KEY,
                        mListOfXTRAReports);
                return bundleDebugReportObj;
            }
        }

        public void startReporting() {
            String callingPackage = getPackageName(Binder.getCallingPid());

            if (DEBUG_DBG) {
                Log.d(TAG, "Request to start periodic reporting by package:"
                           + callingPackage);
            }

            // update ClientData for this package
            synchronized(sCallBacksLock) {
                ClientData clData = mClientDataPerPackage.get(callingPackage);
                if (clData != null) {
                    clData.mReportPeriodic = true;
                }
            }

            if (mDebugReportTimer != null) {
                if (DEBUG_DBG) {
                    Log.d(TAG, "Periodic reporting already in progress");
                }
                return;
            }

            mDebugReportTimer = new Timer();
            mDebugReportTimer.scheduleAtFixedRate(new TimerTask() {
                @Override
                public void run() {
                    synchronized (sCallBacksLock) {
                        mListOfEphmerisReports.clear();
                        mListOfFixStatusReports.clear();
                        mListOfEPIReports.clear();
                        mListOfBestLocationReports.clear();
                        mListGpsTimeOfReports.clear();
                        mListXoStateOfReports.clear();
                        mListRfStateOfReports.clear();
                        mListOfErrorRecoveries.clear();
                        mListOfPDRReports.clear();
                        mListOfSVHealthReports.clear();
                        mListOfXTRAReports.clear();

                        native_debugreport_getReport(1, mListOfEphmerisReports,
                                                        mListOfFixStatusReports,
                                                        mListOfEPIReports,
                                                        mListOfBestLocationReports,
                                                        mListGpsTimeOfReports,
                                                        mListXoStateOfReports,
                                                        mListRfStateOfReports,
                                                        mListOfErrorRecoveries,
                                                        mListOfPDRReports,
                                                        mListOfSVHealthReports,
                                                        mListOfXTRAReports);

                        if (mListOfEphmerisReports.isEmpty() &&
                            mListOfFixStatusReports.isEmpty() &&
                            mListOfEPIReports.isEmpty() &&
                            mListOfBestLocationReports.isEmpty() &&
                            mListGpsTimeOfReports.isEmpty() &&
                            mListXoStateOfReports.isEmpty() &&
                            mListRfStateOfReports.isEmpty() &&
                            mListOfErrorRecoveries.isEmpty() &&
                            mListOfPDRReports.isEmpty() &&
                            mListOfSVHealthReports.isEmpty() &&
                            mListOfXTRAReports.isEmpty()) {
                            if (DEBUG_DBG) {
                                Log.d(TAG, "Empty debug report");
                            }
                            return;
                        }

                        Bundle bundleDebugReportObj = new Bundle();

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_EPH_STATUS_KEY,
                                    mListOfEphmerisReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_FIX_STATUS_KEY,
                                    mListOfFixStatusReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_EXTERNAL_POSITION_INJECTION_KEY,
                                    mListOfEPIReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_BEST_POSITION_KEY,
                                    mListOfBestLocationReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_GPS_TIME_KEY,
                                    mListGpsTimeOfReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_RF_STATE_KEY,
                                    mListRfStateOfReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_XO_STATE_KEY,
                                    mListXoStateOfReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_LAST_ERROR_RECOVERIES_KEY,
                                    mListOfErrorRecoveries.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_PDR_INFO_KEY,
                                    mListOfPDRReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_SV_HEALTH_KEY,
                                    mListOfSVHealthReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        try {
                            bundleDebugReportObj.putParcelable(
                                    IZatDebugConstants.IZAT_DEBUG_XTRA_STATUS_KEY,
                                    mListOfXTRAReports.get(0));
                        } catch (IndexOutOfBoundsException ioobe) {}

                        ClientData clData = null;
                        String ownerPackage = null;
                        for (Map.Entry<String, ClientData> entry :
                            mClientDataPerPackage.entrySet()) {
                            clData = entry.getValue();
                        ownerPackage = entry.getKey();

                        if (clData.mReportPeriodic == true) {
                            if (DEBUG_DBG) {
                                Log.d(TAG, "Sending report to " + ownerPackage);
                            }

                            try {
                                clData.mCallback.onDebugDataAvailable(
                                        bundleDebugReportObj);
                            } catch (RemoteException e) {
                                         // do nothing
                            }
                        }
                    }
                }
                }}, 0, 1000);
            }

        public void stopReporting() {
            String callingPackage = getPackageName(Binder.getCallingPid());

            if (DEBUG_DBG) {
                Log.d(TAG, "Request to stop periodic reporting by package:"
                           + callingPackage);
            }

            // update ClientData for this package
            synchronized (sCallBacksLock) {
                ClientData clData = mClientDataPerPackage.get(callingPackage);
                if (clData != null) {
                    clData.mReportPeriodic = false;
                }

                checkOnPeriodicReporting();
            }
        }
    };

    private void checkOnPeriodicReporting() {
        boolean continuePeriodicReporting = false;
        ClientData clData = null;

        if (mDebugReportTimer == null) {
            if (DEBUG_DBG) {
                Log.d(TAG, "No peridoc reporting in progress !!");
            }
            return;
        }

        for (Map.Entry<String, ClientData> entry :
            mClientDataPerPackage.entrySet()) {
            clData = entry.getValue();
            if (clData.mReportPeriodic == true) {
                continuePeriodicReporting = true;
                break;
            }
        }

        if (continuePeriodicReporting == false) {
            if (DEBUG_DBG) {
                Log.d(TAG, "Service is stopping periodic debug reports");
            }

            mDebugReportTimer.cancel();
            mDebugReportTimer = null;
        }
    }

    private String getPackageName(int pid)
    {
        String packageName = mPackagePerProcess.get(pid);
        if (packageName == null) {
            ActivityManager am = (ActivityManager) mContext.getSystemService(
                Context.ACTIVITY_SERVICE);
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

    public IDebugReportService getDebugReportBinder() {
        return mBinder;
    }

    static {
        native_debugreport_class_init();
    }

    private static native void native_debugreport_class_init();
    private native void native_debugreport_init();
    private native void native_debugreport_deinit();
    private native void native_debugreport_getReport(
            int depth,
            List<IZatEphmerisDebugReport> ephmerisReports,
            List<IZatFixStatusDebugReport> fixStatusReports,
            List<IZatLocationReport> epiReports,
            List<IZatLocationReport> bestLocationReports,
            List<IZatGpsTimeDebugReport> gpsTimeReports,
            List<IZatXoStateDebugReport> xoStateReports,
            List<IZatRfStateDebugReport> rfStateReports,
            List<IZatErrorRecoveryReport> errorRecoveries,
            List<IZatPDRDebugReport> pdrReports,
            List<IZatSVHealthDebugReport> svHealthReports,
            List<IZatXTRADebugReport> xtraReports);
}
