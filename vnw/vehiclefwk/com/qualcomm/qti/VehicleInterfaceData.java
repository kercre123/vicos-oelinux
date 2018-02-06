/*
 *    Copyright (c) 2014-2015 Qualcomm Technologies, Inc. All Rights Reserved.
 *    Qualcomm Technologies Proprietary and Confidential.
 *
 */

package com.qualcomm.qti;

import android.content.Context;
import android.os.Handler;
import android.os.Message;
import android.os.Messenger;
import android.util.Log;

import com.qualcomm.qti.ivi.aidl.IVehicleService;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

/**
 * VehicleInterfaceData provides APIs for querying various vehicle data items,
 * such as, fuel, speed etc.
 */
public final class VehicleInterfaceData {
    private static final String TAG = "VehicleInterfaceData";
    private int[] mSupportedSignals;
    private int[] mWritableSignals;
    private IVehicleService mService;
    private Messenger mServiceRespMessenger;
    private Hashtable mClientCallbackHash;
    private Context mContext;

    /**
     * Constructor for VehicleInterfaceData;
     * @param service Handle to IVehicleService. Please note that this API is
     *            work in progress, and there might be few changes going
     *            forward.
     */
    VehicleInterfaceData(IVehicleService service, Context context) {
        mService = service;
        mContext = context;
        mClientCallbackHash = new Hashtable();
        mServiceRespMessenger = new Messenger(new ServiceRespHandler(context.getMainLooper()));
        try {
            mSupportedSignals = mService.getSupportedSignalIds();
            mWritableSignals = mService.getWritableSignalIds();
        } catch (android.os.RemoteException exception) {
            Log.e(TAG, "Could not communicate with VehicleService....");
            notifyFatalError();
        }
    }

    void onVehicleServiceDisconnected() {
        Log.v(TAG, "onVehicleServiceDisconnected....");
        mService = null;
        notifyFatalError();
    }

    /**
     * Query supported signal Ids on this platform.
     * @return array of supported signals else returns null on error. Please
     *         note that this API is work in progress, and there might be few
     *         changes going forward.
     */
    public int[] getSupportedSignalIds() {
        return mSupportedSignals;
    }

    /**
     * Query writable signals.
     * @return array of writable signals else returns null on error. Please
     *         note that this API is work in progress, and there might be few
     *         changes going forward.
     */
    public int[] getWritableSignalIds() {
        return mWritableSignals;
    }

    /**
     * Query signals identified by @param signalIds.
     * @param signalIds Identify signals to be queried.
     * @return Array of String pointing to value of signals. Although the
     *         return value is String, the actual Signal value could be any
     *         primitive data type. Caller needs to identify data type before
     *         interpreting/converting String value. Please see
     *         VehicleSignalConstants.java for data type definitions. Also,
     *         refer to getSignalsDataType API. Please note that this API is
     *         work in progress, and there might be few changes going forward.
     */
    public String[] getSignals(int[] signalIds) {
        String[] result = null;
        if ((signalIds != null) && (mService != null)) {
            try {
                result = mService.getSignals(signalIds);
            } catch (android.os.RemoteException exception) {
                Log.e(TAG, "Could not communicate with the service....");
                notifyFatalError();
            }
        }
        return result;
    }
    /**
     * Query signal identified by @param signalId.
     * @param signalId Identify signal to be queried.
     * @return String pointing to value of signal. Although the
     *         return value is String, the actual Signal value could be any
     *         primitive data type. Caller needs to identify data type before
     *         interpreting/converting String value. Please see
     *         VehicleSignalConstants.java for data type definitions. Also,
     *         refer to getSignalsDataType API. Please note that this API is
     *         work in progress, and there might be few changes going forward.
     */
    public String getSignal(int signalId) {
        String result = null;
        if (mService != null) {
            try {
                result = mService.getSignal(signalId);
            } catch (android.os.RemoteException exception) {
                Log.e(TAG, "Could not communicate with the service....");
                notifyFatalError();
            }
        }
        return result;
    }

    /**
     * Query user value identified by @param signalId.
     * @param signalId Identify user value to be queried.
     * @return String pointing to value of user. Although the
     *         return value is String, the actual user value value could be any
     *         primitive data type. Caller needs to identify data type before
     *         interpreting/converting String value. Please see
     *         VehicleSignalConstants.java for data type definitions. Also,
     *         refer to getSignalsDataType API. Please note that this API is
     *         work in progress, and there might be few changes going forward.
     */
    public String getUserValue(int signalId) {
        String result = null;
        if (mService != null) {
            try {
                result = mService.getUserValue(signalId);
            } catch (android.os.RemoteException exception) {
                Log.e(TAG, "Could not communicate with the service....");
                notifyFatalError();
            }
        }
        return result;
    }

    /**
     * Query signals data type.
     * @param signalIds Identify signals to be queried.
     * @return int[] where each element identifies the data type. Refer to
     *         VehicleSignalConstants.java for data type definitions. Signal
     *         value could be of any primitive data type. For each signal
     *         value retrieved via getSignals call, caller needs to
     *         interpret/convert according to it's primitive data type. Please
     *         note that this API is work in progress, and there might be few
     *         changes going forward.
     */
    public int[] getSignalsDataType(int[] signalIds) {
        int[] result = null;
        if ((signalIds != null) && (mService != null)) {
            try {
                result = mService.getSignalsDataType(signalIds);
            } catch (android.os.RemoteException exception) {
                Log.e(TAG, "Could not communicate with the service....");
                notifyFatalError();
            }
        }
        return result;
    }
    /**
     * Query signal data type.
     * @param signalId Identify signal to be queried.
     * @return int which identifies the data type. Refer to
     *         VehicleSignalConstants.java for data type definitions. Signal
     *         value could be of any primitive data type. For each signal
     *         value retrieved via getSignals call, caller needs to
     *         interpret/convert according to it's primitive data type. Please
     *         note that this API is work in progress, and there might be few
     *         changes going forward.
     */
    public int getSignalDataType(int signalId) {
        int result = VehicleSignalConstants.DATA_TYPE_UNKNOWN;
        if (mService != null) {
            try {
                result = mService.getSignalDataType(signalId);
            } catch (android.os.RemoteException exception) {
                Log.e(TAG, "Could not communicate with the service....");
                notifyFatalError();
            }
        }
        return result;
    }


    /**
     * Set value of signals.
     * @param signalIds Identify signals to be set.
     * @param values Point to values to be set for signals. A signal could
     *            be of any primitive data type. VehicleInterface will determine
     *            the data type before setting values.
     * @return true if no error occurs while setting the signal else returns
     *         false. Please note that this API is work in progress, and there
     *         might be few changes going forward.
     */
    public boolean setSignals(int[] signalIds, String[] values) {
        boolean bRet = false;
        if ((signalIds != null) && (values != null) && (mService != null)) {
            try {
                bRet = mService.setSignals(signalIds, values);
            } catch (android.os.RemoteException exception) {
                Log.e(TAG, "Could not communicate with the service....");
                notifyFatalError();
            }
        }
        return bRet;
    }
    /**
     * Set value of signal.
     * @param signalId Identify signal to be set.
     * @param value Point to value to be set for signal. A signal could
     *            be of any primitive data type. VehicleInterface will determine
     *            the data type before setting values.
     * @return true if no error occurs while setting the signal else returns
     *         false. Please note that this API is work in progress, and there
     *         might be few changes going forward.
     */
    public boolean setSignal(int signalId, String value) {
        boolean bRet = false;
        if (mService != null) {
            try {
                bRet = mService.setSignal(signalId, value);
            } catch (android.os.RemoteException exception) {
                Log.e(TAG, "Could not communicate with the service....");
                notifyFatalError();
            }
        }
        return bRet;
    }

    private boolean convertTypes(VehicleFrameworkNotificationType[] input, int[] output) {
        int index = 0;
        boolean result = true;
        for (VehicleFrameworkNotificationType type : input) {
            if (type == VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_NONE) {
                Log.e(TAG, "Error:notifyTypes contains NONE");
                result = false;
            }
            output[index++] = type.ordinal();
        }
        return result;
    }
    /**
     * Registers a handler to be called.
     * @param signalIds array of signal Ids caller is interested in.
     * @param handler Handle to the callback.
     * @param notifyTypes Identify the notification type user is interested
     *        in for each signal from signalIds.
     * @param rateMs Array which contain frequency in milli-seconds for call-backs to be fired.
     *        This parameter is valid only if notification type is
     *        VNW_PERIODIC_SIGNAL_VALUE_UPDATE.
     *        For all other notification types, the parameter will be ignored.
     * @return true if no error occurs while registering the callback else
     *         returns false. Please note that this API is work in progress, and
     *         there might be few changes going forward.
     */
    public boolean registerHandler(int[] signalIds, VehicleInterfaceDataHandler handler,
            VehicleFrameworkNotificationType[] notifyTypes, int[] rateMs) {
        boolean bRet = false;
        boolean bError = false;
            if ((handler != null) && (mService != null)) {
                try {
                    int[] types = null;
                    if ((signalIds != null) && (notifyTypes != null)) {
                        if (signalIds.length != notifyTypes.length) {
                            Log.e(TAG, "Error:signalIds and notifyTypes length do not match...");
                            bError = true;
                        } else {
                            types = new int[notifyTypes.length];
                            bError = !convertTypes(notifyTypes, types);
                        }
                    } else {
                        bError = true;
                        Log.v(TAG, "registerHandler failed.signalIds or notifyTypes or both null..");
                    }
                    if (bError == false) {
                        bRet = mService.registerMessenger(signalIds, mServiceRespMessenger,
                                types, rateMs);
                        if ((bRet == true) && (signalIds != null) && (mClientCallbackHash!= null)) {
                            for(int signalId : signalIds) {
                                Vector v = (Vector)
                                        mClientCallbackHash.get(Integer.toString(signalId));
                                if (v == null) {
                                    v = new Vector();
                                }
                                v.add(handler);
                                mClientCallbackHash.put(Integer.toString(signalId), v);
                            }
                        }
                    }
                } catch (android.os.RemoteException exception) {
                    Log.e(TAG, "Could not communicate with the service....");
                    notifyFatalError();
                }
            } else {
                Log.e(TAG, "registerHandler failed. Either handler or service is null..");
            }
        return bRet;
    }

    /**
     * Removes handler which was set successfully via registerHandler API.
     * @param handler Handle to the callback.
     * @return true if no error occurs while removing the handler else returns
     *         false. Please note that this API is work in progress, and there
     *         might be few changes going forward.
     */
    public boolean removeHandler(int[] signalIds, VehicleInterfaceDataHandler handler) {
        boolean bRet = false;
        if ((handler != null) && (mService != null)) {
            try {
                bRet = mService.unregisterMessenger(signalIds, mServiceRespMessenger);
                if (bRet == true) {
                    if (signalIds != null) {
                        for (int signalId : signalIds) {
                            Vector v = (Vector)mClientCallbackHash.get(Integer.toString(signalId));
                            if (v == null) {
                                Log.e(TAG,"Error removing handler for signalId " + signalId);
                            } else {
                                v.remove(handler);
                                mClientCallbackHash.put(Integer.toString(signalId), v);
                            }
                        }
                    }
                }
            } catch (android.os.RemoteException exception) {
                Log.e(TAG, "Could not communicate with the service....");
                notifyFatalError();
            }
        }
        return bRet;
    }
    /**
     * Handler for service response. Please note that this class is work in
     * progress, and there might be few changes going forward.
     */
    private class ServiceRespHandler extends Handler {
        ServiceRespHandler(android.os.Looper looper) {
            super(looper);
        }
        @Override
        public void handleMessage(Message msg) {
            if ((mClientCallbackHash != null) && (mClientCallbackHash.size() > 0)) {
                Vector hndVect = (Vector)mClientCallbackHash.get(Integer.toString(msg.arg1));
                if (hndVect != null) {
                    for (Object handler : hndVect) {
                        ((VehicleInterfaceDataHandler)handler).onNotify(
                                VehicleFrameworkNotificationType.values()[msg.what], msg.arg1);
                    }
                }
            }
        }
        private static final String TAG = "VehicleInterfaceData.ServiceRespHandler";
    }

    /**
     * Notify the client about the fatal error and provide the hint to clean up
     * and restart.
     */
    private void notifyFatalError() {
        Enumeration<String> hashKeys = mClientCallbackHash.keys();
        while(hashKeys.hasMoreElements()) {
            String key = hashKeys.nextElement();
            int signalId = Integer.parseInt(key);
            Vector hndVect = (Vector)mClientCallbackHash.get(Integer.toString(signalId));
            if (hndVect != null) {
                for (Object handler : hndVect) {
                    ((VehicleInterfaceDataHandler)handler).onError(true);
                }
            }
        }
    }
}
