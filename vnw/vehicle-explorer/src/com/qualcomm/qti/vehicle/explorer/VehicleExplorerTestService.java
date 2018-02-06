/*
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
package com.qualcomm.qti.vehicle.explorer;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.qualcomm.qti.VehicleInterfaceData;
import com.qualcomm.qti.VehicleInterfaceDataHandler;
import com.qualcomm.qti.VehicleInterfaceSignals;
import com.qualcomm.qti.VehicleFrameworkNotificationType;
import com.qualcomm.qti.VehicleManager;
import com.qualcomm.qti.vehicle.explorer.tests.SocketCommunicator;

import com.qualcomm.qti.vehicle.explorer.tests.VehicleExplorerProtos;
import com.qualcomm.qti.vehicle.explorer.tests.VehicleExplorerProtos.ToDevice;
import com.qualcomm.qti.vehicle.explorer.tests.VehicleExplorerProtos.FromDevice;
import com.qualcomm.qti.vehicle.explorer.tests.VehicleExplorerProtos.Signal;
import com.qualcomm.qti.vehicle.explorer.tests.VehicleExplorerProtos.GetSignalResult;
import com.qualcomm.qti.vehicle.explorer.tests.VehicleExplorerProtos.SetSignalResult;
import com.qualcomm.qti.vehicle.explorer.tests.VehicleExplorerProtos.RegisterHandlerResult;
import com.qualcomm.qti.vehicle.explorer.tests.VehicleExplorerProtos.RemoveHandlerResult;


public class VehicleExplorerTestService extends Service {
    private static final int PORT = 1080;
    private static final int BUFFER_SIZE = 255;
    private static final String TAG = "VehicleTestService";
    private VehicleInterfaceData mInterfaceData;
    private VehicleManager mVehicleManager;
    private VehicleDataNotification mDataNotification;
    private VehicleManagerStatus mVmStatusCallback;
    private SocketCommunicator mSocketCommunicator;

    private void getSignalAPI(ToDevice toDevice, FromDevice fromDevice) {
        Log.i(TAG, "Received GetSignalCmd");
        boolean success = false;
        GetSignalResult getSignalResult = new GetSignalResult();

        int[] signals = toIntArray(toDevice.getGetSignalCmd().getSignalIdList());

        if ((signals != null) && (mInterfaceData != null)) {
            Log.i(TAG, "Calling mInterfaceData.getSignals");
            String[] values = mInterfaceData.getSignals(signals);
            if (values == null) {
                Log.e(TAG, "getSignals returned null");
            } else {
                success = true;
                for (String value : values) {
                    Log.i(TAG, "value of signal: " + value);
                    getSignalResult.addValue(value);
                }
            }
        }
        getSignalResult.setSuccess(success);
        fromDevice.setGetSignalResult(getSignalResult);
        Log.i(TAG, "GetSignalCmd cmd success " + success);
    }

    private void removeHandlerAPI(ToDevice toDevice, FromDevice fromDevice) {
        Log.i(TAG, "Received RemoveHandlerCmd");
        boolean success = false;
        RemoveHandlerResult removeHandlerResult = new RemoveHandlerResult();
        int[] signals = toIntArray(toDevice.getRemoveHandlerCmd().getSignalIdList());
        if (mInterfaceData != null) {
            success = mInterfaceData.removeHandler(signals, mDataNotification);
        }

        removeHandlerResult.setSuccess(success);
        fromDevice.setRemoveHandlerResult(removeHandlerResult);
        Log.i(TAG, "RemoveHandler cmd success " + success);
    }

    private void registerHandlerAPI(ToDevice toDevice, FromDevice fromDevice) {
        Log.i(TAG, "Received RegisterHandlerCmd");
        boolean success = false;
        int[] signals = toIntArray(toDevice.getRegisterHandlerCmd().getSignalIdList());
        int[] notificationRates = toIntArray(
                toDevice.getRegisterHandlerCmd().getNotificationRateList());
        VehicleFrameworkNotificationType[] notificationTypes  = toVfwNotificationArray(
                toDevice.getRegisterHandlerCmd().getNotificationTypeList());

        if (mInterfaceData != null) {
            Log.i(TAG, "Calling mInterfaceData.registerHandler");
            success = mInterfaceData.registerHandler(signals, mDataNotification,
                    notificationTypes, notificationRates);
        }
        RegisterHandlerResult registerHandlerResult = new RegisterHandlerResult();
        registerHandlerResult.setSuccess(success);
        fromDevice.setRegisterHandlerResult(registerHandlerResult);

        Log.i(TAG, "RegisterHandler cmd success: " + success);
    }

    private void setSignalAPI(ToDevice toDevice, FromDevice fromDevice) {
        Log.i(TAG, "Received SetSignalCmd");
        boolean success = false;
        SetSignalResult setSignalResult = new SetSignalResult();

        List<String> mValuesList = toDevice.getSetSignalCmd().getValueList();
        int[] signals = toIntArray(toDevice.getSetSignalCmd().getSignalIdList());

        if (mInterfaceData != null) {
            Log.i(TAG, "Calling mInterfaceData.setSignals");
            success = mInterfaceData.setSignals(signals,
                    mValuesList.toArray(new String[mValuesList.size()]));
        }

        setSignalResult.setSuccess(success);
        fromDevice.setSetSignalResult(setSignalResult);
        Log.i(TAG, "set signal cmd success " + success);
    }

    public VehicleFrameworkNotificationType[] toVfwNotificationArray(List<Integer> list) {
        VehicleFrameworkNotificationType[] newArray =
                new VehicleFrameworkNotificationType[list.size()];
        for (int i = 0; i < list.size(); i++) {
            switch (list.get(i)) {
                case VehicleExplorerProtos.VNW_PERIODIC_SIGNAL_VALUE_UPDATE:
                    newArray[i] = VehicleFrameworkNotificationType.
                            VNW_PERIODIC_SIGNAL_VALUE_UPDATE;
                    break;
                case VehicleExplorerProtos.VNW_SIGNAL_VALUE_CHANGED:
                    newArray[i] = VehicleFrameworkNotificationType.
                            VNW_SIGNAL_VALUE_CHANGED;
                    break;
                case VehicleExplorerProtos.VNW_SIGNAL_VALUE_REFRESHED:
                    newArray[i] = VehicleFrameworkNotificationType.
                            VNW_SIGNAL_VALUE_REFRESHED;
                    break;
            }
        }
        return newArray;
    }

    public int[] toIntArray(List<Integer> list) {
        int[] newArray = new int[list.size()];
        for (int i = 0; i < newArray.length; i++) {
            newArray[i] = list.get(i);
        }
        return newArray;
    }

    private class VehicleManagerStatus implements VehicleManager.VehicleManagerCallback {
        public void handleVehicleManagerCreationStatus(VehicleManager handle) {
            mVehicleManager = handle;
            setupVehicleManager();
        }
    }

    private class VehicleDataNotification implements VehicleInterfaceDataHandler {
        public void onNotify(VehicleFrameworkNotificationType type, int signalId) {
            Log.i(TAG, "RECEIVED type:"+ type + " id:" + signalId +
                    " value:" + mInterfaceData.getSignal(signalId));

            Signal signalProto = new Signal();
            signalProto.setSignalId(signalId);
            signalProto.setValue(mInterfaceData.getSignal(signalId));
            FromDevice fromDevice = new FromDevice();
            fromDevice.setSignal(signalProto);
            mSocketCommunicator.sendMessage(fromDevice.toByteArray());
        }

         public void onError(boolean refreshVehicleManager) {
            if (mVehicleManager != null && refreshVehicleManager) {
                mVehicleManager = null;
                mInterfaceData = null;
                VehicleManager.getInstance(getBaseContext(), mVmStatusCallback);
            } else {
                Log.e(TAG, "Ignored VehicleDataNotification error");
            }
        }
    }

    private void setupVehicleManager() {
        if (mVehicleManager == null) {
            Log.e(TAG, "mVehicleManager is NULL....");
            return;
        }
        mInterfaceData = mVehicleManager.getInterfaceHandle();

        if (mInterfaceData != null) {
            mDataNotification = new VehicleDataNotification();
        } else {
            Log.e(TAG, "getInterfaceHandle returned null...");
        }
    }

    private class CommandListener extends Thread {
        public void run() {
            Log.i(TAG, "CommandListener thread started");
            boolean run = true;
            byte[] buffer;

            FromDevice fromDevice;
            ToDevice toDevice;
            mSocketCommunicator = new SocketCommunicator(PORT, BUFFER_SIZE, true);

            while (run) {
                if (mSocketCommunicator.isConnected()) {
                    // blocking read to get next toDevice
                    buffer = mSocketCommunicator.read();
                    if (buffer == null) {
                        Log.e(TAG, "Breaking loop. mSocketCommunicator read returned null buffer");
                        mSocketCommunicator.disconnect();
                        break;
                    }

                    try {
                        toDevice = ToDevice.parseFrom(buffer);
                    } catch (Exception e) {
                        SocketCommunicator.printDebugMsgs(e);
                        break;
                    }

                } else {
                    Log.e(TAG, "Breaking loop. mSocketCommunicator disconnected.");
                    break;
                }

                fromDevice = new FromDevice();
                if (toDevice.hasFinalCmdFlag() && toDevice.getFinalCmdFlag().getFinalCmd()) {
                    Log.i(TAG, "Received Final Cmd Flag");
                    run = false;
                }
                if (toDevice.hasRemoveHandlerCmd()) {
                    removeHandlerAPI(toDevice, fromDevice);
                }
                if (toDevice.hasRegisterHandlerCmd()) {
                    registerHandlerAPI(toDevice, fromDevice);
                }
                if (toDevice.hasSetSignalCmd()) {
                    setSignalAPI(toDevice, fromDevice);
                }
                if (toDevice.hasGetSignalCmd()) {
                    getSignalAPI(toDevice, fromDevice);
                }
                if (run) {
                    mSocketCommunicator.sendMessage(fromDevice.toByteArray());
                }
            }
            mSocketCommunicator.disconnect();
        }
    }

    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "Service created...");
        mVmStatusCallback = new VehicleManagerStatus();
    }

    @Override
    public void onStart(Intent intent, int startId) {
        super.onStart(intent, startId);
        Log.i(TAG, "Service started...");

        boolean success = VehicleManager.getInstance(getBaseContext(), mVmStatusCallback);
        if (!success) {
            Log.e(TAG, "Unable to get instance of VehicleManager..");
        }
        CommandListener commandListener = new CommandListener();
        commandListener.start();
    }

    @Override
    public void onDestroy() {
        Log.i(TAG, "Service destroyed...");
        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }
}