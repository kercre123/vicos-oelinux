/*
 * Copyright (c) 2014-2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

package com.qualcomm.qti.vehicle.explorer;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.text.Html;
import android.util.Log;
import android.view.View;
import android.view.KeyEvent;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import com.qualcomm.qti.driver.distraction.DistractionNotifier;
import com.qualcomm.qti.VehicleInterfaceData;
import com.qualcomm.qti.VehicleInterfaceDataHandler;
import com.qualcomm.qti.VehicleInterfaceSignals;
import com.qualcomm.qti.VehicleFrameworkNotificationType;
import com.qualcomm.qti.VehicleManager;

public class VehicleExplorer extends Activity implements OnClickListener {
    private static final String TAG = "VehicleExplorer";
    public static VehicleInterfaceData mInterfaceData;
    private VehicleManager mVehicleManager;
    //Default interval(in milliseconds) for periodic notifications.
    private static final int CALL_BACK_INTERVAL = 1000;
    private Button mStatusBtn;
    private Button mOverviewBtn;
    private Button mMaintenanceBtn;
    private Button mParkingBtn;
    private Button mTransmissionBtn;
    private Button mSafetyBtn;
    private Button mLightsBtn;
    private Button mHvacBtn;
    private Button mUiBackHomeBtn;
    private int[] mSupportedSignals;
    private int[] mWritableSignals;
    private int[] mStatusUiInterestedSignals;
    private VehicleFrameworkNotificationType[] mStatusUiNotificationTypes;
    private int[] mStatusUiNotificationRate;

    private int[] mLightsUiInterestedSignals;
    private VehicleFrameworkNotificationType[] mLightsUiNotificationTypes;
    private int[] mLightsUiNotificationRate;

    private int[] mTransmissionUiInterestedSignals;
    private VehicleFrameworkNotificationType[] mTransmissionUiNotificationTypes;
    private int[] mTransmissionUiNotificationRate;

    private int[] mMaintenanceUiInterestedSignals;
    private VehicleFrameworkNotificationType[] mMaintUiNotificationTypes;
    private int[] mMaintUiNotificationRate;

    private VehicleDataNotification mDataNotification;
    private VehicleManagerStatus mVmStatusCallback;
    private int mCurrentUi;
    private DistractionNotifier mDistractionHandle;
    private DistractionNotifierCallback mDistractionCallback;

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle icicle) {
        super.onCreate(icicle);
        mCurrentUi = 0;
        setContentView(R.layout.vehicle_explorer);
        mVmStatusCallback = new VehicleManagerStatus();
        buildHomeUI();
        VehicleManager.getInstance(getBaseContext(), mVmStatusCallback);
        mDistractionHandle = DistractionNotifier.getInstance(getBaseContext());
        mDistractionCallback = new DistractionNotifierCallback();
        if (mDistractionHandle != null) {
            mDistractionHandle.registerCallback(mDistractionCallback);
        }
    }

    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.statusBtn:
                mCurrentUi = R.id.statusBtn;
                buildStatusUi();
                updateStatusUi();
                break;
            case R.id.maintenanceBtn:
                mCurrentUi = R.id.maintenanceBtn;
                buildMaintenanceUi();
                updateMaintenanceUi();
                break;
            case R.id.transmissionBtn:
                mCurrentUi = R.id.transmissionBtn;
                buildTransmissionUi();
                updateTransmissionUi();
                break;
            case R.id.safetyBtn:
                mCurrentUi = R.id.safetyBtn;
                buildSafetyUi1();
                break;
            case R.id.safetyonetotwo:
                mCurrentUi = R.id.safetyonetotwo;
                buildSafetyUi2();
                break;
            case R.id.safetytwotothree:
                mCurrentUi = R.id.safetytwotothree;
                buildSafetyUi3();
                break;
            case R.id.safetythreetotwo:
                mCurrentUi = R.id.safetythreetotwo;
                buildSafetyUi2();
                break;
            case R.id.safetytwotoone:
                mCurrentUi = R.id.safetytwotoone;
                buildSafetyUi1();
                break;
            case R.id.lightsBtn:
                mCurrentUi = R.id.lightsBtn;
                buildLightsUi();
                updateLightsUi();
                break;
            case R.id.hvacBtn:
                mCurrentUi = R.id.hvacBtn;
                buildHvacUi();
                break;
            case R.id.HomeUIbackBtn:
                mCurrentUi = R.id.HomeUIbackBtn;
                setContentView(R.layout.vehicle_explorer);
                buildHomeUI();
                break;
            case R.id.driverTempDownBtn:
                updateHvacUiElement(R.id.driverTempStatus, -1, 50,
                        VehicleInterfaceSignals.VIM_DRIVER_TEMP_SIGNAL);
                break;
            case R.id.driverTempUpBtn:
                updateHvacUiElement(R.id.driverTempStatus, 1, 80,
                        VehicleInterfaceSignals.VIM_DRIVER_TEMP_SIGNAL);
                break;
            case R.id.passengerTempDownBtn:
                updateHvacUiElement(R.id.passengerTempStatus, -1, 50,
                        VehicleInterfaceSignals.VIM_PASSENGER_TEMP_SIGNAL);
                break;
            case R.id.passengerTempUpBtn:
                updateHvacUiElement(R.id.passengerTempStatus, 1, 80,
                        VehicleInterfaceSignals.VIM_PASSENGER_TEMP_SIGNAL);
                break;
            case R.id.rearTempDownBtn:
                updateHvacUiElement(R.id.rearTempStatus, -1, 50,
                        VehicleInterfaceSignals.VIM_REAR_TEMP_SIGNAL);
                break;
            case R.id.rearTempUpBtn:
                updateHvacUiElement(R.id.rearTempStatus, 1, 80,
                        VehicleInterfaceSignals.VIM_REAR_TEMP_SIGNAL);
                break;
            case R.id.hvacFanSpeedDownBtn:
                updateHvacUiElement(R.id.hvacFanSpeedStatus, -1, 0,
                        VehicleInterfaceSignals.VIM_HVAC_FAN_SPEED_SIGNAL);
                break;
            case R.id.hvacFanSpeedUpBtn:
                updateHvacUiElement(R.id.hvacFanSpeedStatus, 1, 5,
                        VehicleInterfaceSignals.VIM_HVAC_FAN_SPEED_SIGNAL);
                break;
            case R.id.rdbtnHvacDirectionHead:
                if (mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HVAC_FAN_DIRECTION_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnHvacDirectionLegs:
                if (mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HVAC_FAN_DIRECTION_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnHvacDirectionHeadAndLegs:
                if (mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HVAC_FAN_DIRECTION_SIGNAL, "2");
                }
                break;
            case R.id.rdbtnHvacAcOn:
                if (mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_AIR_CONDITIONING_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnHvacAcOff:
                if (mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_AIR_CONDITIONING_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnStatusCruiseControlOn:
                boolean checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_CRUISE_CONTROL_STATUS_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnStatusCruiseControlOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_CRUISE_CONTROL_STATUS_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnMaintenanceMalfunctionOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_MALFUNCTION_INDICATOR_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnMaintenanceMalfunctionOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_MALFUNCTION_INDICATOR_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsHeadLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HEAD_LIGHTS_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsHeadLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HEAD_LIGHTS_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsAutoHeadLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_AUTOMATIC_HEAD_LIGHTS_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsAutoHeadLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_AUTOMATIC_HEAD_LIGHTS_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsHighBeamStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HEAD_LIGHTS_HIGH_BEAM_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsHighBeamStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HEAD_LIGHTS_HIGH_BEAM_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsAutoHighBeamStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_DYNAMIC_HIGH_BEAM_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsAutoHighBeamStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_DYNAMIC_HIGH_BEAM_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsLeftTurnStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_LEFT_TURN_LIGHT_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsLeftTurnStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_LEFT_TURN_LIGHT_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsRightTurnStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_RIGHT_TURN_LIGHT_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsRightTurnStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_RIGHT_TURN_LIGHT_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsBrakeLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_BRAKE_LIGHT_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsBrakeLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_BRAKE_LIGHT_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsFogFrontLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_LIGHT_STATUS_FOG_FRONT_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsFogFrontLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_LIGHT_STATUS_FOG_FRONT_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsFogRearLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_LIGHT_STATUS_FOG_REAR_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsFogRearLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_LIGHT_STATUS_FOG_REAR_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsHazardLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HAZARD_LIGHT_STATUS_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsHazardLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_HAZARD_LIGHT_STATUS_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsDriverLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_INTR_LIGHT_DRIVER_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsDriverLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_INTR_LIGHT_DRIVER_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsPsngrLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_INTR_LIGHT_PSNGR_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsPsngrLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_INTR_LIGHT_PSNGR_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLightsCenterLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_INTR_LIGHT_CENTER_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLightsCenterLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_INTR_LIGHT_CENTER_SIGNAL, "0");
                }
                break;
            case R.id.rdbtnLights3rdRowLightStatusOn:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_INTR_LIGHT_3RD_ROW_SIGNAL, "1");
                }
                break;
            case R.id.rdbtnLights3rdRowLightStatusOff:
                checked = ((RadioButton) v).isChecked();
                if(mInterfaceData != null) {
                    mInterfaceData.setSignal(
                            VehicleInterfaceSignals.VIM_INTR_LIGHT_3RD_ROW_SIGNAL, "0");
                }
                break;
            case R.id.testsButton:
                Intent intent = new Intent(this, TestingActivity.class);
                startActivity(intent);
                break;
        }
    }
    public static class DriverDistractionReceiver extends BroadcastReceiver {
        private static final String TAG = "DriverDistractionReceiver";

        @Override
        public void onReceive(Context context, Intent intent) {
            Toast toast = Toast.makeText(context, intent.getAction(), Toast.LENGTH_SHORT);
            toast.show();
        }
    }

    private class VehicleManagerStatus implements VehicleManager.VehicleManagerCallback {
        public void handleVehicleManagerCreationStatus(VehicleManager handle) {
            mVehicleManager = handle;
            testVehicleManager();
        }
    }
    private class DistractionNotifierCallback implements
            DistractionNotifier.DriverDistractionNotification {
        public void onNotify(String status) {
            Log.v(TAG, "DistractionNotifierCallback status: " + status);
        }
        public void onErrorNotify(boolean bCleanUpAndRestart) {
            //Log.v(TAG, " onErrorNotify bCleanUpAndRestart="+ bCleanUpAndRestart);
            //distraction service notified fatal error, so restart..
            if( (bCleanUpAndRestart == true) && (mDistractionHandle != null) ) {
                Toast toast = Toast.makeText(VehicleExplorer.this.getBaseContext(),
                        "Distraction Service restarting...", Toast.LENGTH_SHORT);
                toast.show();
                mDistractionHandle = DistractionNotifier.getInstance(getBaseContext());
                mDistractionCallback = new DistractionNotifierCallback();
                if (mDistractionHandle != null) {
                    mDistractionHandle.registerCallback(mDistractionCallback);
                }
            }
        }
    }

    private void cleanUpAndRestart() {
        mVehicleManager = null;
        mInterfaceData = null;
        VehicleManager.getInstance(getBaseContext(), mVmStatusCallback);
    }

    private class VehicleDataNotification implements VehicleInterfaceDataHandler {
        public void onNotify(VehicleFrameworkNotificationType type, int signalId) {
            //Log.v(TAG, "VehicleDataNotification onNotify  type "+ type+" SignalId "+signalId);
            readInterfaceData();
        }
        public void onError(boolean bCleanUpAndRestart) {
            //Log.v(TAG, "bCleanUpAndRestart="+ bCleanUpAndRestart);
            //We might run into several RemoteException based on number of signals
            //registered for callback with the service. We should discard duplicate ones.
            //There is still a small window where this could still happen.
            //If one or more duplicate exceptions are delayed in service itself and appear
            //after we have retsrated the application, following check will not be useful.
            if( (bCleanUpAndRestart == true) && (mVehicleManager != null) ) {
                // 1 sec delay to avoid CPU overusage in the case of frequent errors
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    Log.e(TAG, "Caught exception while sleeping in onError:" + e);
                }
                cleanUpAndRestart();
                Toast toast = Toast.makeText(VehicleExplorer.this.getBaseContext(),
                        "Error reported.Restarting the application", Toast.LENGTH_SHORT);
                toast.show();
                mCurrentUi = 0;
                setContentView(R.layout.vehicle_explorer);
                buildHomeUI();
            }
        }
    }

    private void buildHomeUI() {
        TextView textArea  = (TextView)findViewById(R.id.vehicleHistoryTextArea);
        textArea.setText(Html.fromHtml
                ("<h1><center><i><u>QTI Automotive</center></i></u></h1><br><br><p>" +
                 "Vehicle data is grouped into multiple categories.<br>"+
                 "Please select category from the left menu for details...</p>"));
        mStatusBtn = (Button)findViewById(R.id.statusBtn);
        mStatusBtn.setOnClickListener(this);
        mMaintenanceBtn = (Button)findViewById(R.id.maintenanceBtn);
        mMaintenanceBtn.setOnClickListener(this);
        mTransmissionBtn = (Button)findViewById(R.id.transmissionBtn);
        mTransmissionBtn.setOnClickListener(this);
        mSafetyBtn = (Button)findViewById(R.id.safetyBtn);
        //do not enable safety button unless we add the support for querying safety data.
        //mSafetyBtn.setOnClickListener(this);
        mLightsBtn = (Button)findViewById(R.id.lightsBtn);
        mLightsBtn.setOnClickListener(this);
        mHvacBtn = (Button)findViewById(R.id.hvacBtn);
        mHvacBtn.setOnClickListener(this);
        Button testsButton = (Button)findViewById(R.id.testsButton);
        testsButton.setOnClickListener(this);
    }

    private void buildStatusUi() {
        setContentView(R.layout.statusui);
        RadioButton cruiseon = (RadioButton)findViewById(R.id.rdbtnStatusCruiseControlOn);
        RadioButton cruiseoff = (RadioButton)findViewById(R.id.rdbtnStatusCruiseControlOff);
        cruiseon.setOnClickListener(this);
        cruiseoff.setOnClickListener(this);
        mUiBackHomeBtn = (Button)findViewById(R.id.HomeUIbackBtn);
        mUiBackHomeBtn.setOnClickListener(this);
    }

    private void buildMaintenanceUi() {
        setContentView(R.layout.maintenanceui);
        RadioButton malfunctionLightOn = (RadioButton)findViewById(R.id.rdbtnMaintenanceMalfunctionOn);
        RadioButton malfunctionLightOff = (RadioButton)findViewById(R.id.rdbtnMaintenanceMalfunctionOff);
        malfunctionLightOn.setOnClickListener(this);
        malfunctionLightOff.setOnClickListener(this);
        mUiBackHomeBtn = (Button)findViewById(R.id.HomeUIbackBtn);
        mUiBackHomeBtn.setOnClickListener(this);
    }

    private void buildTransmissionUi() {
        setContentView(R.layout.transmissionui);
        mUiBackHomeBtn = (Button)findViewById(R.id.HomeUIbackBtn);
        mUiBackHomeBtn.setOnClickListener(this);
    }

    private void buildLightsUi() {
        setContentView(R.layout.lightsui);
        RadioButton hLightOn = (RadioButton)findViewById(R.id.rdbtnLightsHeadLightStatusOn);
        RadioButton hLightOff = (RadioButton)findViewById(R.id.rdbtnLightsHeadLightStatusOff);
        RadioButton autoHeadLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsAutoHeadLightStatusOn);
        RadioButton autoHeadLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsAutoHeadLightStatusOff);

        RadioButton hBeamOn = (RadioButton)findViewById(R.id.rdbtnLightsHighBeamStatusOn);
        RadioButton hBeamOff = (RadioButton)findViewById(R.id.rdbtnLightsHighBeamStatusOff);
        RadioButton autoHighBeamOn = (RadioButton)findViewById
                (R.id.rdbtnLightsAutoHighBeamStatusOn);
        RadioButton autoHighBeamOff = (RadioButton)findViewById
                (R.id.rdbtnLightsAutoHighBeamStatusOff);

        RadioButton leftTurnOn = (RadioButton)findViewById(R.id.rdbtnLightsLeftTurnStatusOn);
        RadioButton leftTurnOff = (RadioButton)findViewById(R.id.rdbtnLightsLeftTurnStatusOff);
        RadioButton rightTurnOn = (RadioButton)findViewById(R.id.rdbtnLightsRightTurnStatusOn);
        RadioButton rightTurnOff = (RadioButton)findViewById(R.id.rdbtnLightsRightTurnStatusOff);

        RadioButton brkLightOn = (RadioButton)findViewById(R.id.rdbtnLightsBrakeLightStatusOn);
        RadioButton brkLightOff = (RadioButton)findViewById(R.id.rdbtnLightsBrakeLightStatusOff);
        RadioButton fongFrontLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsFogFrontLightStatusOn);
        RadioButton fongFrontLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsFogFrontLightStatusOff);
        RadioButton fogRearLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsFogRearLightStatusOn);
        RadioButton fogRearLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsFogRearLightStatusOff);

        RadioButton hazardLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsHazardLightStatusOn);
        RadioButton hazardLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsHazardLightStatusOff);
        RadioButton driverLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsDriverLightStatusOn);
        RadioButton driverLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsDriverLightStatusOff);
        RadioButton psngrLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsPsngrLightStatusOn);
        RadioButton psngrLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsPsngrLightStatusOff);
        RadioButton centerLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsCenterLightStatusOn);
        RadioButton centerLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsCenterLightStatusOff);
        RadioButton thirdRowLightOn = (RadioButton)findViewById
                (R.id.rdbtnLights3rdRowLightStatusOn);
        RadioButton thirdRowLightOff = (RadioButton)findViewById
                (R.id.rdbtnLights3rdRowLightStatusOff);
        hLightOn.setOnClickListener(this);
        hLightOff.setOnClickListener(this);
        autoHeadLightOn.setOnClickListener(this);
        autoHeadLightOff.setOnClickListener(this);
        hBeamOn.setOnClickListener(this);
        hBeamOff.setOnClickListener(this);
        autoHighBeamOn.setOnClickListener(this);
        autoHighBeamOff.setOnClickListener(this);
        leftTurnOn.setOnClickListener(this);
        leftTurnOff.setOnClickListener(this);
        rightTurnOn.setOnClickListener(this);
        rightTurnOff.setOnClickListener(this);
        brkLightOn.setOnClickListener(this);
        brkLightOff.setOnClickListener(this);
        fongFrontLightOn.setOnClickListener(this);
        fongFrontLightOff.setOnClickListener(this);
        fogRearLightOn.setOnClickListener(this);
        fogRearLightOff.setOnClickListener(this);
        hazardLightOn.setOnClickListener(this);
        hazardLightOff.setOnClickListener(this);
        driverLightOn.setOnClickListener(this);
        driverLightOff.setOnClickListener(this);
        psngrLightOn.setOnClickListener(this);
        psngrLightOff.setOnClickListener(this);
        centerLightOn.setOnClickListener(this);
        centerLightOff.setOnClickListener(this);
        thirdRowLightOn.setOnClickListener(this);
        thirdRowLightOff.setOnClickListener(this);
        mUiBackHomeBtn = (Button)findViewById(R.id.HomeUIbackBtn);
        mUiBackHomeBtn.setOnClickListener(this);
    }

    private void buildHvacUi() {
        setContentView(R.layout.hvacui);
        Button driverTempDown = (Button)findViewById(R.id.driverTempDownBtn);
        Button driverTempUp = (Button)findViewById(R.id.driverTempUpBtn);
        Button passengerTempDown = (Button)findViewById(R.id.passengerTempDownBtn);
        Button passengerTempUp = (Button)findViewById(R.id.passengerTempUpBtn);
        Button rearTempDown = (Button)findViewById(R.id.rearTempDownBtn);
        Button rearTempUp = (Button)findViewById(R.id.rearTempUpBtn);
        Button hvacFanSpeedDownBtn = (Button)findViewById(R.id.hvacFanSpeedDownBtn);
        Button hvacFanSpeedUpBtn = (Button)findViewById(R.id.hvacFanSpeedUpBtn);
        RadioButton rdbtnHvacDirectionHead = (RadioButton)findViewById(
                R.id.rdbtnHvacDirectionHead);
        RadioButton rdbtnHvacDirectionLegs = (RadioButton)findViewById(
                R.id.rdbtnHvacDirectionLegs);
        RadioButton rdbtnHvacDirectionHeadAndLegs = (RadioButton)findViewById(
                R.id.rdbtnHvacDirectionHeadAndLegs);
        RadioButton rdbtnHvacAcOn = (RadioButton)findViewById(R.id.rdbtnHvacAcOn);
        RadioButton rdbtnHvacAcOff = (RadioButton)findViewById(R.id.rdbtnHvacAcOff);
        driverTempDown.setOnClickListener(this);
        driverTempUp.setOnClickListener(this);
        passengerTempDown.setOnClickListener(this);
        passengerTempUp.setOnClickListener(this);
        rearTempDown.setOnClickListener(this);
        rearTempUp.setOnClickListener(this);
        hvacFanSpeedDownBtn.setOnClickListener(this);
        hvacFanSpeedUpBtn.setOnClickListener(this);
        rdbtnHvacDirectionHead.setOnClickListener(this);
        rdbtnHvacDirectionLegs.setOnClickListener(this);
        rdbtnHvacDirectionHeadAndLegs.setOnClickListener(this);
        rdbtnHvacAcOn.setOnClickListener(this);
        rdbtnHvacAcOff.setOnClickListener(this);
        mUiBackHomeBtn = (Button)findViewById(R.id.HomeUIbackBtn);
        mUiBackHomeBtn.setOnClickListener(this);
    }

    private void buildSafetyUi1() {
        setContentView(R.layout.safetyui1);
        mUiBackHomeBtn = (Button)findViewById(R.id.HomeUIbackBtn);
        mUiBackHomeBtn.setOnClickListener(this);
        Button nextbtn = (Button)findViewById(R.id.safetyonetotwo);
        nextbtn.setOnClickListener(this);
    }

    private void buildSafetyUi2() {
        setContentView(R.layout.safetyui2);
        mUiBackHomeBtn = (Button)findViewById(R.id.HomeUIbackBtn);
        mUiBackHomeBtn.setOnClickListener(this);
        Button nextbtn = (Button)findViewById(R.id.safetytwotothree);
        nextbtn.setOnClickListener(this);
        Button prevBtn = (Button)findViewById(R.id.safetytwotoone);
        prevBtn.setOnClickListener(this);
    }

    private void buildSafetyUi3() {
        setContentView(R.layout.safetyui3);
        mUiBackHomeBtn = (Button)findViewById(R.id.HomeUIbackBtn);
        mUiBackHomeBtn.setOnClickListener(this);
        Button prevBtn = (Button)findViewById(R.id.safetythreetotwo);
        prevBtn.setOnClickListener(this);
    }

    private void updateStatusUi() {
        String[] values = null;
        if ((mStatusUiInterestedSignals != null) && (mInterfaceData != null)) {
            values = mInterfaceData.getSignals(mStatusUiInterestedSignals);
            if (values == null) {
                Log.e(TAG, "null getSignals for mStatusUiInterestedSignals...");
            }
        }
        if (values == null) {
            return;
        }
        EditText gear = (EditText)findViewById(R.id.GearStatus);
        EditText type = (EditText)findViewById(R.id.VehicleType);
        EditText rmngFuel = (EditText)findViewById(R.id.RemainingFuel);
        EditText speed = (EditText)findViewById(R.id.CurrentSpeed);
        EditText rpm = (EditText)findViewById(R.id.RPM);
        EditText tp1milege = (EditText)findViewById(R.id.TripMeter1);
        EditText tp2milege = (EditText)findViewById(R.id.TripMeter2);
        EditText tp1avgspeed = (EditText)findViewById(R.id.TripMeter1AvgSpeed);
        EditText tp2avgspeed = (EditText)findViewById(R.id.TripMeter2AvgSpeed);
        EditText tp1fuelconsumption = (EditText)findViewById(R.id.TripMeter1FuelConsumption);
        EditText tp2fuelconsumption = (EditText)findViewById(R.id.TripMeter2FuelConsumption);
        RadioButton cruiseon = (RadioButton)findViewById(R.id.rdbtnStatusCruiseControlOn);
        RadioButton cruiseoff = (RadioButton)findViewById(R.id.rdbtnStatusCruiseControlOff);
        int index = 0;
        for (int signal : mStatusUiInterestedSignals) {
            String data = values[index];
            boolean bDataWasNull = false;
            if (data == null) {
                data = "Not available";
                bDataWasNull = true;
            }
            if (signal == VehicleInterfaceSignals.VIM_VEHICLE_VIN_SIGNAL) {
            } else if (signal == VehicleInterfaceSignals.VIM_TRANSMISSION_GEAR_STATUS_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    gear.setEnabled(false);
                    gear.setEnabled(false);
                }
                gear.setText(data.toCharArray(), 0, data.length());
            } else if (signal == VehicleInterfaceSignals.VIM_VEHICLE_TYPE_SIGNAL) {
                type.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    type.setEnabled(false);
                }
            }
            else if (signal == VehicleInterfaceSignals.VIM_RMNG_FUEL_LVL_SIGNAL) {
                rmngFuel.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    rmngFuel.setEnabled(false);
                }
            }
            else if (signal == VehicleInterfaceSignals.VIM_SPEEDO_METER_SIGNAL) {
                speed.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    speed.setEnabled(false);
                }
            }
            else if (signal == VehicleInterfaceSignals.VIM_ENGINE_SPEED_SIGNAL) {
                rpm.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    rpm.setEnabled(false);
                }
            }
            else if (signal == VehicleInterfaceSignals.VIM_TRIP_METER_1_MILEAGE_SIGNAL) {
                tp1milege.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tp1milege.setEnabled(false);
                }
            }
            else if (signal == VehicleInterfaceSignals.VIM_TRIP_METER_2_MILEAGE_SIGNAL) {
                tp2milege.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tp2milege.setEnabled(false);
                }
            }
            else if (signal ==
                    VehicleInterfaceSignals.VIM_TRIP_METER_1_FUEL_CONSUMPTION_SIGNAL) {
                tp1fuelconsumption.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tp1fuelconsumption.setEnabled(false);
                }
            }
            else if (signal ==
                    VehicleInterfaceSignals.VIM_TRIP_METER_2_FUEL_CONSUMPTION_SIGNAL) {
                tp2fuelconsumption.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tp2fuelconsumption.setEnabled(false);
                }
            }
            else if (signal == VehicleInterfaceSignals.VIM_TRIP_METER_1_AVG_SPEED_SIGNAL) {
                tp1avgspeed.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tp1avgspeed.setEnabled(false);
                }
            }
            else if (signal == VehicleInterfaceSignals.VIM_TRIP_METER_2_AVG_SPEED_SIGNAL) {
                tp2avgspeed.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tp2avgspeed.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_CRUISE_CONTROL_STATUS_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    cruiseon.setEnabled(false);
                    cruiseoff.setEnabled(false);
                }
                if (bDataWasNull == false) {
                    if (Integer.parseInt(data) > 0) {
                        cruiseon.setChecked(true);
                        cruiseoff.setChecked(false);
                    } else {
                        cruiseon.setChecked(false);
                        cruiseoff.setChecked(true);
                    }
                }
            }
            index++;
        }
    }

    private void updateHvacUiElement(int elementId, int delta, int threshold, int signal) {
        EditText editTextField = (EditText)findViewById(elementId);
        String editTextString = String.valueOf(
                Integer.parseInt(editTextField.getText().toString()) + delta);
        if (delta > 0 && Integer.parseInt(editTextString) > threshold) {
                editTextString = String.valueOf(threshold);
        } else if (delta < 0 && Integer.parseInt(editTextString) < threshold) {
            editTextString = String.valueOf(threshold);
        }
        editTextField.setText(editTextString.toCharArray(), 0, editTextString.length());
        if (mInterfaceData != null) {
            mInterfaceData.setSignal(signal, editTextString);
        }
    }


    private void updateTransmissionUi() {
        String[] values = null;
        if ((mTransmissionUiInterestedSignals != null) && (mInterfaceData != null)) {
            values = mInterfaceData.getSignals(mTransmissionUiInterestedSignals);
            if (values == null) {
                Log.e(TAG, "null getSignals for mTransmissionUiInterestedSignals...");
            }
        }
        if (values == null) {
            return;
        }
        EditText powerMode = (EditText)findViewById(R.id.VehiclePowerMode);
        EditText rmngFuelInGalons = (EditText)findViewById(R.id.RemainingFuelInGallons);
        EditText rmngDrvRng = (EditText)findViewById(R.id.RemainingDrivingRange);
        EditText rmngEngineOil = (EditText)findViewById(R.id.RemainingEngineOil);
        EditText rmngCoolantLvl = (EditText)findViewById(R.id.CoolantLevel);
        EditText coolantTemp = (EditText)findViewById(R.id.CoolantTemp);
        EditText strngWheelAngle = (EditText)findViewById(R.id.StrngWheelAngle);
        int index = 0;
        for (int signal : mTransmissionUiInterestedSignals) {
            String data = values[index];
            if (data == null) {
                data = "Not available";
            }
            if (signal == VehicleInterfaceSignals.VIM_VEHICLE_POWER_MODE_SIGNAL) {
                powerMode.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    powerMode.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_MEASUREMENT_FUEL_SIGNAL) {
                rmngFuelInGalons.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    rmngFuelInGalons.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_RMNG_DRVNG_RANGE_SIGNAL) {
                rmngDrvRng.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    rmngDrvRng.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_ENGN_OIL_RMNG_SIGNAL) {
                rmngEngineOil.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    rmngEngineOil.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_COOLANT_LVL_SIGNAL) {
                rmngCoolantLvl.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    rmngCoolantLvl.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_COOLANT_TEMP_SIGNAL) {
                coolantTemp.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    coolantTemp.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_STRNG_WHEEL_ANGLE_SIGNAL) {
                strngWheelAngle.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    strngWheelAngle.setEnabled(false);
                }
            }
            index++;
        }
    }
    private void updateMaintenanceUi() {
        String[] values = null;
        if ((mMaintenanceUiInterestedSignals != null) && (mInterfaceData != null)) {
            values = mInterfaceData.getSignals(mMaintenanceUiInterestedSignals);
            if (values == null) {
                Log.e(TAG, "null getSignals for mMaintenanceUiInterestedSignals...");
            }
        }
        if (values == null) {
            return;
        }
        EditText odoMeter = (EditText)findViewById(R.id.mntOdometer);
        EditText oilLevel = (EditText)findViewById(R.id.mntOilLevel);
        EditText oilTemp = (EditText)findViewById(R.id.mntOilTemp);
        EditText brakeFluidLvl = (EditText)findViewById(R.id.mntBrakeFluidLevel);
        EditText washerFluidLvl = (EditText)findViewById(R.id.mntWasherFluidLevel);
        RadioButton malfunctionLightOn = (RadioButton)findViewById(R.id.rdbtnMaintenanceMalfunctionOn);
        RadioButton malfunctionLightOff = (RadioButton)findViewById(R.id.rdbtnMaintenanceMalfunctionOff);
        EditText tireFrontLeft = (EditText)findViewById(R.id.mntTirePressureFL);
        EditText tireFrontRight = (EditText)findViewById(R.id.mntTirePressureFR);
        EditText tireRearLeft = (EditText)findViewById(R.id.mntTirePressureRL);
        EditText tireRearRight = (EditText)findViewById(R.id.mntTirePressureRR);
        EditText batteryVoltage = (EditText)findViewById(R.id.mntBatteryVoltage);
        EditText batteryCurrent = (EditText)findViewById(R.id.mntBatteryCurrent);
        int index = 0;
        for (int signal : mMaintenanceUiInterestedSignals) {
            boolean bDataWasNull = false;
            String data = values[index];
            if (data == null) {
                data = "Not available";
                bDataWasNull = true;
            }
            if (signal == VehicleInterfaceSignals.VIM_ODOMETER_SIGNAL) {
                odoMeter.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    odoMeter.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_TRANSMISSION_OIL_LIFE_LVL_SIGNAL) {
                oilLevel.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    oilLevel.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_TRANSMISSION_OIL_TEMP_SIGNAL) {
                oilTemp.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    oilTemp.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_BRAKE_FLUID_LVL_SIGNAL) {
                brakeFluidLvl.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    brakeFluidLvl.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_WASHER_FLUID_LVL_SIGNAL) {
                washerFluidLvl.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    washerFluidLvl.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_TIRE_PRESSURE_FRONT_LEFT_SIGNAL) {
                tireFrontLeft.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tireFrontLeft.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_TIRE_PRESSURE_FRONT_RIGHT_SIGNAL) {
                tireFrontRight.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tireFrontRight.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_TIRE_PRESSURE_REAR_LEFT_SIGNAL) {
                tireRearLeft.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tireRearLeft.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_TIRE_PRESSURE_REAR_RIGHT_SIGNAL) {
                tireRearRight.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    tireRearRight.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_BATTERY_VOLTAGE_SIGNAL) {
                batteryVoltage.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    batteryVoltage.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_BATTERY_CURRENT_SIGNAL) {
                batteryCurrent.setText(data.toCharArray(), 0, data.length());
                if (isWritableSignal(signal) != true) {
                    batteryCurrent.setEnabled(false);
                }
            } else if (signal == VehicleInterfaceSignals.VIM_MALFUNCTION_INDICATOR_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    malfunctionLightOn.setEnabled(false);
                    malfunctionLightOff.setEnabled(false);
                }
                if (bDataWasNull == false) {
                    if (Integer.parseInt(data) > 0) {
                        malfunctionLightOn.setChecked(true);
                        malfunctionLightOff.setChecked(false);
                    } else {
                        malfunctionLightOn.setChecked(false);
                        malfunctionLightOff.setChecked(true);
                    }
                }
            }
            index++;
        }
    }

    private void updateLightsUi() {
        String[] values = null;
        if ((mLightsUiInterestedSignals != null) && (mInterfaceData != null)) {
            values = mInterfaceData.getSignals(mLightsUiInterestedSignals);
            if (values == null) {
                Log.e(TAG, "null getSignals for mLightsUiInterestedSignals...");
            }
        }
        if (values == null) {
            return;
        }
        // Log.v(TAG, "mLightsUiInterestedSignals.length " +
        // mLightsUiInterestedSignals.length);
        // Log.v(TAG, "values.length " + values.length);
        RadioButton hLightOn = (RadioButton)findViewById(R.id.rdbtnLightsHeadLightStatusOn);
        RadioButton hLightOff = (RadioButton)findViewById(R.id.rdbtnLightsHeadLightStatusOff);
        RadioButton autoHeadLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsAutoHeadLightStatusOn);
        RadioButton autoHeadLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsAutoHeadLightStatusOff);

        RadioButton hBeamOn = (RadioButton)findViewById(R.id.rdbtnLightsHighBeamStatusOn);
        RadioButton hBeamOff = (RadioButton)findViewById(R.id.rdbtnLightsHighBeamStatusOff);
        RadioButton autoHighBeamOn = (RadioButton)findViewById
                (R.id.rdbtnLightsAutoHighBeamStatusOn);
        RadioButton autoHighBeamOff = (RadioButton)findViewById
                (R.id.rdbtnLightsAutoHighBeamStatusOff);

        RadioButton leftTurnOn = (RadioButton)findViewById(R.id.rdbtnLightsLeftTurnStatusOn);
        RadioButton leftTurnOff = (RadioButton)findViewById(R.id.rdbtnLightsLeftTurnStatusOff);
        RadioButton rightTurnOn = (RadioButton)findViewById(R.id.rdbtnLightsRightTurnStatusOn);
        RadioButton rightTurnOff = (RadioButton)findViewById(R.id.rdbtnLightsRightTurnStatusOff);

        RadioButton brkLightOn = (RadioButton)findViewById(R.id.rdbtnLightsBrakeLightStatusOn);
        RadioButton brkLightOff = (RadioButton)findViewById(R.id.rdbtnLightsBrakeLightStatusOff);
        RadioButton fongFrontLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsFogFrontLightStatusOn);
        RadioButton fongFrontLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsFogFrontLightStatusOff);
        RadioButton fogRearLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsFogRearLightStatusOn);
        RadioButton fogRearLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsFogRearLightStatusOff);

        RadioButton hazardLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsHazardLightStatusOn);
        RadioButton hazardLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsHazardLightStatusOff);
        RadioButton driverLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsDriverLightStatusOn);
        RadioButton driverLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsDriverLightStatusOff);
        RadioButton psngrLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsPsngrLightStatusOn);
        RadioButton psngrLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsPsngrLightStatusOff);
        RadioButton centerLightOn = (RadioButton)findViewById
                (R.id.rdbtnLightsCenterLightStatusOn);
        RadioButton centerLightOff = (RadioButton)findViewById
                (R.id.rdbtnLightsCenterLightStatusOff);
        RadioButton thirdRowLightOn = (RadioButton)findViewById
                (R.id.rdbtnLights3rdRowLightStatusOn);
        RadioButton thirdRowLightOff = (RadioButton)findViewById
                (R.id.rdbtnLights3rdRowLightStatusOff);
        int index = 0;
        for (int signal : mLightsUiInterestedSignals) {
            String data = values[index];
            if (signal == VehicleInterfaceSignals.VIM_HEAD_LIGHTS_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    hLightOn.setEnabled(false);
                    hLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        hLightOn.setChecked(true);
                        hLightOff.setChecked(false);
                    } else {
                        hLightOn.setChecked(false);
                        hLightOff.setChecked(true);
                    }
                }
            } else if (signal ==
                    VehicleInterfaceSignals.VIM_AUTOMATIC_HEAD_LIGHTS_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    autoHeadLightOn.setEnabled(false);
                    autoHeadLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        autoHeadLightOn.setChecked(true);
                        autoHeadLightOff.setChecked(false);
                    } else {
                        autoHeadLightOn.setChecked(false);
                        autoHeadLightOff.setChecked(true);
                    }

                }
            } else if (signal ==
                    VehicleInterfaceSignals.VIM_HEAD_LIGHTS_HIGH_BEAM_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    hBeamOn.setEnabled(false);
                    hBeamOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        hBeamOn.setChecked(true);
                        hBeamOff.setChecked(false);
                    } else {
                        hBeamOn.setChecked(false);
                        hBeamOff.setChecked(true);
                    }

                }
            } else if (signal == VehicleInterfaceSignals.VIM_DYNAMIC_HIGH_BEAM_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    autoHighBeamOn.setEnabled(false);
                    autoHighBeamOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        autoHighBeamOn.setChecked(true);
                        autoHighBeamOff.setChecked(false);
                    } else {
                        autoHighBeamOn.setChecked(false);
                        autoHighBeamOff.setChecked(true);
                    }

                }

            } else if (signal == VehicleInterfaceSignals.VIM_LEFT_TURN_LIGHT_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    leftTurnOn.setEnabled(false);
                    leftTurnOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        leftTurnOn.setChecked(true);
                        leftTurnOff.setChecked(false);
                    } else {
                        leftTurnOn.setChecked(false);
                        leftTurnOff.setChecked(true);
                    }
                }

            } else if (signal == VehicleInterfaceSignals.VIM_RIGHT_TURN_LIGHT_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    rightTurnOn.setEnabled(false);
                    rightTurnOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        rightTurnOn.setChecked(true);
                        rightTurnOff.setChecked(false);
                    } else {
                        rightTurnOn.setChecked(false);
                        rightTurnOff.setChecked(true);
                    }

                }
            } else if (signal == VehicleInterfaceSignals.VIM_BRAKE_LIGHT_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    brkLightOn.setEnabled(false);
                    brkLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        brkLightOn.setChecked(true);
                        brkLightOff.setChecked(false);
                    } else {
                        brkLightOn.setChecked(false);
                        brkLightOff.setChecked(true);
                    }

                }

            } else if (signal ==
                    VehicleInterfaceSignals.VIM_LIGHT_STATUS_FOG_FRONT_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    fongFrontLightOn.setEnabled(false);
                    fongFrontLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        fongFrontLightOn.setChecked(true);
                        fongFrontLightOff.setChecked(false);
                    } else {
                        fongFrontLightOn.setChecked(false);
                        fongFrontLightOff.setChecked(true);
                    }

                }

            } else if (signal ==
                    VehicleInterfaceSignals.VIM_LIGHT_STATUS_FOG_REAR_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    fogRearLightOn.setEnabled(false);
                    fogRearLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        fogRearLightOn.setChecked(true);
                        fogRearLightOff.setChecked(false);
                    } else {
                        fogRearLightOn.setChecked(false);
                        fogRearLightOff.setChecked(true);
                    }

                }

            } else if (signal ==
                    VehicleInterfaceSignals.VIM_HAZARD_LIGHT_STATUS_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    hazardLightOn.setEnabled(false);
                    hazardLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        hazardLightOn.setChecked(true);
                        hazardLightOff.setChecked(false);
                    } else {
                        hazardLightOn.setChecked(false);
                        hazardLightOff.setChecked(true);
                    }

                }

            } else if (signal == VehicleInterfaceSignals.VIM_INTR_LIGHT_DRIVER_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    driverLightOn.setEnabled(false);
                    driverLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        driverLightOn.setChecked(true);
                        driverLightOff.setChecked(false);
                    } else {
                        driverLightOn.setChecked(false);
                        driverLightOff.setChecked(true);
                    }

                }

            } else if (signal == VehicleInterfaceSignals.VIM_INTR_LIGHT_PSNGR_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    psngrLightOn.setEnabled(false);
                    psngrLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        psngrLightOn.setChecked(true);
                        psngrLightOff.setChecked(false);
                    } else {
                        psngrLightOn.setChecked(false);
                        psngrLightOff.setChecked(true);
                    }

                }

            } else if (signal == VehicleInterfaceSignals.VIM_INTR_LIGHT_CENTER_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    centerLightOn.setEnabled(false);
                    centerLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        centerLightOn.setChecked(true);
                        centerLightOff.setChecked(false);
                    } else {
                        centerLightOn.setChecked(false);
                        centerLightOff.setChecked(true);
                    }

                }

            } else if (signal == VehicleInterfaceSignals.VIM_INTR_LIGHT_3RD_ROW_SIGNAL) {
                if (isWritableSignal(signal) != true) {
                    thirdRowLightOn.setEnabled(false);
                    thirdRowLightOff.setEnabled(false);
                }
                if (data != null) {
                    if (Integer.parseInt(data) > 0) {
                        thirdRowLightOn.setChecked(true);
                        thirdRowLightOff.setChecked(false);
                    } else {
                        thirdRowLightOn.setChecked(false);
                        thirdRowLightOff.setChecked(true);
                    }

                }
            }
            index++;
        }
    }

    private void testVehicleManager() {
        if (mVehicleManager == null) {
            Log.e(TAG, "mVehicleManager is NULL....");
            return;
        }
        mInterfaceData = mVehicleManager.getInterfaceHandle();
        if (mInterfaceData != null) {
            mDataNotification = new VehicleDataNotification();
            mSupportedSignals = mInterfaceData.getSupportedSignalIds();
            mWritableSignals  = mInterfaceData.getWritableSignalIds();
            mStatusUiInterestedSignals = new int[] {
                    VehicleInterfaceSignals.VIM_TRANSMISSION_GEAR_STATUS_SIGNAL,
                    VehicleInterfaceSignals.VIM_VEHICLE_TYPE_SIGNAL,
                    VehicleInterfaceSignals.VIM_RMNG_FUEL_LVL_SIGNAL,
                    VehicleInterfaceSignals.VIM_SPEEDO_METER_SIGNAL,
                    VehicleInterfaceSignals.VIM_ENGINE_SPEED_SIGNAL,
                    VehicleInterfaceSignals.VIM_TRIP_METER_1_MILEAGE_SIGNAL,
                    VehicleInterfaceSignals.VIM_TRIP_METER_2_MILEAGE_SIGNAL,
                    VehicleInterfaceSignals.VIM_TRIP_METER_1_AVG_SPEED_SIGNAL,
                    VehicleInterfaceSignals.VIM_TRIP_METER_2_AVG_SPEED_SIGNAL,
                    VehicleInterfaceSignals.
                    VIM_TRIP_METER_1_FUEL_CONSUMPTION_SIGNAL,
                    VehicleInterfaceSignals.
                    VIM_TRIP_METER_2_FUEL_CONSUMPTION_SIGNAL,
                    VehicleInterfaceSignals.VIM_CRUISE_CONTROL_STATUS_SIGNAL
            };
            mStatusUiNotificationTypes = new VehicleFrameworkNotificationType[] {
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_PERIODIC_SIGNAL_VALUE_UPDATE,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED
            };
            mStatusUiNotificationRate =
                    new int[mStatusUiInterestedSignals.length];
            for(int i = 0; i < mStatusUiInterestedSignals.length; i++) {
                mStatusUiNotificationRate[i] = CALL_BACK_INTERVAL;
            }
            mLightsUiInterestedSignals = new int[] {
                    VehicleInterfaceSignals.VIM_HEAD_LIGHTS_SIGNAL,
                    VehicleInterfaceSignals.
                    VIM_AUTOMATIC_HEAD_LIGHTS_SIGNAL,
                    VehicleInterfaceSignals.
                    VIM_HEAD_LIGHTS_HIGH_BEAM_SIGNAL,
                    VehicleInterfaceSignals.VIM_DYNAMIC_HIGH_BEAM_SIGNAL,
                    VehicleInterfaceSignals.VIM_LEFT_TURN_LIGHT_SIGNAL,
                    VehicleInterfaceSignals.VIM_RIGHT_TURN_LIGHT_SIGNAL,
                    VehicleInterfaceSignals.VIM_BRAKE_LIGHT_SIGNAL,
                    VehicleInterfaceSignals.
                    VIM_LIGHT_STATUS_FOG_FRONT_SIGNAL,
                    VehicleInterfaceSignals.
                    VIM_LIGHT_STATUS_FOG_REAR_SIGNAL,
                    VehicleInterfaceSignals.VIM_HAZARD_LIGHT_STATUS_SIGNAL,
                    VehicleInterfaceSignals.VIM_INTR_LIGHT_DRIVER_SIGNAL,
                    VehicleInterfaceSignals.VIM_INTR_LIGHT_PSNGR_SIGNAL,
                    VehicleInterfaceSignals.VIM_INTR_LIGHT_CENTER_SIGNAL,
                    VehicleInterfaceSignals.VIM_INTR_LIGHT_3RD_ROW_SIGNAL
            };
            mLightsUiNotificationTypes = new VehicleFrameworkNotificationType[] {
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED
            };
            mLightsUiNotificationRate =
                    new int[mLightsUiInterestedSignals.length];
            for(int i = 0; i < mLightsUiInterestedSignals.length; i++) {
                mLightsUiNotificationRate[i] = CALL_BACK_INTERVAL;
            }
            mTransmissionUiInterestedSignals = new int[] {
                    VehicleInterfaceSignals.
                    VIM_VEHICLE_POWER_MODE_SIGNAL,
                    VehicleInterfaceSignals.VIM_MEASUREMENT_FUEL_SIGNAL,
                    VehicleInterfaceSignals.
                    VIM_RMNG_DRVNG_RANGE_SIGNAL,
                    VehicleInterfaceSignals.VIM_ENGN_OIL_RMNG_SIGNAL,
                    VehicleInterfaceSignals.VIM_COOLANT_LVL_SIGNAL,
                    VehicleInterfaceSignals.VIM_COOLANT_TEMP_SIGNAL,
                    VehicleInterfaceSignals.VIM_STRNG_WHEEL_ANGLE_SIGNAL
            };
            mTransmissionUiNotificationTypes = new VehicleFrameworkNotificationType[] {
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED
            };
            mTransmissionUiNotificationRate =
                    new int[mTransmissionUiInterestedSignals.length];
            for(int i = 0; i < mTransmissionUiInterestedSignals.length; i++) {
                mTransmissionUiNotificationRate[i] = CALL_BACK_INTERVAL;
            }

            mMaintenanceUiInterestedSignals = new int[] {
                    VehicleInterfaceSignals.VIM_ODOMETER_SIGNAL,
                    VehicleInterfaceSignals.VIM_TRANSMISSION_OIL_LIFE_LVL_SIGNAL,
                    VehicleInterfaceSignals.VIM_TRANSMISSION_OIL_TEMP_SIGNAL,
                    VehicleInterfaceSignals.VIM_BRAKE_FLUID_LVL_SIGNAL,
                    VehicleInterfaceSignals.VIM_WASHER_FLUID_LVL_SIGNAL,
                    VehicleInterfaceSignals.VIM_MALFUNCTION_INDICATOR_SIGNAL,
                    VehicleInterfaceSignals.VIM_TIRE_PRESSURE_FRONT_LEFT_SIGNAL,
                    VehicleInterfaceSignals.VIM_TIRE_PRESSURE_FRONT_RIGHT_SIGNAL,
                    VehicleInterfaceSignals.VIM_TIRE_PRESSURE_REAR_LEFT_SIGNAL,
                    VehicleInterfaceSignals.VIM_TIRE_PRESSURE_REAR_RIGHT_SIGNAL,
                    VehicleInterfaceSignals.VIM_BATTERY_VOLTAGE_SIGNAL,
                    VehicleInterfaceSignals.VIM_BATTERY_CURRENT_SIGNAL
            };
            mMaintUiNotificationTypes = new VehicleFrameworkNotificationType[] {
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED,
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED
            };
            mMaintUiNotificationRate =
                    new int[mMaintenanceUiInterestedSignals.length];
            for(int i = 0; i < mMaintenanceUiInterestedSignals.length; i++) {
                mMaintUiNotificationRate[i] = CALL_BACK_INTERVAL;
            }
            mInterfaceData.registerHandler(mStatusUiInterestedSignals, mDataNotification,
                    mStatusUiNotificationTypes, mStatusUiNotificationRate);
            mInterfaceData.registerHandler(mLightsUiInterestedSignals, mDataNotification,
                    mLightsUiNotificationTypes, mLightsUiNotificationRate);
            mInterfaceData.registerHandler(mTransmissionUiInterestedSignals, mDataNotification,
                    mTransmissionUiNotificationTypes, mTransmissionUiNotificationRate);
            mInterfaceData.registerHandler(mMaintenanceUiInterestedSignals, mDataNotification,
                    mMaintUiNotificationTypes, mMaintUiNotificationRate);
        } else {
            Log.e(TAG, "getInterfaceHandle returned null...");
        }
    }
    private boolean isWritableSignal(int signalId) {
        if (mWritableSignals != null) {
            for( int oSignal : mWritableSignals) {
                if (signalId == oSignal) {
                    return true;
                }
            }
        }
        return false;
    }

    private void readInterfaceData() {
        if (mCurrentUi == R.id.statusBtn) {
            updateStatusUi();
        } else if (mCurrentUi == R.id.transmissionBtn) {
            updateTransmissionUi();
        } else if (mCurrentUi == R.id.maintenanceBtn) {
            updateMaintenanceUi();
        } else if (mCurrentUi == R.id.lightsBtn) {
            updateLightsUi();
        } else {
            //Log.v(TAG, "readInterfaceData mCurrentUi=" + mCurrentUi);
        }
    }
}
