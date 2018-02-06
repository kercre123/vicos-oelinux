/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

package com.qualcomm.qti.vehicle.explorer;

import java.util.Arrays;

import com.qualcomm.qti.VehicleFrameworkNotificationType;
import com.qualcomm.qti.VehicleInterfaceDataHandler;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

public class TestingActivity extends Activity implements OnClickListener {
    private native String nativeSecurityTest();
    private static final String LOG_TAG = "TestingActivity";
    private Button mSecurityTestButton;
    private TextView mSecurityTestResult, mRegisterResult;
    private Button mRegisterButton, mUnregisterButton;
    private EditText mSignalIdEditText;
    private CheckBox mOnChangeCheckbox;
    private VehicleInterfaceDataHandler mDataNotification = new TestActivityHandler();

    @Override
    public void onCreate(Bundle icicle) {
        super.onCreate(icicle);
        setContentView(R.layout.testing_activity);
        mSecurityTestButton = (Button)findViewById(R.id.securityTestButton);
        mSecurityTestButton.setOnClickListener(this);
        mSecurityTestResult  = (TextView)findViewById(R.id.securityTestResult);
        mRegisterButton = (Button)findViewById(R.id.registerButton);
        mRegisterButton.setOnClickListener(this);
        mUnregisterButton = (Button)findViewById(R.id.unregisterButton);
        mUnregisterButton.setOnClickListener(this);
        mSignalIdEditText = (EditText)findViewById(R.id.signalIdEditText);
        mRegisterResult = (TextView)findViewById(R.id.regTestResult);
        mOnChangeCheckbox = (CheckBox)findViewById(R.id.onChangeCheckBox);
    }

    @Override
    public void onClick(View v) {
        boolean result;
        switch (v.getId()) {
            case R.id.securityTestButton:
                Log.d(LOG_TAG, "Socket CAN security test button clicked");
                String response = nativeSecurityTest();
                Log.d(LOG_TAG, "respose: " + response);
                mSecurityTestResult.setText(response);
                break;
            case R.id.registerButton:
                Log.d(LOG_TAG, "registerButton clicked");
                int[] signal = getSignalId();
                VehicleFrameworkNotificationType[] type = getNotificationType();

                int[] rate = new int[] {1000};

                result = VehicleExplorer.mInterfaceData.registerHandler(signal, mDataNotification, type, rate);
                mRegisterResult.setText("Registration result: " + result);
                break;
            case R.id.unregisterButton:
                Log.d(LOG_TAG, "unregisterButton clicked");
                int[] unregSignal = getSignalId();
                result = VehicleExplorer.mInterfaceData.removeHandler(unregSignal, mDataNotification);
                mRegisterResult.setText("Remove result: " + result);
                break;
        }
    }

    private int[] getSignalId() {
        int[] signalId = new int[] {1117};
        String text = mSignalIdEditText.getText().toString();
        if (text != null && !text.isEmpty()) {
            try {
                signalId = new int[] {Integer.parseInt(text)};
            } catch (NumberFormatException e) { }
        }
        Log.d(LOG_TAG, "signalId is " + Arrays.toString(signalId));
        return signalId;
    }

    private VehicleFrameworkNotificationType[] getNotificationType() {
        boolean onChange = mOnChangeCheckbox.isChecked();
        VehicleFrameworkNotificationType[] type;
        if (onChange) {
            Log.d(LOG_TAG, "registerButton on_change");
            type = new VehicleFrameworkNotificationType[] {
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_CHANGED};
        } else {
            Log.d(LOG_TAG, "registerButton on_refresh");
            type = new VehicleFrameworkNotificationType[] {
                    VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED};
        }
        return type;
    }

    static {
        System.loadLibrary("ve_jni");
    }

    private class TestActivityHandler implements VehicleInterfaceDataHandler {
        public void onNotify(VehicleFrameworkNotificationType type, int signalId) {
            Log.d(LOG_TAG, "TestActivityHandler onNotify type " + type + " signalId " + signalId);
        }

        public void onError(boolean bCleanUpAndRestart) {
            Log.d(LOG_TAG, "TestActivityHandler onError bCleanUpAndRestart " + bCleanUpAndRestart);
        }
    }
}
