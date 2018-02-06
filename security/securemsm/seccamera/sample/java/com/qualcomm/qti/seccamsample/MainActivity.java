/*=============================================================================
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

package com.qualcomm.qti.seccamsample;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.res.Configuration;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.LinearLayout;
import android.graphics.Bitmap;
import android.view.View;
import android.view.Gravity;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.support.v4.content.ContextCompat;
import android.support.v4.app.ActivityCompat;

import com.qualcomm.qti.seccamsample.R;
import com.qualcomm.qti.seccamapi.SecCamServiceClient;

public class MainActivity extends Activity implements SecCamServiceClient.ClientCallback {

    private static boolean isServiceActive = false;
    private static final Integer isServiceActiveDelay = 3000; // Time to wait (in ms) for the serviceConnected callback to be called
    private static final int REQUEST_CAMERA_PERMISSION = 0; // Id to identify a camera permission request

    private OnClickListener startPreviewButtonClickListener = new OnClickListener() {
        @Override
        public void onClick(View v) {
            startPreview();
        }
    };

    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {

        switch (requestCode) {
            case REQUEST_CAMERA_PERMISSION: {

                // In case the user declines to give camera permissions to the app, a notification is displayed,
                // and the application closes.
                if ((grantResults.length == 0) || (grantResults[0] != PackageManager.PERMISSION_GRANTED)) {
                    AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                    builder.setTitle("");
                    builder.setMessage("This app does not have critical permissions needed to run. Please check your permissions settings");
                    builder.setPositiveButton("OK", null);
                    AlertDialog dialog = builder.show();

                    TextView messageView = (TextView)dialog.findViewById(android.R.id.message);
                    messageView.setGravity(Gravity.CENTER);

                    final Button okButton = dialog.getButton(AlertDialog.BUTTON_POSITIVE);
                    okButton.setOnClickListener(new View.OnClickListener() {
                        public void onClick(View v) {
                            finish();
                            System.exit(0);
                        }
                    });
                    LinearLayout.LayoutParams okButtonLL = (LinearLayout.LayoutParams) okButton.getLayoutParams();
                    okButtonLL.width = ViewGroup.LayoutParams.MATCH_PARENT;
                    okButton.setLayoutParams(okButtonLL);

                }
                break;
            }
            default:
                break;
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        findViewById(R.id.start_secure_preview).setOnClickListener(startPreviewButtonClickListener);
        findViewById(R.id.start_secure_preview).setEnabled(false);
        findViewById(R.id.done_button).setOnClickListener(mExitButtonClickListener);

        int permissionCheck = ContextCompat.checkSelfPermission(MainActivity.this,
        android.Manifest.permission.CAMERA);

        // Validate camera permissons are granted, otherwise request it during runtime
        if (permissionCheck != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(MainActivity.this,
                new String[]{android.Manifest.permission.CAMERA},
                REQUEST_CAMERA_PERMISSION);
        }

        // It is required to recieve the instance of the Service client.
        // This results in binding to the Secure camera service.
        SecCamServiceClient.getInstance().start(this, this);

        // Handler to be called after isServiceActiveDelay miliseconds, showing an indication that binding
        // to the Secure camera service was not successful. This may be caused if the TA (seccamdemo64) is unavailable.
        Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            public void run() {
                if (isServiceActive == false) {
                    AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                    builder.setTitle("");
                    builder.setMessage("Secure camera service is inactive\n\n Validate the TA (seccamdemo64)\n is available");
                    builder.setPositiveButton("OK", null);
                    AlertDialog dialog = builder.show();

                    TextView messageView = (TextView)dialog.findViewById(android.R.id.message);
                    messageView.setGravity(Gravity.CENTER);

                    final Button okButton = dialog.getButton(AlertDialog.BUTTON_POSITIVE);
                    LinearLayout.LayoutParams okButtonLL = (LinearLayout.LayoutParams) okButton.getLayoutParams();
                    okButtonLL.width = ViewGroup.LayoutParams.MATCH_PARENT;
                    okButton.setLayoutParams(okButtonLL);
                }
            }
        }, isServiceActiveDelay);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // Release the instance of the Service client when the activity is destroyed.
        // This results in unbinding from the Secure camera service.
        SecCamServiceClient.getInstance().release();
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        View view =  this.getWindow().getDecorView();
    }

    private void startPreview() {
        startActivityForResult(new Intent(MainActivity.this, CameraActivity.class), 1);
    }

    private OnClickListener mExitButtonClickListener = new OnClickListener() {
        @Override
        public void onClick(View v) {
            finish();
            System.exit(0);
        }
    };

    @Override
    public void serviceConnected() {
        // Indicates that the service is available and connected.
        // Secure Camera can be activated after this callback occurs.
        findViewById(R.id.start_secure_preview).setEnabled(true);
        isServiceActive = true;
    }

    @Override
    public void serviceDisconnected() {
        // This callback is called if there is some issue connecting to the service. once it is triggered
        // the application must wait for the serviceConnected() callback to be called, before starting Secure Camera.
        findViewById(R.id.start_secure_preview).setEnabled(false);
        isServiceActive = false;
    }
}