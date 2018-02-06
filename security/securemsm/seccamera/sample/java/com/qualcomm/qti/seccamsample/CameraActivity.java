/*=============================================================================
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

package com.qualcomm.qti.seccamsample;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import android.app.Activity;
import android.content.res.Configuration;
import android.os.Bundle;
import android.os.AsyncTask;
import android.util.Log;
import android.hardware.Camera;
import android.hardware.Camera.CameraInfo;
import android.hardware.Camera.PictureCallback;
import android.widget.Button;
import android.widget.Toast;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.WindowManager;
import android.graphics.ImageFormat;

import com.qualcomm.qti.seccamsample.R;
import com.qualcomm.qti.seccamapi.SecureSurface;
import com.qualcomm.qti.seccamapi.SecureCameraSurface;
import com.qualcomm.qti.seccamapi.SecCamServiceClient;
import com.qualcomm.qti.seccamapi.SecCamServiceVendorClient;

public class CameraActivity extends Activity implements PictureCallback, SurfaceHolder.Callback {

    private Camera previewCamera;
    private Camera captureCamera;
    private SurfaceView cameraPreview;
    private SurfaceHolder surfaceHolder;
    private SecureCameraSurface secureCameraPreviewSurface;
    private SecureCameraSurface secureCameraCaptureSurface;

    private static final Integer previewNumOfBuffers = 2;
    private static final Integer previewWidth = 1280;
    private static final Integer previewHeight = 960;
    private static final Integer previewFormat = SecureSurface.IMAGE_FORMAT_YUV420SP;
    private static final Integer previewSurfaceNumOfBuffers = 3;
    private static final Integer previewCameraID = CameraInfo.CAMERA_FACING_FRONT;
    private static Integer previewRotation = SecureSurface.ROTATE_90_VERTICAL_FLIP; //rotate source image 90 degrees clockwise + vertical flip

    private static final Integer captureNumOfBuffers = 9;
    private static final Integer captureFormat = SecureSurface.IMAGE_FORMAT_RAW;
    private static final Integer captureCameraID = 2; //2 is the index for the front IR camera on MTP

    // Setting captureCameraEnabled to false will allow to only test secure preview
    private static final boolean captureCameraEnabled = true;

    private StartTask mStartTask = null;
    private EndTask mEndTask = null;

    private static final String LOG_TAG = "SECCAM-SAMPLE-APP";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Prevent screen from going to sleep during preview
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.activity_camera);
        cameraPreview = (SurfaceView) findViewById(R.id.preview_view);
        surfaceHolder = cameraPreview.getHolder();
        surfaceHolder.addCallback(this);
        surfaceHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);

        final Button doneButton = (Button) findViewById(R.id.done_button);
        doneButton.setOnClickListener(mDoneButtonClickListener);

        if (mStartTask == null) {
            mStartTask = new StartTask();
            mStartTask.execute();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    protected void onStart() {
        super.onStart();
    }

    @Override
    protected void onStop() {
        super.onStop();
    }

    @Override
    protected void onResume() {
        super.onResume();
        cameraPreview.setVisibility(SurfaceView.VISIBLE);
        // Since the camera resources are released when the app is paused, the camera and the preview must be reopened on resume
        if (mStartTask == null) {
            mStartTask = new StartTask();
            mStartTask.execute();
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        cameraPreview.setVisibility(SurfaceView.GONE);
        // It is vital to release camera resources when the app is not active, otherwise a runtime exception is thrown
         if (mEndTask == null) {
             mEndTask = new EndTask();
             mEndTask.execute();
         }
    }

    private OnClickListener mDoneButtonClickListener = new OnClickListener() {
        @Override
        public void onClick(View v) {
            // Hide the SufaceView before stopping the secure preview. This is done to avoid a green screen which may appear
            // as a result of cleaning the preview buffers, while still displaying them on the screen.
            cameraPreview.setVisibility(SurfaceView.GONE);
            final Button doneButton = (Button) findViewById(R.id.done_button);
            doneButton.setVisibility(View.GONE);

            if (mEndTask == null) {
                mEndTask = new EndTask();
                mEndTask.execute();
            }

            finish();
        }
    };

    private String getMaxRdiResolution() {

        if (captureCamera == null) {
            throw new NullPointerException("Capture camera is undefined");
        }

        List<Camera.Size> sizes = new ArrayList<Camera.Size>();

        Camera.Parameters params = captureCamera.getParameters();
        Integer[][] rdiResolutions = {{0, 0}};
        // Receive the RAW resolution from the sensor
        String maxResolution = params.get("raw-size");

        return maxResolution;
    }

    private void dispatchVendorCommand_exchangeTimestamps() {
        // Set a bundle with the data needed for the command.
        Bundle timestamp_bundle = new Bundle();
        timestamp_bundle.putLong("hlosTimestamp", System.currentTimeMillis());

        // Dispatch the command using the vendor command ID, and the created bundle.
        SecCamServiceClient.getInstance().dispatchVendorCommand(SecCamServiceVendorClient.MSG_VENDOR_EXCHANGE_TIMESTAMP, timestamp_bundle);
    }

    public static int getCameraRotation(int cameraId, Camera camera) {

        int rotation = 0;
        Camera.CameraInfo info = new Camera.CameraInfo();
        Camera.getCameraInfo(cameraId, info);

        int defaultRotation = (info.orientation) % 360;
        switch (defaultRotation) {
            case 0:
                rotation = SecureSurface.FLIP_HORIZONTALLY;
                break;
            case 90:
                rotation = SecureSurface.ROTATE_90_VERTICAL_FLIP;
                break;
            case 180:
                rotation = SecureSurface.FLIP_VERTICALLY;
                break;
            case 270:
                rotation = SecureSurface.ROTATE_90_HORIZONTAL_FLIP;
                break;
            default:
                rotation = SecureSurface.NO_ROTATION;
        }

        return rotation;
    }

    private class StartTask extends AsyncTask<String, Void, Boolean> {

        protected void onPreExecute() {
        }

        protected Boolean doInBackground(String... tmpStr) {
            try {

                // Usage of a vendor custom command, exchanging timestamps between HLOS and TZ.
                dispatchVendorCommand_exchangeTimestamps();

                if (captureCameraEnabled && (captureCamera == null)) {

                        try {
                            // openLegacy is mandatory in order to open the camera in HAL1 API
                            // first parameter is the camera ID, dependent on the cameras available for the target
                            captureCamera = Camera.openLegacy(captureCameraID, Camera.CAMERA_HAL_API_VERSION_1_0);
                        } catch (Exception e) {
                            // In case capture camera is unsupported, capture is disabled and only preview will be shown
                            captureCamera = null;
                        }

                        if (captureCamera != null) {
                            Camera.Parameters params = captureCamera.getParameters();

                            // In case of RDI, the camera should be configured to use the highest resolution supported by the sensor.
                            String rdiResolution = getMaxRdiResolution();
                            String[] rdiDimensions = rdiResolution.split("x");
                            if (rdiDimensions.length != 2) {
                                throw new Exception("Invalid max resolution obtained from RDI camera " + captureCameraID);
                            }

                            Integer maxRdiWidth = Integer.parseInt(rdiDimensions[0]);
                            Integer maxRdiHeight = Integer.parseInt(rdiDimensions[1]);

                            // Set a new Secure Camera Surface. The allocated capture buffer size will be (maxRdiWidth x maxRdiHeight).
                            secureCameraCaptureSurface = new SecureCameraSurface(captureCameraID,
                                params, maxRdiWidth, maxRdiHeight, captureFormat, captureNumOfBuffers);

                            // RDI resolution is not supported directly by HAL1, therefore any of the supported values can be set.
                            // This will not affect the actual RDI frame resolution.
                            List<Camera.Size> supportedSizes = params.getSupportedPictureSizes();
                            Integer captureWidth = supportedSizes.get(0).width;
                            Integer captureHeight = supportedSizes.get(0).height;

                            params.setPreviewSize(captureWidth, captureHeight);
                            params.setPreviewFormat(ImageFormat.NV21);
                            captureCamera.setParameters(params);

                            captureCamera.setPreviewDisplay(secureCameraCaptureSurface.getSurfaceHolder());
                            captureCamera.startPreview();
                        }
                }

                if (previewCamera == null) {

                        previewCamera = Camera.openLegacy(previewCameraID, Camera.CAMERA_HAL_API_VERSION_1_0);
                        Camera.Parameters params = previewCamera.getParameters();

                        // Get the preview rotation needed, according to the default rotation of the camera
                        previewRotation = getCameraRotation(previewCameraID, previewCamera);

                        // Validate that the defined preview resolution is supported by the camera
                        boolean isPreviewResolutionValid = false;
                        List<Camera.Size> supportedSizes = params.getSupportedPictureSizes();
                        for (Camera.Size size : supportedSizes){
                            if ((previewWidth == size.width) && (previewHeight == size.height)) {
                                isPreviewResolutionValid = true;
                            }
                        }

                        if (isPreviewResolutionValid == false) {
                            throw new Exception("Resolution (" + previewWidth + "," + previewHeight + ") unsupported by camera " + previewCameraID);
                        }

                        // Set a new Secure Camera Surface. The allocated preview buffer size will be (previewWidth x previewHeight).
                        secureCameraPreviewSurface = new SecureCameraSurface(previewCameraID,
                            params, previewWidth, previewHeight, previewFormat, previewNumOfBuffers);

                        params.setPreviewSize(previewWidth, previewHeight);
                        params.setPreviewFormat(ImageFormat.NV21);
                        previewCamera.setParameters(params);

                        Surface previewSurface = cameraPreview.getHolder().getSurface();
                        // assignPreviewSurface Sets the connection between the Secure Camera Surface, and the Preview Surface
                        secureCameraPreviewSurface.assignPreviewSurface(surfaceHolder, previewWidth, previewHeight, previewFormat, previewRotation, previewSurfaceNumOfBuffers);
                        previewCamera.setPreviewDisplay(secureCameraPreviewSurface.getSurfaceHolder());
                        previewCamera.startPreview();
                }
            } catch (Exception e) {
                Log.e(LOG_TAG, "StartTask Exception caught", e);
                return Boolean.FALSE;
            }

            return Boolean.TRUE;
        }

        protected void onPostExecute(Boolean result) {
            mStartTask = null;
        }

    }

    private class EndTask extends AsyncTask<String, Void, Boolean> {

        protected void onPreExecute() {
        }

        protected Boolean doInBackground(String... tmpStr) {

            try {
                if (previewCamera != null) {
                    previewCamera.stopPreview();
                    previewCamera.release();
                    previewCamera = null;
                }

                if (secureCameraPreviewSurface != null) {
                    secureCameraPreviewSurface.release();
                    secureCameraPreviewSurface = null;
                }

                if (captureCamera != null) {
                    captureCamera.stopPreview();
                    captureCamera.release();
                    captureCamera = null;
                }

                if (secureCameraCaptureSurface != null) {
                    secureCameraCaptureSurface.release();
                    secureCameraCaptureSurface = null;
                }
            } catch (Exception e) {
                Log.e(LOG_TAG, "EndTask Exception caught", e);
                return Boolean.FALSE;
            }

            return Boolean.TRUE;
        }

        protected void onPostExecute(Boolean result) {
            mEndTask = null;
        }

    }

    // Mandatory abstract method stubs

    @Override
    public void onPictureTaken(byte[] data, Camera camera) {
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
    }
}