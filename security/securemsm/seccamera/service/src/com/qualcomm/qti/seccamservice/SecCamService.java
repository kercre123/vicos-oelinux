// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All Rights Reserved.
// Confidential and Proprietary - Qualcomm Technologies, Inc.
package com.qualcomm.qti.seccamservice;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Map;
import java.util.Hashtable;
import android.app.Service;
import android.content.Intent;
import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.Parcelable;
import android.os.RemoteException;
import android.os.SystemClock;
import android.util.Log;
import android.view.Surface;
import android.widget.Chronometer;

import com.qualcomm.qti.seccamservice.SecCamServiceVendorHandler;

public class SecCamService extends Service {

    public static final int JNI_OK = 0;
    public static final int JNI_EPERM = -1;
    public static final int JNI_EINVAL = -22;
    public static final int JNI_ETIMEDOUT = -110;
    private final static String LOG_TAG = "SECCAM-SERVICE";
    private static final String TZ_APP_NAME = "seccamdemo64";
    private static final int MSG_GET_TIMESTAMP = 1000;
    private static final int MSG_GET_CAPTURE_SURFACE = 1001;
    private static final int MSG_SET_PREVIEW_SURFACE = 1002;
    private static final int MSG_RELEASE_CAPTURE_SURFACE = 1003;
    private static final int MSG_RELEASE_PREVIEW_SURFACE = 1004;
    private static final int MSG_ENABLE_FRAME_CALLBACK = 1005;
    private static final int MSG_FRAME_CALLBACK = 1006;

    public static final int MAX_RETURN_PARAMS = 32;

    private class TZappInfo {
        public int verMaj_ = 0;
        public int verMin_ = 0;
        public String appName_;
    }

    static public class FrameInfo {
        public long frameNumber_ = 0;
        public long timeStamp_ = 0;
        public int width_ = 0;
        public int height_= 0;
        public int stride_ = 0;
        public int format_ = 0;
    }

    TZappInfo tzAppInfo_ = null;
    static private Context serviceContext = null;
    private boolean tzAppIfAvalable_ = false;
    private Chronometer chronometer_;
    static private Hashtable<Long, Thread> frameCallbacks_ = null;

    //=========================================================================
    // JNI API
    //=========================================================================
    private static native Surface getSecureCameraSurface(
            int cameraId, int width, int height, int format, int numOfBuffers);
    private static native boolean releaseCaptureSurface(Surface secureSurface);
    private static native boolean setSecurePreviewSurface(Surface previewSurface, Surface captureSurface,
            int width, int height, int format, int rotation, int numOfBuffers);
    private static native boolean releasePreviewSurface(Surface previewSurface, Surface captureSurface);

    private native int doStartTzApp(String str, TZappInfo appInfo);
    private native int doShutdownTzApp();

    private static native long enableFrameCallback(Surface secureSurface);
    private static native int isFrameAvailable(long surfaceId, int timeout,
            FrameInfo frameInfo, long[] returnParams);

    static {
        System.loadLibrary("seccam");
    }

    //=========================================================================
    //
    //=========================================================================
    static class SecCamServiceHandler extends Handler {
         private final WeakReference<SecCamService> service_;

         public SecCamServiceHandler(SecCamService service) {
             service_ = new WeakReference<SecCamService>(service);
         }

         //=========================================================================
         //
         //=========================================================================
         private class FrameCallbackThread extends Thread {
            private Messenger callbackActivityMessenger_ = null;
            private long surfaceId_ = 0;
            private int timeout_ = 0;

            public FrameCallbackThread(Messenger callbackActivityMessenger,
                 long surfaceId, int timeout) {
                callbackActivityMessenger_ = callbackActivityMessenger;
                surfaceId_ = surfaceId;
                timeout_ = timeout;
            }

            @Override
            public void run() {
                SecCamService.FrameInfo frameInfo = new FrameInfo();
                long[] returnParams = new long[MAX_RETURN_PARAMS];

                while(true) {
                    int result = isFrameAvailable(surfaceId_, timeout_,
                            frameInfo, returnParams);
                    if (result == JNI_OK) {
                        Message replyMsg = Message.obtain();
                        replyMsg.what = MSG_FRAME_CALLBACK;
                        Bundle out_bundle = new Bundle();
                        out_bundle.putLong("surfaceId", surfaceId_);
                        out_bundle.putLong("frameNumber", frameInfo.frameNumber_);
                        out_bundle.putLong("timeStamp", frameInfo.timeStamp_);
                        out_bundle.putInt("width", frameInfo.width_);
                        out_bundle.putInt("height", frameInfo.height_);
                        out_bundle.putInt("stride", frameInfo.stride_);
                        out_bundle.putInt("format", frameInfo.format_);
                        out_bundle.putLongArray("returnParams", returnParams);
                        out_bundle.putBoolean("result", true);
                        Log.d(LOG_TAG, "::FrameCallbackThread::run - " +
                                "SurfaceId:" + surfaceId_ + ", " +
                                "FrameId: " + frameInfo.frameNumber_);
                        replyMsg.setData(out_bundle);
                        try {
                            callbackActivityMessenger_.send(replyMsg);
                        } catch (RemoteException e) {
                           e.printStackTrace();
                        }
                    }
                    else if(result != JNI_ETIMEDOUT) {
                        Log.e(LOG_TAG, "The Frame Callback is not available!");
                        break;
                    }
                }
                Log.d(LOG_TAG, "::FrameCallbackThread::run - Done, " +
                        " SurfaceId:" + surfaceId_);
            }
        }

        //=========================================================================
        //
        //=========================================================================
        private void handleMessage_MSG_GET_CAPTURE_SURFACE(Message msg) {
            Log.d(LOG_TAG, "::ihandleMessage_MSG_GET_CAPTURE_SURFACE");
            Bundle in_bundle = msg.getData();
            int cameraId = in_bundle.getInt("cameraId");
            int width = in_bundle.getInt("width");
            int height = in_bundle.getInt("height");
            int format = in_bundle.getInt("format");
            int numOfBuffers = in_bundle.getInt("numOfBuffers");

            Messenger activityMessenger = msg.replyTo;
            Message replyMsg = Message.obtain();
            replyMsg.what = MSG_GET_CAPTURE_SURFACE;
            Bundle out_bundle = new Bundle();
            Surface surface = getSecureCameraSurface(cameraId, width, height, format, numOfBuffers);
            Log.d(LOG_TAG, "::handleMessage_MSG_GET_CAPTURE_SURFACE =" + surface.toString());
            out_bundle.putParcelable("SURFACE", surface);
            replyMsg.setData(out_bundle);
            try {
                activityMessenger.send(replyMsg);
            } catch (RemoteException e) {
                e.printStackTrace();
            }
        }

        //=========================================================================
        //
        //=========================================================================
        private void handleMessage_MSG_SET_PREVIEW_SURFACE(Message msg) {
            Log.d(LOG_TAG, "::handleMessage_MSG_SET_PREVIEW_SURFACE");
            Bundle in_bundle = msg.getData();
            int width = in_bundle.getInt("width");
            int height = in_bundle.getInt("height");
            int format = in_bundle.getInt("format");
            int rotation = in_bundle.getInt("rotation");
            int numOfBuffers = in_bundle.getInt("numOfBuffers");

            in_bundle.setClassLoader(this.getClass().getClassLoader());
            Surface captureSurface = null;
            Surface previewSurface = null;
            Parcelable parcelable = in_bundle.getParcelable("CSURFACE");
            if (parcelable instanceof Surface) {
                captureSurface = (Surface) parcelable;
                Log.d(LOG_TAG, "::handleMessage_MSG_SET_PREVIEW_SURFACE - CaptureSurface:" + captureSurface.toString());
            }
            parcelable = in_bundle.getParcelable("PSURFACE");
            if (parcelable instanceof Surface) {
                previewSurface = (Surface) parcelable;
                Log.d(LOG_TAG, "::handleMessage_MSG_SET_PREVIEW_SURFACE - PreviewSurface:" + previewSurface.toString());
            }

            Messenger activityMessenger = msg.replyTo;
            Message replyMsg = Message.obtain();
            replyMsg.what = MSG_SET_PREVIEW_SURFACE;
            Bundle out_bundle = new Bundle();
            boolean result = setSecurePreviewSurface(previewSurface, captureSurface, width, height, format, rotation, numOfBuffers);

            //The parcelable instance increases the Java object reference count for the Surface,
            //therefore we release the recieved surface object. This assures the surface is released
            //when the preview session is completed. A reference to the surface is kept within the JNI, until
            //it is released during the MSG_RELEASE_PREVIEW_SURFACE handler.
            previewSurface.release();

            Log.v(LOG_TAG, "::handleMessage_MSG_SET_PREVIEW_SURFACE - " + result);
            out_bundle.putBoolean("result", result);
            replyMsg.setData(out_bundle);
            try {
                activityMessenger.send(replyMsg);
            } catch (RemoteException e) {
                e.printStackTrace();
            }
        }

        //=========================================================================
        //
        //=========================================================================
        private void handleMessage_MSG_RELEASE_CAPTURE_SURFACE(Message msg) {
            Log.d(LOG_TAG, "::handleMessage_MSG_RELEASE_CAPTURE_SURFACE");
            Bundle in_bundle = msg.getData();
            in_bundle.setClassLoader(this.getClass().getClassLoader());
            Surface surface = null;
            Long surfaceId = in_bundle.getLong("surfaceId");
            Parcelable parcelable = in_bundle.getParcelable("SURFACE");
            if (parcelable instanceof Surface) {
                surface = (Surface) parcelable;
                Log.d(LOG_TAG, "::handleMessage_MSG_RELEASE_CAPTURE_SURFACE - surface:" + surface.toString());
            }

            if (frameCallbacks_ != null && surfaceId != 0) {
                Log.d(LOG_TAG, "::handleMessage_MSG_RELEASE_CAPTURE_SURFACE - Disable Frame Callback - " + surfaceId);
                Thread val = frameCallbacks_.get(surfaceId);
                try {
                    if(val.isAlive()) {
                        val.stop();
                    }
                } catch (Exception e) {
                    Log.w(LOG_TAG, "::handleMessage_MSG_RELEASE_CAPTURE_SURFACE - Callback thread is not active");
                }
                frameCallbacks_.remove(surfaceId);
            }

            Messenger activityMessenger = msg.replyTo;
            Message replyMsg = Message.obtain();
            replyMsg.what = MSG_RELEASE_CAPTURE_SURFACE;
            Bundle out_bundle = new Bundle();
            boolean result = releaseCaptureSurface(surface);
            Log.d(LOG_TAG, "::handleMessage_MSG_RELEASE_CAPTURE_SURFACE - " + result);
            out_bundle.putBoolean("result", result);
            replyMsg.setData(out_bundle);
            try {
                activityMessenger.send(replyMsg);
            } catch (RemoteException e) {
                e.printStackTrace();
            }
        }

        //=========================================================================
        //
        //=========================================================================
        private void handleMessage_MSG_RELEASE_PREVIEW_SURFACE(Message msg) {
            Log.d(LOG_TAG, "::handleMessage_MSG_RELEASE_PREVIEW_SURFACE");
            Bundle in_bundle = msg.getData();
            in_bundle.setClassLoader(this.getClass().getClassLoader());
            Surface surface = null;

            Surface captureSurface = null;
            Surface previewSurface = null;
            Parcelable parcelable = in_bundle.getParcelable("CSURFACE");
            if (parcelable instanceof Surface) {
                captureSurface = (Surface) parcelable;
                Log.d(LOG_TAG, "::handleMessage_MSG_SET_PREVIEW_SURFACE - CaptureSurface:" + captureSurface.toString());
            }
            parcelable = in_bundle.getParcelable("PSURFACE");
            if (parcelable instanceof Surface) {
                previewSurface = (Surface) parcelable;
                Log.d(LOG_TAG, "::handleMessage_MSG_SET_PREVIEW_SURFACE - PreviewSurface:" + previewSurface.toString());
            }

            Messenger activityMessenger = msg.replyTo;
            Message replyMsg = Message.obtain();
            replyMsg.what = MSG_RELEASE_PREVIEW_SURFACE;
            Bundle out_bundle = new Bundle();
            boolean result = releasePreviewSurface(previewSurface, captureSurface);
            Log.d(LOG_TAG, "::handleMessage_MSG_RELEASE_PREVIEW_SURFACE - " + result);
            out_bundle.putBoolean("result", result);
            replyMsg.setData(out_bundle);
            try {
                activityMessenger.send(replyMsg);
            } catch (RemoteException e) {
                e.printStackTrace();
            }
        }

        //=========================================================================
        //
        //=========================================================================
        protected void handleMessage_MSG_ENABLE_FRAME_CALLBACK(Message msg) {
            Log.d(LOG_TAG, "::handleMessage_MSG_ENABLE_FRAME_CALLBACK");
            Bundle in_bundle = msg.getData();
            in_bundle.setClassLoader(this.getClass().getClassLoader());
            Messenger activityMessenger = msg.replyTo;
            Message replyMsg = Message.obtain();
            replyMsg.what = MSG_ENABLE_FRAME_CALLBACK;
            Bundle out_bundle = new Bundle();

            Surface surface = null;
            int timeout = 0;
            boolean result = false;
            long surfaceId = 0;
            Parcelable parcelable = in_bundle.getParcelable("SURFACE");
            if (parcelable instanceof Surface) {
                surface = (Surface) parcelable;
                Log.d(LOG_TAG, "::handleMessage_MSG_ENABLE_FRAME_CALLBACK - Surface:" + surface.toString());
                timeout = in_bundle.getInt("timeout");
                surfaceId = enableFrameCallback(surface);
                if(surfaceId != 0) {
                    out_bundle.putLong("surfaceId", surfaceId);
                    result = true;
                }
            }
            out_bundle.putBoolean("result", result);
            replyMsg.setData(out_bundle);
            try {
                activityMessenger.send(replyMsg);
            } catch (RemoteException e) {
                e.printStackTrace();
            }

            if (result == true) {
                if (frameCallbacks_ == null) {
                    frameCallbacks_ = new Hashtable<Long, Thread>();
                }
                //callback thread
                Thread frameCalbackThread = new FrameCallbackThread(activityMessenger, surfaceId, timeout);
                frameCallbacks_.put(surfaceId, frameCalbackThread);
                frameCalbackThread.start();
            }
        }

        //=========================================================================
        //
        //=========================================================================
        @Override
        public void handleMessage(Message msg) {

            if (!isUidPermitted(msg.sendingUid)) {
                Log.e(LOG_TAG, "The uid " + msg.sendingUid +  " is not authorized for seccamservice");
                return;
            }

            switch (msg.what) {
                case MSG_GET_TIMESTAMP: {
                    Log.d(LOG_TAG, "::handleMessage - MSG_GET_TIMESTAMP");
                    long elapsedMillis = SystemClock.elapsedRealtime() - service_.get().chronometer_.getBase();
                    int hours = (int) (elapsedMillis / 3600000);
                    int minutes = (int) (elapsedMillis - hours * 3600000) / 60000;
                    int seconds = (int) (elapsedMillis - hours * 3600000 - minutes * 60000) / 1000;
                    int millis = (int) (elapsedMillis - hours * 3600000 - minutes * 60000 - seconds * 1000);
                    Messenger activityMessenger = msg.replyTo;
                    Bundle out_bundle = new Bundle();
                    out_bundle.putString("timestamp", hours + ":" + minutes + ":" + seconds + ":" + millis);
                    Message replyMsg = Message.obtain(null, MSG_GET_TIMESTAMP);
                    replyMsg.setData(out_bundle);
                    try {
                        activityMessenger.send(replyMsg);
                    } catch (RemoteException e) {
                        e.printStackTrace();
                    }
                }
                break;
                case MSG_GET_CAPTURE_SURFACE:
                    handleMessage_MSG_GET_CAPTURE_SURFACE(msg);
                    break;
                case MSG_SET_PREVIEW_SURFACE:
                    handleMessage_MSG_SET_PREVIEW_SURFACE(msg);
                    break;
                case MSG_RELEASE_CAPTURE_SURFACE:
                    handleMessage_MSG_RELEASE_CAPTURE_SURFACE(msg);
                    break;
                case MSG_RELEASE_PREVIEW_SURFACE:
                    handleMessage_MSG_RELEASE_PREVIEW_SURFACE(msg);
                    break;
                case MSG_ENABLE_FRAME_CALLBACK:
                    handleMessage_MSG_ENABLE_FRAME_CALLBACK(msg);
                    break;
                default: {
                    // In case an unfamiliar message was recieved, we check if it is a vendor specific one
                    // and handle it accordingly
                    try {
                        SecCamServiceVendorHandler vendorHandler = new SecCamServiceVendorHandler();
                        if (!vendorHandler.handleVendorMessage(msg)) {
                            super.handleMessage(msg);
                        }
                    } catch (UnsatisfiedLinkError e) {
                        Log.e(LOG_TAG, "seccam_vendor lib is unavailable");
                        return;
                    }
                }
            }
        }
    }

    final Messenger mMessenger = new Messenger(new SecCamServiceHandler(this));

    //=========================================================================
    //
    //=========================================================================
    public boolean init() {
        Log.d(LOG_TAG, "::init");
        if (tzAppIfAvalable_ == false) {
            if (tzAppInfo_ == null) {
                tzAppInfo_ = new TZappInfo();
            }
            tzAppInfo_.verMaj_ = 0;
            tzAppInfo_.verMin_ = 0;
            tzAppInfo_.appName_ = "N/A";
            if (JNI_OK != doStartTzApp(TZ_APP_NAME , tzAppInfo_)) {
                Log.d(LOG_TAG, "::init - faild to start TZ app " + tzAppInfo_);
                tzAppIfAvalable_ = false;
            }
            else {
                tzAppInfo_.appName_ = TZ_APP_NAME;
                Log.d(LOG_TAG, "::init - TZ App loaded = " + tzAppInfo_.appName_ +
                        "(" + tzAppInfo_.verMaj_ + "." + tzAppInfo_.verMin_ + ")");
                tzAppIfAvalable_ = true;
            }
        }
        return tzAppIfAvalable_;
    }

    //=========================================================================
    //
    //=========================================================================
    @Override
    public void onCreate() {
        super.onCreate();
        Log.d(LOG_TAG, "::onCreate");
        chronometer_ = new Chronometer(this);
        chronometer_.setBase(SystemClock.elapsedRealtime());
        chronometer_.start();

        // Save the service context, required for future use by isUidPermitted
        serviceContext = getApplicationContext();
    }

    //=========================================================================
    //
    //=========================================================================
    @Override
    public IBinder onBind(Intent intent) {
        Log.d(LOG_TAG, "::onBind");
        init();
        // In case the TA is not loaded, binding to the service is disabled
        if (tzAppIfAvalable_) {
            return mMessenger.getBinder();
        }
        else {
            Log.e(LOG_TAG, "::onBind - binding to service failed, TZ app is not available");
            return null;
        }
    }

    //=========================================================================
    //
    //=========================================================================
    @Override
    public void onRebind(Intent intent) {
        Log.d(LOG_TAG, "::onRebind");
        super.onRebind(intent);
        init();
    }

    //=========================================================================
    //
    //=========================================================================
    @Override
    public boolean onUnbind(Intent intent) {
        Log.d(LOG_TAG, "::onUnbind");
        if (tzAppIfAvalable_) {
            doShutdownTzApp();
            tzAppIfAvalable_ = false;
        }
        if (frameCallbacks_ != null) {
            for (Thread val: frameCallbacks_.values()) {
                try {
                    if(val.isAlive()) {
                        val.stop();
                    }
                } catch (Exception e) {
                    Log.d(LOG_TAG, "::onUnbind - Callback thread is not active");
                }
            }
            frameCallbacks_ = null;
        }
        return true;
    }

    //=========================================================================
    //
    //=========================================================================
    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.d(LOG_TAG, "::onDestroy");
        if (tzAppIfAvalable_) {
            doShutdownTzApp();
            tzAppIfAvalable_ = false;
        }
        chronometer_.stop();

        serviceContext = null;
    }

    //=========================================================================
    // isUidPermitted is intendent for service access control based on package
    // name. It should be updated accordingly, to allow supported applications.
    //=========================================================================
    static private boolean isUidPermitted(int sendingUid) {

        // Set the packageNames that are allowed to use the service
        ArrayList<String> allowedPackages = new ArrayList<String>();
        allowedPackages.add("com.qualcomm.qti.seccamsample");
        allowedPackages.add("com.qualcomm.qti.seccamdemoapp");

        try {
            // Get the packageNames installed under the app uid, which is attempting to use the service
            String[] packageNames = serviceContext.getPackageManager().getPackagesForUid(sendingUid);

            // Check if one of the packageNames installed under the given uid is permitted
            for (int i=0; i<packageNames.length; i++) {
                for (int j=0; j<allowedPackages.size(); j++) {
                    if (packageNames[i].equals(allowedPackages.get(j))) {
                        return true;
                    }
                }
            }
        } catch (Exception e) {
             e.printStackTrace();
        }

        return false;
    }

}
