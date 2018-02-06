// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All Rights Reserved.
// Confidential and Proprietary - Qualcomm Technologies, Inc.
package com.qualcomm.qti.seccamapi;

import android.hardware.camera2.CameraDevice;
import android.util.Log;
import android.view.Surface;

import com.qualcomm.qti.seccamapi.SecureSurface;

public class SecureCamera2Surface extends SecureSurface {

    private CameraDevice camera2_ = null;

    //=========================================================================
    //
    //=========================================================================
    public SecureCamera2Surface(CameraDevice camera, int width, int height, int format, int numOfBuffers) {
        super(-1, width, height, format, numOfBuffers);
        camera2_ = camera;
    }

}
