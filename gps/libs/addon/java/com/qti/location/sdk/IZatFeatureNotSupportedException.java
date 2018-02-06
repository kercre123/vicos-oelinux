/* ======================================================================
*  Copyright (c) 2017 Qualcomm Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Qualcomm Technologies, Inc.
*  ====================================================================*/

package com.qti.location.sdk;

/**
 * An exception indicating feature not supported
 * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
 * <p>All Rights Reserved.</p>
 * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
 */
public class IZatFeatureNotSupportedException extends RuntimeException{
    public IZatFeatureNotSupportedException() {
    }

    public IZatFeatureNotSupportedException(String error) {
        super(error);
    }

    public IZatFeatureNotSupportedException(String error, Throwable cause) {
        super(error, cause);
    }
}
