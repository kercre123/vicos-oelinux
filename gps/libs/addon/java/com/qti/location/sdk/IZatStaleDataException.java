/* ======================================================================
*  Copyright (c) 2017 Qualcomm Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Qualcomm Technologies, Inc.
*  ====================================================================*/

package com.qti.location.sdk;

/**
 * An exception indicating access of stale/invalid data.
 * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
 * <p>All Rights Reserved.</p>
 * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
 */
public class IZatStaleDataException extends RuntimeException{
    public IZatStaleDataException() {
    }

    public IZatStaleDataException(String error) {
        super(error);
    }

    public IZatStaleDataException(String error, Throwable cause) {
        super(error, cause);
    }
}
