/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

package com.qti.debugreport;

import android.os.Parcel;
import android.os.Parcelable;
import android.os.Bundle;
import android.util.Log;

/**
 * class IZatPDRDebugReport
 * IZatPDRDebugReport class contains the PDR (Pedestrian dead reckoning)
 * information
 */
public class IZatPDRDebugReport implements Parcelable {
    private static String TAG = "IZatPDR";
    private static final boolean VERBOSE = Log.isLoggable(TAG, Log.VERBOSE);

    private static final int HEADING_FILTER_ENGAGED = 1;
    private static final int INS_FILTER_ENGAGED = 2;
    private static final int PDR_ENGAGED =  4;
    private static final int PDR_MAG_CALIBRATED = 8;

    private int mPDRInfoMask;
    private IZatUtcSpec mUtcTimeLastUpdated, mUtcTimeLastReported;

    public IZatPDRDebugReport(IZatUtcSpec utcTimeLastUpdated,
        IZatUtcSpec utcTimeLastReported, int pdrInfoMask) {
        mUtcTimeLastUpdated = utcTimeLastUpdated;
        mUtcTimeLastReported = utcTimeLastReported;
        mPDRInfoMask = pdrInfoMask;
    }

    public IZatPDRDebugReport(Parcel source) {
        mUtcTimeLastUpdated = source.readParcelable(IZatUtcSpec.class.getClassLoader());
        mUtcTimeLastReported = source.readParcelable(IZatUtcSpec.class.getClassLoader());

        mPDRInfoMask = source.readInt();
    }

    /**
     * Get if PDR was engaged.
     *
     * @return  A Boolean type to indicate if PDR was engaged or not.
     */
    public boolean isPDREngaged() {
        return ((mPDRInfoMask & PDR_ENGAGED) != 0);
    }

    /**
     * Get if PDR magnetometer was calibrated.
     *
     * @return  A Boolean type to indicate if PDR magnetometer was
     * calibrated or not.
     */
    public boolean isPDRMagCalibrated() {
        return ((mPDRInfoMask & PDR_MAG_CALIBRATED) != 0);
    }

    /**
     * Get if heading filter was engaged.
     *
     * @return  A Boolean type to indicate if heading filter was engaged or not.
     */
    public boolean isHDGFilterEngaged() {
        return ((mPDRInfoMask & HEADING_FILTER_ENGAGED) != 0);
    }

    /**
     * Get if INS (intertial navigation system) was engaged.
     *
     * @return  A Boolean type to indicate if INS was engaged or not.
     */
    public boolean isINSFilterEngaged() {
        return ((mPDRInfoMask & INS_FILTER_ENGAGED) != 0);
    }


    /**
     * Get the UTC time of when the data was last updated / changed.
     *
     * @return Returns a UTC time as {@link IZatUtcSpec}
    */
    public IZatUtcSpec getUTCTimestamp() {
        return mUtcTimeLastUpdated;
    }

    /**
     * Get the UTC time of when the data was last reported.
     *
     * @return Returns a UTC time as {@link IZatUtcSpec}
    */
    public IZatUtcSpec getLastReportedUTCTime() {
        return mUtcTimeLastReported;
    }



    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeParcelable(mUtcTimeLastUpdated, 0);
        dest.writeParcelable(mUtcTimeLastReported, 0);

        dest.writeInt(mPDRInfoMask);
    }

    public static final Parcelable.Creator<IZatPDRDebugReport> CREATOR =
            new Parcelable.Creator<IZatPDRDebugReport>() {
        @Override
        public IZatPDRDebugReport createFromParcel(Parcel source) {
             return new IZatPDRDebugReport(source);
        }
        @Override
        public IZatPDRDebugReport[] newArray(int size) {
            return new IZatPDRDebugReport[size];
        }
    };
};
