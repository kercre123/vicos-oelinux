/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

package com.qti.debugreport;

import android.os.Parcel;
import android.os.Parcelable;
import android.os.Bundle;

import java.lang.String;

/**
 * class IZatErrorRecoveryReport
 * IZatErrorRecoveryReport class contains the receiver error recovery state
 */
public class IZatErrorRecoveryReport implements Parcelable {
    private IZatUtcSpec mUtcTimeLastUpdated, mUtcTimeLastReported;

    public IZatErrorRecoveryReport(IZatUtcSpec utcTimeLastUpdated,
        IZatUtcSpec utcTimeLastReported) {
        mUtcTimeLastUpdated = utcTimeLastUpdated;
        mUtcTimeLastReported = utcTimeLastReported;
    }

    public IZatErrorRecoveryReport(Parcel source) {
        mUtcTimeLastUpdated = source.readParcelable(IZatUtcSpec.class.getClassLoader());
        mUtcTimeLastReported = source.readParcelable(IZatUtcSpec.class.getClassLoader());
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
    }

    public static final Parcelable.Creator<IZatErrorRecoveryReport> CREATOR =
            new Parcelable.Creator<IZatErrorRecoveryReport>() {
        @Override
        public IZatErrorRecoveryReport createFromParcel(Parcel source) {
             return new IZatErrorRecoveryReport(source);
        }
        @Override
        public IZatErrorRecoveryReport[] newArray(int size) {
            return new IZatErrorRecoveryReport[size];
        }
    };
};
