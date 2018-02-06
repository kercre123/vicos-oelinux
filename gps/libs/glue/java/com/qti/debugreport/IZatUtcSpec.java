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
 * class IZatUtcSpec
 * IZatUtcSpec class containing the UTC Time broken into whole seconds
 * and nano seconds.
 */
public class IZatUtcSpec implements Parcelable {
    private static String TAG = "IZatUtcSpec";
    private static final boolean VERBOSE = Log.isLoggable(TAG, Log.VERBOSE);

    private long mWholeSeconds;
    private long mNanoSeconds;

    public IZatUtcSpec(long seconds, long nanoseconds) {
        mWholeSeconds = seconds;
        mNanoSeconds = nanoseconds;
    }

    public IZatUtcSpec(Parcel source) {
        mWholeSeconds = source.readLong();
        mNanoSeconds = source.readLong();
    }

    /**
     * Get the UTC time in whole seconds
     *
     * @return Returns a UTC time in whole seconds
    */
    public long getSeconds() {
            return mWholeSeconds;
    }

    /**
    * Get the left over fraction of UTC time in nano seconds
     * @return Returns the left over fraction of UTC Time in nanoseconds.
    */
    public long getNanoSeconds() {
        return mNanoSeconds;
    }


    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeLong(mWholeSeconds);
        dest.writeLong(mNanoSeconds);
    }

    public static final Parcelable.Creator<IZatUtcSpec> CREATOR =
            new Parcelable.Creator<IZatUtcSpec>() {
        @Override
        public IZatUtcSpec createFromParcel(Parcel source) {
             return new IZatUtcSpec(source);
        }
        @Override
        public IZatUtcSpec[] newArray(int size) {
            return new IZatUtcSpec[size];
        }
    };
};
