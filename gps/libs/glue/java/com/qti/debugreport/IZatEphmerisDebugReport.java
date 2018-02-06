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
 * class IZatEphmerisDebugReport
 * IZatEphmerisDebugReport class contains the Ephmeris data validity for
    each GNSS constellation.
 */
public class IZatEphmerisDebugReport implements Parcelable {
    private static String TAG = "IZatEphmeris";
    private static final boolean VERBOSE = Log.isLoggable(TAG, Log.VERBOSE);

    private int  mGpsEphemrisDataValidity;
    private int mGlonassEphemrisDataValidity;
    private long mBdsEphemrisDataValidity;
    private long mGalEphemrisDataValidity;
    private byte mQzssEphemrisDataValidity;
    private IZatUtcSpec mUtcTimeLastUpdated, mUtcTimeLastReported;

    public IZatEphmerisDebugReport(IZatUtcSpec utcTimeLastUpdated,
        IZatUtcSpec utcTimeLastReported, int gpsEphDataValidity, int glonassEphDataValidity,
        long bdsEphDataValidity, long galEphDataValidity, byte qzssEphDataValidity) {

        mUtcTimeLastUpdated = utcTimeLastUpdated;
        mUtcTimeLastReported = utcTimeLastReported;

        mGpsEphemrisDataValidity = gpsEphDataValidity;
        mGlonassEphemrisDataValidity = glonassEphDataValidity;
        mBdsEphemrisDataValidity = bdsEphDataValidity;
        mGalEphemrisDataValidity = galEphDataValidity;
        mQzssEphemrisDataValidity = qzssEphDataValidity;
    }

    public IZatEphmerisDebugReport(Parcel source) {
        mUtcTimeLastUpdated = source.readParcelable(IZatUtcSpec.class.getClassLoader());
        mUtcTimeLastReported = source.readParcelable(IZatUtcSpec.class.getClassLoader());

        mGpsEphemrisDataValidity = source.readInt();
        mGlonassEphemrisDataValidity = source.readInt();
        mBdsEphemrisDataValidity = source.readLong();
        mGalEphemrisDataValidity = source.readLong();
        mQzssEphemrisDataValidity = source.readByte();
    }

        /**
     * Get the ephmeris data validity for GPS satellites
     *
     * @return Bitmask of Ephmeris data as int. Each bit represents the
     * ephmeris state of a satellite, 0 indicates the ephmeris is not present
     * and 1 indicates ephmeris is present for that satellite.
     */
    public int getEphmerisForGPS() {
        return mGpsEphemrisDataValidity;
    }

    /**
     * Get the ephmeris data validity for Glonass satellites
     *
     * @return Bitmask of Ephmeris data as int. Each bit represents the
     * ephmeris state of a satellite, 0 indicates the ephmeris is not present
     * and 1 indicates ephmeris is present for that satellite.
     */
    public int getEphmerisForGlonass() {
        return mGlonassEphemrisDataValidity;
    }

    /**
     * Get the ephmeris data validity for BDS satellites
     *
     * @return Bitmask of Ephmeris data as long. Each bit represents the
     * ephmeris state of a satellite, 0 indicates the ephmeris is not present
     * and 1 indicates ephmeris is present for that satellite.
     */
    public long getEphmerisForBDS() {
        return mBdsEphemrisDataValidity;
    }

    /**
     * Get the ephmeris data validity for GAL satellites
     *
     * @return Bitmask of Ephmeris data as long. Each bit represents the
     * ephmeris state of a satellite, 0 indicates the ephmeris is not present
     * and 1 indicates ephmeris is present for that satellite.
     */
    public long getEphmerisForGal() {
        return mGalEphemrisDataValidity;
    }

    /**
     * Get the ephmeris data validity for Qzss satellites
     *
     * @return Bitmask of Ephmeris data as byte. Each bit represents the
     * ephmeris state of a satellite, 0 indicates the ephmeris is not present
     * and 1 indicates ephmeris is present for that satellite.
     */
    public byte getEphmerisForQzss() {
        return mQzssEphemrisDataValidity;
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

        dest.writeInt(mGpsEphemrisDataValidity);
        dest.writeInt(mGlonassEphemrisDataValidity);
        dest.writeLong(mBdsEphemrisDataValidity);
        dest.writeLong(mGalEphemrisDataValidity);
        dest.writeByte(mQzssEphemrisDataValidity);
    }

    public static final Parcelable.Creator<IZatEphmerisDebugReport> CREATOR =
            new Parcelable.Creator<IZatEphmerisDebugReport>() {
        @Override
        public IZatEphmerisDebugReport createFromParcel(Parcel source) {
             return new IZatEphmerisDebugReport(source);
        }
        @Override
        public IZatEphmerisDebugReport[] newArray(int size) {
            return new IZatEphmerisDebugReport[size];
        }
    };
};
