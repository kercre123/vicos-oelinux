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
 * class IZatRfStateDebugReport
 * IZatRfStateDebugReport class contains the RF state and parameters
 */
public class IZatRfStateDebugReport implements Parcelable {
    private static String TAG = "IZatRfStateReport";
    private static final boolean VERBOSE = Log.isLoggable(TAG, Log.VERBOSE);

    private IZatUtcSpec mUtcTimeLastUpdated, mUtcTimeLastReported;
    private int mPGAGain;
    private long mGPSBPAmpI;
    private long mGPSBPAmpQ;
    private long mADCAmplitudeI;
    private long mADCAmplitudeQ;
    private long mJammerMetricGPS;
    private long mJammerMetricGlonass;
    private long mJammerMetricBds;
    private long mJammerMetricGal;
    private long mErrorRecovery;

    public IZatRfStateDebugReport(IZatUtcSpec utcTimeLastUpdated,
        IZatUtcSpec utcTimeLastReported,
        int pgaGain, long bpAmplI, long bpAmplQ, long adcAmplI, long adcAmplQ,
        long jammermetricGps, long jammermetricGlonass,
        long jammermetricBds, long jammermetricGal) {

        mUtcTimeLastUpdated = utcTimeLastUpdated;
        mUtcTimeLastReported = utcTimeLastReported;

        mPGAGain = pgaGain;
        mGPSBPAmpI = bpAmplI;
        mGPSBPAmpQ = bpAmplQ;
        mADCAmplitudeI = adcAmplI;
        mADCAmplitudeQ = adcAmplQ;

        mJammerMetricGPS = jammermetricGps;
        mJammerMetricGlonass = jammermetricGlonass;
        mJammerMetricBds = jammermetricBds;
        mJammerMetricGal = jammermetricGal;
    }

    public IZatRfStateDebugReport(Parcel source) {
        mUtcTimeLastUpdated = source.readParcelable(IZatUtcSpec.class.getClassLoader());
        mUtcTimeLastReported = source.readParcelable(IZatUtcSpec.class.getClassLoader());

        mPGAGain = source.readInt();
        mGPSBPAmpI = source.readLong();
        mGPSBPAmpQ = source.readLong();
        mADCAmplitudeI = source.readLong();
        mADCAmplitudeQ = source.readLong();

        mJammerMetricGPS = source.readLong();
        mJammerMetricGlonass = source.readLong();
        mJammerMetricBds = source.readLong();
        mJammerMetricGal = source.readLong();
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


    /**
    * Get GNSS RF Gain
    */
    public int getPGAGain() {
        return mPGAGain;
    }


    /**
    * Get GPS Baseband Processor Amplitude I
    */
    public long getGPSBPAmpI() {
        return  mGPSBPAmpI;
    }

    /**
    * Get GPS Baseband Processor Amplitude Q
    */
    public long getGPSBPAmpQ() {
        return mGPSBPAmpQ;
    }

    /**
    * Get ADC Amplitude I
    */
    public long getADCAmplitudeI() {
        return mADCAmplitudeI;
    }

    /**
    * Get ADC Amplitude Q
    */
    public long getADCAmplitudeQ() {
        return mADCAmplitudeQ;
    }

    /**
    * Get Jammer metric for GPS
    */
    public long getJammerMetricGPS() {
        return mJammerMetricGPS;
    }

    /**
    * Get Jammer metric for Glonass
    */
    public long getJammerMetricGlonass() {
        return mJammerMetricGlonass;
    }

    /**
    * Get Jammer metric for Beidou
    */
    public long getJammerMetricBds() {
        return mJammerMetricBds;
    }

    /**
    * Get Jammer metric for Gal
    */
    public long getJammerMetricGal() {
        return mJammerMetricGal;
    }

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeParcelable(mUtcTimeLastUpdated, 0);
        dest.writeParcelable(mUtcTimeLastReported, 0);

        dest.writeInt(mPGAGain);
        dest.writeLong(mGPSBPAmpI);
        dest.writeLong(mGPSBPAmpQ);
        dest.writeLong(mADCAmplitudeI);
        dest.writeLong(mADCAmplitudeQ);

        dest.writeLong(mJammerMetricGPS);
        dest.writeLong(mJammerMetricGlonass);
        dest.writeLong(mJammerMetricBds);
        dest.writeLong( mJammerMetricGal);
    }

    public static final Parcelable.Creator<IZatRfStateDebugReport> CREATOR =
            new Parcelable.Creator<IZatRfStateDebugReport>() {
        @Override
        public IZatRfStateDebugReport createFromParcel(Parcel source) {
             return new IZatRfStateDebugReport(source);
        }
        @Override
        public IZatRfStateDebugReport[] newArray(int size) {
            return new IZatRfStateDebugReport[size];
        }
    };
};
