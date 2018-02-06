/* ======================================================================
*  Copyright (c) 2017 Qualcomm Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Qualcomm Technologies, Inc.
*  ====================================================================*/

package com.qti.location.sdk;

import java.util.List;

import android.content.Context;
import android.location.Location;

/**
 * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
 * <p>All Rights Reserved.</p>
 * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
 * <br/>
 * <p><b>IZatWiFiDBReceiver</b> interface - the api for
 * injecting WiFi AP Location database to IZat Location framework. </p>
 * @version 1.0.0
 */

public abstract class IZatWiFiDBReceiver {
    protected final IZatWiFiDBReceiverResponseListener mResponseListener;

    /**
     * IZatWiFiDBReceiver
     * <p>
     * Constructor - IZatWiFiDBReceiver </p>
     *
     * @param  listener the listener to receive WiFi DB Receiver
     *         responses. This parameter can not be null, otherwise
     *         a {@link IZatIllegalArgumentException} will be
     *         thrown.
     * @throws IZatIllegalArgumentException
     */
    protected IZatWiFiDBReceiver(IZatWiFiDBReceiverResponseListener listener)
                                throws IZatIllegalArgumentException {
        if(null == listener) {
            throw new IZatIllegalArgumentException("Unable to obtain IZatWiFiDBReceiver instance");
        }
        mResponseListener = listener;
    }

    /**
     * Request list of access points.
     * <p>
     * This allows WiFi database provider to request list of APs
     * which needs location information.
     * </p>
     *
     * @param  expire_in_days the number of days in future in which
     *                        the associated location of an AP if
     *                        available, will expire, to be fetched
     *                        by this request. Optional Parameter.
     *                        If 0 is provided only APs which has an
     *                        already expired location or no
     *                        location associated with, will be
     *                        fetched.
     */
    public abstract void requestAPList(int expire_in_days);

    /**
     * Request WiFi DB update.
     * <p>
     * This allows WiFi database provider to insert a list of APs
     * with location location information.
     * </p>
     *
     * @param location_data Location information of access points.
     *                      If not available null/empty list can be
     *                      provided.
     * @param special_info Special information on access point. If
     *                     not available null/empty list can be
     *                     provided.
     * @param days_valid days in future, for which location_data and
     *                   special_info will be valid. Optional
     *                   parameter. Default to 15 days if 0 is
     *                   provided.
     */
    public abstract void pushWiFiDB(List<IZatAPLocationData> location_data,
                                    List<IZatAPSpecialInfo> special_info,
                                    int days_valid);

    /**
     * Interface class IZatWiFiDBReceiverResponseListener.
     * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     * <br/>
     * <p><b>IZatWiFiDBReceiverResponseListener</b> is the interface
     * to receive responses from WiFi dabase receiver in IZat
     * location framework.</p>
     */
    public interface IZatWiFiDBReceiverResponseListener {
        /**
         * Response for AP List request.
         * <p>
         * This API will be called by the underlying service back
         * to applications when list of APs are available.
         * Applications should implement this interface.</p>
         *
         * @param ap_list the list of APs
         */
        void onAPListAvailable(List<IZatAPInfo> ap_list);

        /**
         * Response for AP location injection request.
         * <p>
         * This API will be called by the underlying service back
         * to applications when AP Location injection completes.
         * Applications should implement this interface.</p>
         *
         * @param is_success the injection of AP locations success or
         *                   failure.
         * @param error the error details if the AP location injection
         *              was a failure.
         */
        void onStatusUpdate(boolean is_success, String error);

        /**
         * Service request to WiFi DB Provider.
         * <p>
         * This API will be called by the underlying service back
         * to applications when they need service. Applications should
         * implement this interface.</p>
         *
         */
        void onServiceRequest();
    }

    /**
     * class IZatAPInfo.
     * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     */
    public static class IZatAPInfo {
        String mMacAddress;
        IZatAPInfoExtra mExtra;

        /**
         * IZatAPInfo Constructor
         * <p>Constructor</p>
         *
         * @param mac Mac address if WiFi access point in the form of a
         *            six-byte MAC address: {@code XX:XX:XX:XX:XX:XX}.
         * @param extra Extra context information for this AP
         */
        public IZatAPInfo(String mac, IZatAPInfoExtra extra) {
            mMacAddress = mac;
            mExtra = extra;
        }

        /**
         * Get Macaddress of this AP
         *  <p>
         *  Return mac address of this access point. </p>
         *
         * @return String in the form of a six-byte MAC address: {@code
         *         XX:XX:XX:XX:XX:XX}
         */
        public String getMacAddress() {
            return mMacAddress;
        }

        /**
         * Get extra information on this AP.
         * <p>
         *  Get extra context information on this AP.</p>
         *
         * @return IZatAPInfoExtra
         */
        public IZatAPInfoExtra getExtra() {
            return mExtra;
        }

        /**
         * Check whether Extra information about this AP is available
         * <p>
         *  Check whether Extra information about this AP is available
         *  </p>
         *
         * @return boolean
         */
        public boolean isExtraAvailable() {
            return (null == mExtra ? false : mExtra.isAvailable());
        }
    }

    /**
     * class IZatAPInfoExtra.
     * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     */
    public static class IZatAPInfoExtra {
        IZatCellInfo mCellInfo;
        IZatAPSSIDInfo mSSID;

        /**
         *  Constructor - IZatAPInfoExtra
         *  <p>
         *   Constructor</p>
         *
         * @param cellInfo CellInfo in which this AP is observed
         * @param ssid SSID of this access point.
         */
        public IZatAPInfoExtra(IZatCellInfo cellInfo, IZatAPSSIDInfo ssid) {
            mCellInfo = cellInfo;
            mSSID = ssid;
        }

        /**
         * Get Cell Information
         * <p>
         *  Get cellInfo in which this AP is observed. This information
         *  is not supported now</p>
         *
         * @return IZatCellInfo
         */
        public IZatCellInfo getCellInfo() {
            return mCellInfo;
        }

        /**
         * Get SSID
         * <p>
         *  Returns the service set identifier (SSID) of this AP.</p>
         *
         * @return IZatAPSSIDInfo
         */
        public IZatAPSSIDInfo getSSID() {
            return mSSID;
        }

        /**
         * Any Extra Information available
         * <p>
         *  Returns if Extra context information for this AP is vailable
         *
         * @return boolean
         */
        public boolean isAvailable() {
            return ((null != mSSID) || (null != mCellInfo));
        }
    }

    /**
     * class IZatCellInfo.
     * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     */
    public static class IZatCellInfo {
        public final int mRegionID1;
        public final int mRegionID2;
        public final int mRegionID3;
        public final int mRegionID4;

        /**
         * enum IZatCellTypes
         * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
         * <p>All Rights Reserved.</p>
         * <p>Confidential and Proprietary - Qualcomm Technologies,
         * Inc</p>
         */
        public enum IZatCellTypes {
            GSM, CDMA, WCDMA, LTE
        }
        public final IZatCellTypes mType;

        /**
         * Constructor - IZatCellInfo
         * <p>
         *  Constructor</p>
         *
         * @param regionID1 Mobile Country Code(MCC), For CDMA Set to 0
         * @param regionID2 Mobile Network Code(MNC), For CDMA set to
         *                  System ID(SID), For WCDMA set to 0 if not
         *                  vailable
         * @param regionID3 GSM: Local Area Code(LAC), WCDMA: Local Area
         *                  Code(LAC) set to 0 if not available, CDMA:
         *                  Network ID(NID), LTE: Tracking Area
         *                  Code(TAC) set to 0 if not available
         * @param regionID4 Cell ID(CID), For CDMA set to Base Station
         *                  ID(BSID)
         * @param type Cell type
         */
        public IZatCellInfo(int regionID1, int regionID2, int regionID3,
                            int regionID4, IZatCellTypes type) {
            mRegionID1 = regionID1;
            mRegionID2 = regionID2;
            mRegionID3 = regionID3;
            mRegionID4 = regionID4;
            mType = type;
        }
    }

    /**
     * class IZatAPSSIDInfo.
     * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     */
    public static class IZatAPSSIDInfo {

        public final byte[] mSSID;

        /**
         * Contructor - IZatAPSSIDInfo
         * <p>
         *  Constructor </p>
         *
         * @param ssid Available bytes in SSID of AP. This can be
         *             smaller than actual SSID size. How many bytes
         *             from SSID are available is indicated by
         *             validBytesCount
         * @param validBytesCount  How many bytes from actual SSID of AP
         *                         are available in ssid
         */
        public IZatAPSSIDInfo(byte[] ssid, short validBytesCount) {
            mSSID = new byte[validBytesCount];
        }
    }

    /**
     * class IZatAPLocationData.
     * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     */
    public static class IZatAPLocationData {

        String mMacAddress;
        float mLatitude;
        float mLongitude;
        float mMaxAntenaRange;
        float mHorizontalError;

        /**
         * enum IZatReliablityTypes
         * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
         * <p>All Rights Reserved.</p>
         * <p>Confidential and Proprietary - Qualcomm Technologies,
         * Inc</p>
         */
        public enum IZatReliablityTypes {
            VERY_LOW, LOW, MEDIUM, HIGH, VERY_HIGH
        }
        IZatReliablityTypes mReliability;

        public static final int IZAT_AP_LOC_WITH_LAT_LON         = 0x0;
        public static final int IZAT_AP_LOC_MAR_VALID            = 0x1;
        public static final int IZAT_AP_LOC_HORIZONTAL_ERR_VALID = 0x2;
        public static final int IZAT_AP_LOC_RELIABILITY_VALID    = 0x4;
        int mValidBits;

        /**
         * Constructor - IZatAPLocationData
         * <p>
         *  Constructor </p>
         *
         * @param mac in the form of a six-byte MAC address string:
         *         {@code XX:XX:XX:XX:XX:XX}
         * @param latitude Latitude of AP
         * @param longitude Longitude of AP
         */
        public IZatAPLocationData(String mac, float latitude, float longitude) {
            mMacAddress = mac;
            mLatitude = latitude;
            mLongitude = longitude;
            mValidBits = IZAT_AP_LOC_WITH_LAT_LON;
        }

        /**
         * Set Latitude
         * <p>
         * Set Latitude <p>
         *
         * @param latitude Latitude of AP
         */
        public void setLatitude(float latitude) {
            mLatitude = latitude;
        }

        /**
         * Set Longitude
         * <p>
         * Set Longitude <p>
         *
         * @param longitude Longitude of AP
         */
        public void setLongitude(float longitude) {
            mLongitude = longitude;
        }


        /**
         * Set Maximum Antena range of AP
         * <p>
         *  Set Maximum Antena range of AP.  </p>
         *
         * @param mar Maximum Antena Range of AP
         */
        public void setMaxAntenaRange(float mar) {
            mMaxAntenaRange = mar;
            mValidBits |= IZAT_AP_LOC_MAR_VALID;
        }

        /**
         * Set Horizontal Error
         * <p>
         *  Set Horizontal Error.  </p>
         *
         * @param he Horizontal Error
         */
        public void setHorizontalError(float he) {
            mHorizontalError = he;
            mValidBits |= IZAT_AP_LOC_HORIZONTAL_ERR_VALID;
        }

        /**
         * Set Reliability
         * <p>
         *  Set Reliability of the location information provided.
         *  VERY_LOW : when probability of position outlier 1 in one
         *  hundres or even more likely
         *  LOW : when probability of position outlier about 1 in a
         *  thousand
         *  MEDIUM : when probability of position outlier about 1 in a
         *  100 thousand
         *  HIGH : when probability of position outlier about 1 in a
         *  10 million
         *  VERY HIGH : when probability of position outlier about 1 in
         *  a thousand million
         *  until sufficient experience is obtained, the reliability
         *  input value should remain unset or set to LOW.
         *  </p>
         * @param reliability
         */
        public void setReliability(IZatReliablityTypes reliability) {
            mReliability = reliability;
            mValidBits |= IZAT_AP_LOC_RELIABILITY_VALID;
        }

        /**
         * Get Mac Address
         * <p>
         *  Get MAC in the form of a six-byte MAC address string:
         *  {@code XX:XX:XX:XX:XX:XX}
         * @return String
         */
        public String getMacAddress() {
            return mMacAddress;
        }

        /**
         * Get Latitude
         *
         * @return float
         */
        public float getLatitude() {
            return mLatitude;
        }

        /**
         * Get Longitude
         *
         * @return float
         */
        public float getLongitude() {
            return mLongitude;
        }

        /**
         * Get Maximum Antena Range
         *
         * @throws IZatStaleDataException
         * @return float
         */
        public float getMaxAntenaRange() throws IZatStaleDataException {
            if(0 == (IZAT_AP_LOC_MAR_VALID & mValidBits)) {
                throw new IZatStaleDataException("Maximum Antena Range information is not valid");
            }
            return mMaxAntenaRange;
        }

        /**
         * Get Horizontal Error
         *
         * @throws IZatStaleDataException
         * @return float
         */
        public float getHorizontalError() throws IZatStaleDataException {
            if(0 == (IZAT_AP_LOC_HORIZONTAL_ERR_VALID & mValidBits)) {
                throw new IZatStaleDataException("Horizontal error information is not valid");
            }
            return mHorizontalError;
        }

        /**
         * Get Reliability
         *
         * @throws IZatStaleDataException
         * @return IZatReliablityTypes
         */
        public IZatReliablityTypes getReliability() throws IZatStaleDataException {
            if(0 == (IZAT_AP_LOC_RELIABILITY_VALID & mValidBits)) {
                throw new IZatStaleDataException("Reliability information is not valid");
            }
            return mReliability;
        }

    }

    /**
     * class IZatAPSpecialInfo.
     * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     */
    public static class IZatAPSpecialInfo {

        public final String mMacAddress;

        /**
         * enum IZatAPSpecialInfoTypes
         * <p>Copyright (c) 2017 Qualcomm Technologies, Inc.</p>
         * <p>All Rights Reserved.</p>
         * <p>Confidential and Proprietary - Qualcomm Technologies,
         * Inc</p>
         */
        public enum IZatAPSpecialInfoTypes {
            NO_INFO_AVAILABLE, MOVING_AP
        }
        public final IZatAPSpecialInfoTypes mInfo;


        /**
         * Constructor - IZatAPSpecialInfo
         * <p>
         *  Constructor </p>
         *
         * @param mac in the form of a six-byte MAC address string:
         *         {@code XX:XX:XX:XX:XX:XX}
         * @param info Information on this AP
         * @return
         *
         */
        public IZatAPSpecialInfo(String mac, IZatAPSpecialInfoTypes info) {
            mMacAddress = mac;
            mInfo = info;
        }
    }

}
