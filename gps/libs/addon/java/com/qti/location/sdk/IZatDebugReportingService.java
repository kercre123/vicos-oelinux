/* ======================================================================
*  Copyright (c) 2017 Qualcomm Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Qualcomm Technologies, Inc.
*  ====================================================================*/

package com.qti.location.sdk;

import android.content.Context;
import android.os.Bundle;

import com.qti.debugreport.IZatDebugConstants;
import java.util.ArrayList;

public interface IZatDebugReportingService {
    /**
     * Register to start receiving IZat debug reports
     * <p>
     * This allows applications to start receiving IZat debug
     * reports at 1Hz. If there is already a callback registered,
     * the new one one will replace the existing one </p>
     *
     * @param  debugReportCb the callback to receive the debug report.
     *         This parameter can not be null, otherwise a
     *         {@link IZatIllegalArgumentException} will be thrown.
     * @throws IZatIllegalArgumentException
     */

    void registerForDebugReports(IZatDebugReportCallback debugReportCb)
                                      throws IZatIllegalArgumentException;

        /**
     * De-register stop receiving IZat debugs reports.
     * <p>
     * This allows applications to stop getting debug reports.</p>
     *
     * @param  debugReportCb the callback to stop receiving debug reports.
     *         This parameter can not be null, otherwise a
     *         {@link IZatIllegalArgumentException} will be thrown.
     * @throws IZatIllegalArgumentException
     */

    void deregisterForDebugReports(IZatDebugReportCallback debugReportCb)
                                        throws IZatIllegalArgumentException;

    /**
     * Synchronous call to receive an instant debug report.
     * <p> Application can make this call to receive an instant
     * debug report at any time. </p>
     *
     * @return A <a href="https://developer.android.com/reference/android/os/Bundle.html">Bundle</a> type.
     * The Bundle object can contain the following fields
     *<ol>
     * {
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_EXTERNAL_POSITION_INJECTION_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatLocationReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_BEST_POSITION_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatLocationReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_EPH_STATUS_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatEphmerisDebugReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_XTRA_STATUS_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatXTRADebugReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_SV_HEALTH_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatSVHealthDebugReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_PDR_INFO_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatPDRDebugReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_FIX_STATUS_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatFixStatusDebugReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_GPS_TIME_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatGpsTimeDebugReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_RF_STATE_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatRfStateDebugReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_XO_STATE_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatXoStateDebugReport}>) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_LAST_ERROR_RECOVERIES_KEY},
     *          {@link <a href="https://developer.android.com/reference/java/util/ArrayList.html">ArrayList</a>}
     *          <{@link com.qti.debugreport.IZatUtcSpec}>) </li>
     * }
     * </ol>
     */

    Bundle getDebugReport();

    interface IZatDebugReportCallback {
    /**
     * Debug Report callback
     * <p>
     * This API will be called by the underlying service back
     * to applications when debug report is available.
     * Applications should implement this interface.</p>
     *
     * @param debugReport <a href="https://developer.android.com/reference/android/os/Bundle.html">Bundle</a> type.
     * The Bundle object can contain the following fields
     *<ol>
     * {
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_EXTERNAL_POSITION_INJECTION_KEY},
     *          {@link com.qti.debugreport.IZatLocationReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_BEST_POSITION_KEY},
     *          {@link com.qti.debugreport.IZatLocationReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_EPH_STATUS_KEY},
     *          {@link com.qti.debugreport.IZatEphmerisDebugReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_XTRA_STATUS_KEY},
     *          {@link com.qti.debugreport.IZatXTRADebugReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_SV_HEALTH_KEY},
     *          {@link com.qti.debugreport.IZatSVHealthDebugReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_PDR_INFO_KEY},
     *          {@link com.qti.debugreport.IZatPDRDebugReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_FIX_STATUS_KEY},
     *          {@link com.qti.debugreport.IZatFixStatusDebugReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_GPS_TIME_KEY},
     *          {@link com.qti.debugreport.IZatGpsTimeDebugReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_RF_STATE_KEY},
     *          {@link com.qti.debugreport.IZatRfStateDebugReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_XO_STATE_KEY},
     *          {@link com.qti.debugreport.IZatXoStateDebugReport}) </li>
     *    <li> ({@link com.qti.debugreport.IZatDebugConstants#IZAT_DEBUG_LAST_ERROR_RECOVERIES_KEY},
     *          {@link com.qti.debugreport.IZatUtcSpec}) </li>
     * }
     * </ol>
     */

    void onDebugReportAvailable(Bundle debugReport);
    }
}
