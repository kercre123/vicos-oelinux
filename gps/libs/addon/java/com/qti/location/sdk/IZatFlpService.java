/* ======================================================================
*  Copyright (c) 2015 Qualcomm Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Qualcomm Technologies, Inc.
*  ====================================================================*/

package com.qti.location.sdk;

import android.location.Location;

/**
 * <p>Copyright (c) 2015 Qualcomm Technologies, Inc.</p>
 * <p>All Rights Reserved.</p>
 * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
 * <br/>
 * <p><b>IZatFlpService</b> interface - the api for interacting with the IZat FLP service. </p>
 * <pre>
 * <code>
 *    // Sample Code
 *
 *    import android.app.Activity;
 *    import android.os.Bundle;
 *    import android.view.View;
 *    import android.widget.Button;
 *    import com.qti.location.sdk.IZatManager;
 *    import com.qti.location.sdk.IZatFlpService;
 *
 *    public class FullscreenActivity extends Activity {
 *
 *       private IZatManager mIzatMgr = null;
 *       private IZatFlpService mFlpSevice = null;
 *       <b>// session handle which will be assigned after session starts.
 *       private IZatFlpService.IZatFlpSessionHandle mHandle = null;</b>
 *
 *       &#64Override
 *       protected void onCreate(Bundle savedInstanceState) {
 *
 *            ...
 *            // get the instance of IZatManager
 *            mIzatMgr = IZatManager.getInstance(getApplicationContext());
 *
 *            ...
 *            <b>// create a callback object used to receive the location objects.
 *            final LocationCallback locationCb = new LocationCallback();</b>
 *
 *            ...
 *            final Button connectButton = (Button)findViewById(R.id.connectButton);
 *            connectButton.setOnClickListener(new View.OnClickListener() {
 *                &#64Override
 *                public void onClick(View v) {
 *
 *                    // connect to IZatFlpService through IZatManager
 *                    if (mIzatMgr != null) {
 *                        mFlpSevice = mIzatMgr.connectFlpService();
 *                    }
 *                }
 *            });
 *
 *            ...
 *            final Button startButton = (Button)findViewById(R.id.startButton);
 *            startButton.setOnClickListener(new View.OnClickListener() {
 *                &#64Override
 *                public void onClick(View v) {
 *
 *                    <b>// start a flp position session with time interval and distance interval
 *                    if (mFlpSevice != null) {
 *
 *                        // create batching request object
 *                        IZatFlpService.IzatFlpRequest request =
 *                            IZatFlpService.IzatFlpRequest.getBackgroundFlprequest();
 *                        request.setTimeInterval(3000); // in milliseconds
 *                        request.setDistanceInterval(1); // in meters
 *
 *                        // start the session.
 *                        mHandle = mFlpSevice.startFlpSession(locationCb, request);
 *                    }</b>
 *                }
 *            });
 *        }
 *    }
 *
 *    public class LocationCallback implements IZatFlpService.IFlpLocationCallback {
 *        public void onLocationAvailable(Location[] locations) {
 *            ...
 *        }
 *    }
 *
 * </code>
 * </pre>
 * @version 1.0.0
 */

public interface IZatFlpService {
    /**
     * Start a Fused Location Positioning Session.
     * <p>
     * This allows applications to start a fused location positioning session,
     * which can be configured as location background session or location
     * foreground session. Location background session is able to batch
     * locations in the hardware when the device is asleep. Location
     * foreground session is navigating involving fused locations. </p>
     *
     * @param  callback the callback to receive locations.
     *         This parameter can not be null, otherwise a
     *         {@link IZatIllegalArgumentException} will be thrown.
     * @param  flpRequest the location request for the updates.
     *         This parameter can not be null, otherwise a
     *         {@link IZatIllegalArgumentException} will be thrown.
     * @throws IZatIllegalArgumentException
     * @return the handle binding with the session.
     */
    IZatFlpSessionHandle startFlpSession(IFlpLocationCallback callback,
                                         IzatFlpRequest flpRequest)
                                         throws IZatIllegalArgumentException;

    /**
     * Stop a Fused Location Positioning Session.
     * <p>
     * This allows applications to stop a running fused location positioning
     * session. </p>
     *
     * @param  handler the handle returned by {@link #startFlpSession}.
     *         This parameter can not be null, otherwise a
     *         {@link IZatIllegalArgumentException} will be thrown.
     * @throws IZatIllegalArgumentException
     */
    void stopFlpSession(IZatFlpSessionHandle handler)
                        throws IZatIllegalArgumentException;

    /**
     * Start to listen for the passive locations.
     * <p>
     * This allows applications to passively get locations when the user did not
     * start a FLP session, saving power. A passive client gets location updates
     * via callback only when there are updates for other active clients either
     * for a foreground or background session. </p>
     *
     * @param  callback the callback to receive locations.
     *         This parameter can not be null, otherwise a
     *         {@link IZatIllegalArgumentException} will be thrown.
     * @throws IZatIllegalArgumentException
     */
    void registerForPassiveLocations(IFlpLocationCallback callback)
                                     throws IZatIllegalArgumentException;

    /**
     * Stop listening for the passive locations.
     * <p>
     * This allows applications to stop listening for the passive locations.
     * If the callback is not registered, then this function
     * will just return. </p>
     *
     * @param  callback the callback to receive locations.
     *         This parameter can not be null, otherwise a
     *         {@link IZatIllegalArgumentException} will be thrown.
     * @throws IZatIllegalArgumentException
     */
    void deregisterForPassiveLocations(IFlpLocationCallback callback)
                                       throws IZatIllegalArgumentException;

    /**
     * Interface class IZatFlpSessionHandle.
     * <p>Copyright (c) 2015 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     * <br/>
     * <p><b>IZatFlpSessionHandle</b> is the interface used to manipulate the undergoing FLP session.</p>
     * <pre>
     * <code>
     *    // Sample Code
     *
     *    import android.app.Activity;
     *    import android.os.Bundle;
     *    import android.view.View;
     *    import android.widget.Button;
     *    import com.qti.location.sdk.IZatManager;
     *    import com.qti.location.sdk.IZatFlpService;
     *
     *    public class FullscreenActivity extends Activity {
     *
     *       ...
     *       <b>// session handle which will be assigned after session starts.
     *       private IZatFlpService.IZatFlpSessionHandle mHandle = null;</b>
     *
     *       &#64Override
     *       protected void onCreate(Bundle savedInstanceState) {
     *
     *            ...
     *            final Button startButton = (Button)findViewById(R.id.startButton);
     *            startButton.setOnClickListener(new View.OnClickListener() {
     *                &#64Override
     *                public void onClick(View v) {
     *                   ...
     *                   <b>// start the session.
     *                   mHandle = mFlpSevice.startFlpSession(locationCb, request);</b>
     *                }
     *            });
     *
     *            ...
     *            final Button queryButton = (Button)findViewById(R.id.queryButton);
     *            queryButton.setOnClickListener(new View.OnClickListener() {
     *                &#64Override
     *                public void onClick(View v) {
     *                    ...
     *                    <b>if (mHandle != null) {
     *                        // pull out all available locations, and the result will be
     *                        // returned through the session callback.
     *                        mHandle.pullLocations();
     *                    }</b>
     *                }
     *            });
     *
     *            ...
     *            final Button setToBackgroundButton =
     *                (Button)findViewById(R.id.setToBackgroundButton);
     *            setToBackgroundButton.setOnClickListener(new View.OnClickListener() {
     *                &#64Override
     *                public void onClick(View v) {
     *                    ...
     *                    <b>if (mHandle != null) {
     *                        // set the current Flp session to back ground
     *                        mHandle.setToBackground();
     *                    }</b>
     *                }
     *            });
     *        }
     *    }
     *
     *    public class LocationCallback implements IZatFlpService.IFlpLocationCallback {
     *        public void onLocationAvailable(Location[] locations) {
     *            ...
     *        }
     *    }
     *
     * </code>
     * </pre>
     */

    interface IZatFlpSessionHandle {
        /**
         * Request recent locations.
         * <p>
         * This allows applications to pull out all the available locations saved
         * in the device buffer. The locations returned will subsequently be
         * deleted form the buffer.</p>
         * <p> If the FLP session associated with this handle has been stopped,
         * the handle will no longer be valid, and there will be no fix returned
         * by calling this method of the invalidated handle. </p>
         *
         */
        void pullLocations();

        /**
         * Make the current FLP session to run in foreground.
         * <p>
         * This allows applications to make the current FLP session to run in
         * foreground, which is doing navigation only. In the case, applications
         * will get a callback on every fix. Compared to running in background,
         * this is more power consuming. </p>
         */
        void setToForeground();

        /**
         * Make the current FLP session to run in background.
         * <p>
         * This *may* allows applications to make the current FLP session to run in
         * background, which is doing location batching only. In the case,
         * applications will be get a callback when the batching buffer is full.
         * Compared to running in foreground, this is more power saving. </p>
         */
        void setToBackground();

        /**
         * Make current FLP session to batch fixes for a trip distance.
         * <p>
         * This allows applications to make the current FLP session to run in
         * background for the entire distance of trip specified.
         * Locations will be batched at 1 sec TBF by default, unless the time interval
         * is also specified. Applications will get a callback for batched locations
         * only after the trip distance is covered or batch buffer is full,
         * whichever happens first. On completion of trip, applications will receive
         * a {@link com.qti.location.sdk.IZatFlpService.IFlpStatusCallback#onBatchingStatusCb(
         * com.qti.location.sdk.IZatFlpService.IzatFlpStatus)}
         * callback with
         * {@link com.qti.location.sdk.IZatFlpService.IzatFlpStatus#OUTDOOR_TRIP_COMPLETED}
         * status. Compared to background mode, this gives better accuracy.
         * </p>
        */
        void setToTripMode()
                throws IZatFeatureNotSupportedException;

        /**
         * Start to listen for the batch status notifications
         * <p>
         * This allows applications to get status notifications on their batching requests. </p>
         *
         * @param  callback the callback to receive status notifications.
         *         This parameter can not be null, otherwise a
         *         {@link IZatIllegalArgumentException} will be thrown.
         * @throws IZatIllegalArgumentException
         */

        void registerForSessionStatus(IFlpStatusCallback callback)
                throws IZatIllegalArgumentException, IZatFeatureNotSupportedException;

        /**
         * Stop listening for the batching status notifications
         * <p>
         * This allows applications to stop listening for the status notifications.
         * If a callback was not registered, then this function
         * will just return. </p>
         */
        void unregisterForSessionStatus()
                throws IZatFeatureNotSupportedException;
    }

    /**
     * Interface class IFlpLocationCallback.
     * <p>Copyright (c) 2015 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     * <br/>
     * <p><b>IFlpLocationCallback</b> is the interface receiving locations.</p>
     * <pre>
     * <code>
     *    // Sample Code
     *
     *    import android.app.Activity;
     *    import android.os.Bundle;
     *    import android.view.View;
     *    import android.widget.Button;
     *    import android.util.Log;
     *    import com.qti.location.sdk.IZatManager;
     *    import com.qti.location.sdk.IZatFlpService;
     *
     *    public class FullscreenActivity extends Activity {
     *
     *        ...
     *        <b>// create a callback object used to receive the location objects.
     *        final LocationCallback locationCb = new LocationCallback();</b>
     *
     *        ...
     *        final Button startButton = (Button)findViewById(R.id.startButton);
     *        startButton.setOnClickListener(new View.OnClickListener() {
     *            &#64Override
     *            public void onClick(View v) {
     *               ...
     *               // start the session.
     *               <b>mHandle = mFlpSevice.startFlpSession(locationCb, request);</b>
     *            }
     *        });
     *    }
     *
     *    <b>public class LocationCallback implements IZatFlpService.IFlpLocationCallback {
     *        public void onLocationAvailable(Location[] locations) {
     *            for (int i =0; i < locations.length; i++) {
     *                if (location[i] != null) {
     *                   Log.d("IzatSDK",
     *                         "Position measurement: " + location[i].toString());
     *                }
     *            }
     *        }
     *    }</b>
     *
     * </code>
     * </pre>
     */
    interface IFlpLocationCallback {
        /**
         * Location callback.
         * <p>
         * This API will be called by the underlying service back
         * to applications when locations are available.
         * Applications should implement this interface.</p>
         *
         * @param  locations the array of locations available
         */
        void onLocationAvailable(Location[] locations);
    }

    /**
     * class IzatFlpStatus
     * IzatFlpStatus allows application to know status of ongoing
     * background FLP sessions. Application must register to receive
     * status using the
     * {@link com.qti.location.sdk.IZatFlpService.IZatFlpSessionHandle#registerForSessionStatus(
     * com.qti.location.sdk.IZatFlpService.IFlpStatusCallback)}
     */
    public enum IzatFlpStatus {
        /**
         * Application running a FLP session in
         * {@link com.qti.location.sdk.IZatFlpService.IzatFlpBgRequest.IzatFlpBgRequestMode#TRIP_MODE TRIP_MODE}
         * will receive OUTDOOR_TRIP_COMPLETED status once the trip distance is covered.
         */
        OUTDOOR_TRIP_COMPLETED(0),

        /**
         * Application will receive a POSITION_AVAILABLE status if the underlying
         * engine is able to generate any new position fixes after being stuck in a
         * {@link com.qti.location.sdk.IZatFlpService.IzatFlpStatus#POSITION_NOT_AVAILABLE}
         * state for sometime.
         */
        POSITION_AVAILABLE(1),

        /**
         * Application will receive a POSITION_NOT_AVAILABLE once if the underlying engine
         * is not able to generate any new position fixes. Once new positions are available
         * the underlying engine will send a
         * {@link com.qti.location.sdk.IZatFlpService.IzatFlpStatus#POSITION_AVAILABLE}
         * status. Applications may choose to stop the flp session at this time if power
         * is a concern, as underlying engine will continue its attempt to make a position fix.
        */
        POSITION_NOT_AVAILABLE(2);

        private final int value;
        private IzatFlpStatus(int value) {
            this.value = value;
        }
        public int getValue() {
            return value;
        }
    }

        /**
         * class IFlpStatusCallback
         */
    interface IFlpStatusCallback {

        /**
         * Engine status callback.
         * <p>
         * This API will be called by the underlying service back
         * to applications when the batching engine reports its status.</p>
         * </p>
         * @param status Batching status sent back from engine,
         * which must be one these types
         * {@link com.qti.location.sdk.IZatFlpService.IzatFlpStatus IZatFlpStatus}
         */
        void onBatchingStatusCb(IzatFlpStatus status);
    }


    /**
     * class IzatFlpRequest.
     * <p>Copyright (c) 2015 Qualcomm Technologies, Inc.</p>
     * <p>All Rights Reserved.</p>
     * <p>Confidential and Proprietary - Qualcomm Technologies, Inc</p>
     */
    public abstract class IzatFlpRequest {
        long mMaxTimeInterval = 0;
        int mMaxDistanceInterval = 0;
        long mTripDistance = 0;
        protected boolean mIsRunningInBackground = false;

        /**
         * Create FLP foreground request.
         * <p> This creates a FLP request running in foreground, which
         * is doing navigation only. With this request, applications
         * will get a callback on every fix. </p>
         */
        public static IzatFlpFgRequest getForegroundFlprequest() {
            return new IzatFlpFgRequest();
        }

        /**
         * Create FLP background request.
         * <p> This creates a FLP request running in background, which
         * is doing location batching only. With this request, applications
         * may get a callback when the batching buffer is full.</p>
         */
        public static IzatFlpBgRequest getBackgroundFlprequest() {
            return new IzatFlpBgRequest();
        }

        /**
         * Set the time interval, in milliseconds.
         * <p>
         * This allows applications to set the time interval in the FLP
         * request. Time interval is optional when distance of displacement is set.
         * The time interval set through this API is the maximum time interval, which means
         * locations could be reported with a smaller time interval if there is another
         * session set a smaller time interval. </p>
         *
         * @param  maxTimeInterval the time interval in milliseconds,
         *         which has to be greater than zero, otherwise a
         *         {@link IZatIllegalArgumentException} will be thrown.
         * @throws IZatIllegalArgumentException
         */
        public void setTimeInterval(long maxTimeInterval)
            throws IZatIllegalArgumentException {
            if (maxTimeInterval <= 0)
                throw new IZatIllegalArgumentException("invalid time interval");

            mMaxTimeInterval = maxTimeInterval;
        }

        /**
         * Get the time interval that has been set.
         * <p>
         * This allows applications to get the time interval that has
         * been set in the FLP request. It returns 0 if the time interval
         * is not set.</p>
         *
         * @return the time interval in milliseconds.
         */
        public long getTimeInterval() {
            return mMaxTimeInterval;
        }

        /**
         * Set the distance of displacement, in meters.
         * <p>
         * This allows applications to set the distance of displacement in the FLP request.
         * Distance interval is optional when time interval is set. The distance interval
         * set through this API is the maximum distance interval, which means locations
         * could be reported with a smaller distance interval if there is another
         * session set a smaller distance interval. </p>
         *
         * @param  maxDistanceInterval the distance of displacement in meters,
         *         which has to be greater than zero, otherwise a
         *         {@link IZatIllegalArgumentException} will be thrown.
         * @throws IZatIllegalArgumentException
         */
        public void setDistanceInterval(int maxDistanceInterval)
            throws IZatIllegalArgumentException {
            if (maxDistanceInterval <= 0)
                throw new IZatIllegalArgumentException("invalid distance of displacement");
            mMaxDistanceInterval = maxDistanceInterval;
        }

        /**
         * Get the distance of displacement that has been set.
         * <p>
         * This allows applications to get the distance of displacement that has
         * been set in the FLP request. It returns 0 if the distance of displacement
         * is not set. </p>
         *
         * @return the distance of displacement in meters.
         */
        public int getDistanceInterval() {
            return mMaxDistanceInterval;
        }

        /**
         * Set the trip distance in meters.
         * <p>
         * This allows applications to set the trip distance.
         * Trip distance is applicable only if request mode is
         * {@link com.qti.location.sdk.IZatFlpService.IzatFlpBgRequest.IzatFlpBgRequestMode
         * #TRIP_MODE TRIP_MODE}.
         * </p>
         *
         * @param  tripDistance the trip distance in meters,
         *         which has to be greater than zero, otherwise a
         *         {@link IZatIllegalArgumentException} will be thrown.
         * @throws IZatIllegalArgumentException
         */
        public void setTripDistance(long tripDistance)
                throws IZatIllegalArgumentException {
            if (tripDistance <= 0)
                throw new IZatIllegalArgumentException("invalid trip distance");
            mTripDistance = tripDistance;
        }

        /**
         * Get the trip distance that has been set.
         * <p>
         * This allows applications to get the trip distance
         * been set in the FLP request. It returns 0 if the distance is not set. </p>
         *
         * @return the distance of trip
         */
        public long getTripDistance() {
            return mTripDistance;
        }
    }

    class IzatFlpFgRequest extends IzatFlpRequest {
        IzatFlpFgRequest() {
            super.mIsRunningInBackground = false;
        }
    }

    class IzatFlpBgRequest extends IzatFlpRequest {

        /**
         * class IzatFlpBgRequestMode
         * IzatFlpBgRequestMode defines the background batching modes.
         */
        public enum IzatFlpBgRequestMode {
            /**
            * Routine mode will batch positions based on Time Interval only, Distance Only,
            * or Time Interval and Distance combined and it will be more power conservative,
            * with the tradeoff of potentially giving lower accuracy.
            * By default a background request is created in this mode.
            */
            ROUTINE_MODE(0),

            /**
            * Trip mode will batch positions for a trip distance at 1Hz (by default, unless
            * time interval is specified) and it will prioritize accuracy over power.
            */
            TRIP_MODE(1);

            private final int value;
            private IzatFlpBgRequestMode(int value) {
                this.value = value;
            }
            public int getValue() {
                return value;
            }
        }

        private IzatFlpBgRequestMode mActiveMode;

        IzatFlpBgRequest() {
            super.mIsRunningInBackground = true;
            mActiveMode = IzatFlpBgRequestMode.ROUTINE_MODE;
        }

        public void setActiveMode(IzatFlpBgRequestMode mode) {
            mActiveMode = mode;
        }

        public IzatFlpBgRequestMode getActiveMode() {
            return mActiveMode;
        }
    }
}
