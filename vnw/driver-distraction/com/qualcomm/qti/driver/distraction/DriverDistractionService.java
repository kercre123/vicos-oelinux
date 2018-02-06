/*
 *    Copyright (c) 2015 Qualcomm Technologies, Inc. All Rights Reserved.
 *    Qualcomm Technologies Proprietary and Confidential.
 *
 */

package com.qualcomm.qti.driver.distraction;

import android.app.Service;
import android.content.Intent;
import android.os.Bundle;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;
import java.util.ArrayList;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import android.util.Xml;
import java.util.Arrays;

import com.qualcomm.qti.VehicleFrameworkNotificationType;
import com.qualcomm.qti.VehicleInterfaceData;
import com.qualcomm.qti.VehicleInterfaceDataHandler;
import com.qualcomm.qti.VehicleInterfaceSignals;
import com.qualcomm.qti.VehicleSignalConstants;
import com.qualcomm.qti.VehicleManager;

import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;
import org.xmlpull.v1.XmlPullParserFactory;

import java.util.Collection;
import java.util.Vector;

public final class DriverDistractionService extends Service {
    private static final String TAG = "DriverDistractionService";
    private static final String mXmlFile = "system/etc/vnw/distraction-settings.xml";
    private static final String nameSpace = null;
    private static final int START_TAG = XmlPullParser.START_TAG;
    private static final int END_TAG = XmlPullParser.END_TAG;
    private static final int TEXT = XmlPullParser.TEXT;
    private ArrayList<VidEvent> mEventListFromXml;
    private ArrayList<Integer> mSignalIdlist;
    private XmlPullParser mParser;
    private static DriverDistractionService mMyself = null;
    private Vector<Messenger> mGenericCallbackVector;
    private VehicleManager mVehicleMgr;
    private VehicleManagerStatus mVmStatusCallback;
    private VehicleInterfaceData mVehicleDataHandle;
    private VehicleDataNotification mDataNotification;
    public static final int VEHICLE_DISTRACTION_DEFAULT_SPEED_THRESHOLD = 1;
    private static final int VEHICLE_MANAGER_ENABLE_DRIVER_DISTRACTION_INTENT_ID = 0;
    private static final int VEHICLE_MANAGER_DISABLE_DRIVER_DISTRACTION_INTENT_ID = 1;
    private String mLastDistractionStatusSent;

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

    @Override
    public void onCreate() {
    }

    @Override
    public int onStartCommand(Intent intent, int flag, int statusId) {
        Log.v(TAG, "starting driver distraction module...");
        mMyself = new DriverDistractionService();
        VehicleManager.getInstance(getBaseContext(), mVmStatusCallback);
        return START_STICKY;
    }

    /**
     * Registers the messenger to be notified.
     * @return true if successful in registering the messenger else returns false.
     */
    public boolean registerMessenger(Messenger msngr) {
        if (msngr != null) {
            if (mGenericCallbackVector != null) {
                mGenericCallbackVector.add(msngr);
                return true;
            } else {
                Log.e(TAG, "registerMessenger failed.mGenericCallbackVector is null");
            }
        } else {
            Log.e(TAG, "registerMessenger failed.msngr is null");
        }
        return false;
    }

    /**
     * Removes the messenger registered earlier via call to registerMessenger.
     */
    public boolean unregisterMessenger(Messenger msngr) {
        if (msngr != null) {
            if (mGenericCallbackVector != null) {
                mGenericCallbackVector.remove(msngr);
                return true;
            } else {
                Log.e(TAG, "unregisterMessenger failed.mGenericCallbackVector is null");
            }
        } else {
            Log.e(TAG, "unregisterMessenger failed.msngr is null");
        }
        return false;
    }

    /**
    * Constructor for DriverDistractionService.
    */
    public DriverDistractionService() {
        mEventListFromXml = new ArrayList<VidEvent>();
        mSignalIdlist = new ArrayList<Integer>();
        parseFromXmlFile(mXmlFile);
        mVmStatusCallback = new VehicleManagerStatus();
        mGenericCallbackVector = new Vector();
        mDataNotification = new VehicleDataNotification();
    }

    private void parseFromXmlFile(String fileName) {
        InputStream inStream = null;
        try {
            boolean ret = true;
            inStream = new FileInputStream(fileName);
            mParser = Xml.newPullParser();
            mParser.setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false);
            mParser.setInput(inStream, null);
            mParser.nextTag();
            String name = mParser.getName();
            while (!name.equals("vnwsettings")) {
                  mParser.nextTag(); // will throw exception if EOF
                  name = mParser.getName();
            }
            while (mParser.next() != END_TAG) { // .next advances parser
                // skip comments and whitespace
                if (mParser.getEventType() != START_TAG) {
                     continue;
                 }
                 name = mParser.getName();
                 if (name.equals("event")) {
                     readEvent();
                 }
           }
        } catch (XmlPullParserException parserErr) {
            Log.e(TAG, "XML Parser Threw: " + parserErr.getMessage());
        } catch (IOException ioErr) {
            Log.e(TAG, "IOError: " + ioErr.getMessage());
        } finally {
            try {
                if (inStream != null) {
                    inStream.close();
                }
            } catch (IOException ioErr) {
                Log.e(TAG, "Error closing input: " + ioErr.getMessage());
            }
        }
    }

    private void readEvent() throws XmlPullParserException, IOException {
        String eventType = mParser.getAttributeValue(null, "string");
        VidEvent nVidEvent = new VidEvent();
        nVidEvent.setEventName(eventType);
        while (mParser.next() != END_TAG) {
            if (mParser.getEventType() != START_TAG) {
                continue;
            }
            String name = mParser.getName();
            if (name.equals("signal")) {
                VidDistractionSignal signal = readSignal();
                if (signal != null) {
                    nVidEvent.addToEventMap(signal.mSigId, signal);
                }
            } else { // unexpected tag, skip thru the end_tag
                // $ToDo report this error??
                while (mParser.next() != END_TAG) {
                }; // skip thru end tag
            }
        }
        mEventListFromXml.add(nVidEvent);
    }

    private VidDistractionSignal readSignal() throws XmlPullParserException,IOException {
        int signalId = getAttributeInt("id", 0);
        String signalName = getAttributeString("name", null);
        VidDistractionSignal signal = new VidDistractionSignal (signalId, signalName);
        while (mParser.next() != END_TAG) { // .next advances parser
            // skip comments and whitespace
            if (mParser.getEventType() != START_TAG) {
                continue;
            }
            String name = mParser.getName();
            if (name.equals("threshold")) {
                readThreshold(signal);
            } else { // unexpected tag, skip thru the end_tag
                // $ToDo report this error??
                while (mParser.next() != END_TAG) {
                }; // skip thru end tag
            }
        }
        return signal;
    }

    private void readThreshold(VidDistractionSignal signal) throws
            XmlPullParserException,IOException {
        int value = getAttributeInt("value", -1);
        String condition = getAttributeString("condition", null);
        if (signal != null) {
            signal.setValueAndCondition(value, condition);
        }
        skipToEndTag();
    }

    private class VidEvent {
        private String TAG = "VidEvent";
        StringBuilder mEventName;
        private Map<Integer, VidDistractionSignal> mEventMap;
        private boolean misLastSignalUpdateSentNotification;
        VidEvent() {
            mEventName = new StringBuilder();
            mEventMap = new HashMap<Integer, VidDistractionSignal>();
        }
        void setEventName(String eventName) {
            if (mEventName.length() == 0 ) {
                mEventName.append(eventName);
            }
        }
        StringBuilder getEventName() {
            return mEventName;
        }
        void addToEventMap(int signalId, VidDistractionSignal signal) {
            int ret = 0;
            if (mEventMap != null) {
                mEventMap.put(signalId, signal);
            }
        }
        VidDistractionSignal getEventSignal(int signalId) {
            return mEventMap.get(signalId);
        }
        boolean checkEvent (int signalId) {
            boolean bRet = false;
            if (mEventMap != null) {
                if (mEventMap.containsKey(signalId) == true) {
                    //this event contains the signalId
                    Collection<VidDistractionSignal> signals = mEventMap.values();
                    bRet = true;
                    //check status of all signals in this event
                    for ( Object sigObject: signals.toArray()) {
                        bRet &= ((VidDistractionSignal)sigObject).checkSignal();
                    }
                    if (bRet == true) {
                        /*
                        * All conditions are true for this event;
                        * To avoid duplicate notifications, check if last update resulted in
                        * sending the notification.
                        */
                        if (misLastSignalUpdateSentNotification == false) {
                            //We are going to send the notification now, update our variable
                            misLastSignalUpdateSentNotification = true;
                        } else {
                            //Log.v(TAG, "Dropping duplicate "+mEventName+ " notification..");
                            bRet = false;
                        }
                    } else {
                        //All specified conditions are not true, reset our variable
                        //so that next time when all conditions are met, we will send the
                        //notification.
                        misLastSignalUpdateSentNotification = false;
                    }
                }
            }
            return bRet;
        }
    } // end class VidEvent

    private class VidDistractionSignal {
        private String TAG = "VidDistractionSignal";
        private int mSigId;
        private String mSignalName;
        private String mCondition;
        private int mValue;

        VidDistractionSignal(int sigId, String sigName) {
            mSigId = sigId;
            mSignalName = sigName;
        }
        void setValueAndCondition(int value, String condition) {
            mValue = value;
            mCondition = condition;
        }
        boolean checkSignal() {
            int value = 0;
            boolean bRet = false;
            String valueString = mVehicleDataHandle.getSignal(mSigId);
            if (valueString != null) {
                value = Integer.parseInt(valueString.trim());
            } else {
                Log.e(TAG, "checkSignal mSigId "+mSigId+" getSignal returned null..");
            }
            //check condition
            if ((mCondition != null) && (valueString != null)) {
                if (mCondition.trim().equals("<") == true ) {
                    if (value < mValue) {
                        bRet = true;
                    }
                } else if (mCondition.trim().equals("<=") == true ) {
                    if (value <= mValue) {
                        bRet = true;
                    }
                } else if (mCondition.trim().equals(">") == true ) {
                    if (value > mValue) {
                        bRet = true;
                    }
                } else if (mCondition.trim().equals(">=") == true ) {
                    if (value >= mValue) {
                        bRet = true;
                    }
                } else if (mCondition.trim().equals("==") == true ) {
                    if (value == mValue) {
                        bRet = true;
                    }
                } else if (mCondition.trim().equals("!=") == true ) {
                    if (value != mValue) {
                        bRet = true;
                    }
                }
            } else {
                Log.e(TAG, "checkSignal mCondition or valueString is null...");
            }
            return bRet;
        }
    }

    private int [] getSignalIdsFromList() {
        int[] signals = null;
        for (int i=0; i < mEventListFromXml.size(); i++) {
             for (int k: mEventListFromXml.get(i).mEventMap.keySet()) {
                   if (!mSignalIdlist.contains(k))
                   mSignalIdlist.add(k);
             }
        }
        Object[] sigList = mSignalIdlist.toArray();
        signals = new int[sigList.length];
        int index = 0;
        for (Object i : sigList) {
            signals[index++] = ((Integer)i).intValue();
        }
        return signals;
    }

    private int getAttributeInt(String name, int defaultValue) throws
            XmlPullParserException, IOException {
        int val = defaultValue;
        String str = mParser.getAttributeValue(nameSpace, name);
        if (str != null) {
            val = Integer.parseInt(str);
        }
        return val;
    }

    private String getAttributeString(String name, String defaultValue) throws
            XmlPullParserException, IOException {
        String val = defaultValue;
        String str = mParser.getAttributeValue(nameSpace, name);
        if (str != null) {
            val = str;
        }
        return val;
    }

    private double getAttributeDouble(String name, double defaultValue) throws
            XmlPullParserException, IOException {
        double val = defaultValue;
        String str = mParser.getAttributeValue(nameSpace, name);
        if (str != null) {
            val = Double.parseDouble(str);
        }
        return val;
    }

    private void skipToEndTag() throws XmlPullParserException, IOException {
        int level = 0;
        while (level >= 0) {
            int type = mParser.next();
            if (type == START_TAG) {
                level++; // have an interceding element to skip
            }
            if (type == END_TAG) {
                level--; // finished some element
            }
        }
    }

    private void notifyExternalClient(String status) {
        if (mGenericCallbackVector != null) {
            for (Messenger element : mGenericCallbackVector) {
                if (element != null) {
                    //Message.obtain(Handler, what, arg1, arg2, Object o);
                    Message clntmsg = Message.obtain(null, 0, 0, 0, null);
                    Bundle bundle = new Bundle();
                    bundle.putString("distraction", status);
                    clntmsg.setData(bundle);
                    try {
                        element.send(clntmsg);
                    } catch (RemoteException ex) {
                        /**
                        * This may not be an error as this could happen if app/client has
                        * crashed and service has stale callbacks/messenger objects.
                        * We should remove these stale messenger objects as we encounter them
                        * to keep service clean. One can test this by manually killing the
                        * application to make sure service cleans up dead objects.
                        */
                        Log.e(TAG, "notifyExternalClient RemoteException.."+
                                "removing messenger from vector");
                        mGenericCallbackVector.remove(element);
                    }
                }
            }
        }
    }
    private IDriverDistractionService.Stub mBinder = new IDriverDistractionService.Stub() {
        /**
        * Registers the messenger to be called when there is an update to data.
        */
        public boolean registerMessenger(Messenger msngr) {
            return DriverDistractionService.this.registerMessenger(msngr);
        }
        /**
        * Unregisters the messenger set earlier via call to registerMessenger.
        */
        public boolean unregisterMessenger(Messenger msngr) {
            return DriverDistractionService.this.unregisterMessenger(msngr);
        }
        private static final String TAG = "IDriverDistractionService";
    };

    private class VehicleManagerStatus implements VehicleManager.VehicleManagerCallback {
        public void handleVehicleManagerCreationStatus(VehicleManager handle) {
            mVehicleMgr = handle;
            if (handle != null) {
                mVehicleDataHandle = handle.getInterfaceHandle();
                //get all the signals specified in distraction xml file.
                int[] signals = getSignalIdsFromList();
                VehicleFrameworkNotificationType[] notificationTypes =
                        new VehicleFrameworkNotificationType [signals.length];
                for (int i = 0; i < signals.length; i++) {
                    notificationTypes[i] =
                            VehicleFrameworkNotificationType.VNW_SIGNAL_VALUE_REFRESHED;
                }
                //register listener for all the signals mentioned in distraction xml file.
                if (mVehicleDataHandle != null) {
                    mVehicleDataHandle.registerHandler(signals, mDataNotification,
                            notificationTypes,null);
                }
            }
        }
    }

    private class VehicleDataNotification implements VehicleInterfaceDataHandler {
        public void onNotify(VehicleFrameworkNotificationType type, int signalId) {
            checkAndBroadcastEvent(signalId);
        }
        public void onError(boolean bCleanUpAndRestart) {
            //Log.v(TAG, "bCleanUpAndRestart="+ bCleanUpAndRestart);
            //We might run into several RemoteException based on number of signals
            //registered for callback with the service. We should discard duplicate ones.
            //There is still a small window where this could still happen.
            //If one or more duplicate exceptions are delayed in service itself and appear
            //after we have retsrated the application, following check will not be useful.
            if ( (bCleanUpAndRestart == true) && (mVehicleMgr != null) ) {
                //sleep for 1 sec to avoid spamming in case onError is called repeatedly
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    Log.e(TAG, "Caught exception while sleeping in onError: " + e);
                }
                Log.e(TAG, "VehicleDataNotification::onError, creating VehicleManager from start");
                cleanUpAndRestart();
            }
        }
    }
    private void checkAndBroadcastEvent(int signalId) {
        if ((mEventListFromXml != null) && (mEventListFromXml.size() > 0)) {
            for (Object event: mEventListFromXml.toArray()) {
                VidEvent xmlEvent = (VidEvent) event;
                if ((xmlEvent != null) && ( xmlEvent.checkEvent(signalId) == true)) {
                    if (updateDriverDistraction(xmlEvent.getEventName().toString()) == true) {
                        notifyExternalClient(xmlEvent.getEventName().toString());
                    }
                }
            }
        } else {
            Log.e(TAG, "mEventListFromXml is null or has no element");
        }
    }
    private void cleanUpAndRestart() {
        mVehicleMgr = null;
        mVehicleDataHandle = null;
        VehicleManager.getInstance(getBaseContext(), mVmStatusCallback);
    }
    private boolean updateDriverDistraction(String eventString) {
        boolean bSendIntent = true;
        if (mLastDistractionStatusSent != null) {
            if (mLastDistractionStatusSent.equals(eventString) == true) {
                //we do not want to send same intent over and over..
                bSendIntent = false;
            }
        }
        if (bSendIntent == true) {
            Intent i = new Intent(eventString);
            if (getBaseContext() != null) {
                sendBroadcast(i);
            }
        }
        return bSendIntent;
    }
}
