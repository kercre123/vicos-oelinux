/*
*    Copyright (c) 2014-2015 Qualcomm Technologies, Inc. All Rights Reserved.
*    Qualcomm Technologies Proprietary and Confidential.
*
*/
package com.qualcomm.qti.ivi.aidl;

import android.os.Messenger;

interface IVehicleService {
    /**
    *    Returns array of supported signal Ids on this platform.
    *    @return array of integers, each identifying the signal supported on this platform.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    *
    *    There is a difference between available and supported signals.
    *    Available signals are something that this framework/software supports.
    *    Platform supported signals are subset of available signals and these are the
    *    signals that an OEM decides to support.
    */
    int[] getSupportedSignalIds();

    /**
    *    Returns array of writable signal Ids available on this platform.
    *    @return array of integers, each identifying the writable
    *    signal supported on this platform.
    *    Array returned from this API is in general the sub-set of signals
    *    returned via getSupportedSignalIds API.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    *
    *    There is a difference between available and supported signals.
    *    Available signals are something that this framework/software supports.
    *    Platform supported signals are subset of available signals and these are the
    *    signals that an OEM decides to support.
    */
    int[] getWritableSignalIds();

    /**
    *    Registers a messenger with signals.
    *    @param signalIds Identify signals to be associated with the messenger.
    *    @param msngr is the handle to the messenger.
    *    @notifyTypes Identify the notification type user is interested in for each signal
                      from signalIds.
    *    @param rateMs Array which contain frequency in milli-seconds for call-backs to be fired.
    *                 This parameter is valid only if notification type is
    *                 VNW_PERIODIC_SIGNAL_VALUE_UPDATE.
    *                 For all other notification types, the parameter will be ignored.
    *    @return true if successful in registering the messenger else returns false.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    *
    *    There is a difference between available and supported signals.
    *    Available signals are something that this framework/software supports.
    *    Platform supported signals are subset of available signals and these are the
    *    signals that an OEM decides to support.
    */
    boolean registerMessenger(in int[] signalIds, in Messenger msngr, in int[] notifyTypes,
                              in int[] rateMs);

    /**
    *    Unregisters the messenger registered earlier via successful call to registerMessenger.
    *    @param signalIds Identify signals to be associated with the messenger.
    *    @param msngr is the handle to the Messenger.
    *    @return true if successful in removing the messenger else returns false.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    *
    *    There is a difference between available and supported signals.
    *    Available signals are something that this framework/software supports.
    *    Platform supported signals are subset of available signals and these are the
    *    signals that an OEM decides to support.
    */
    boolean unregisterMessenger(in int[] signalIds, in Messenger msngr);

    /**
    *    Return values associated with signals specified via @param  signalIds.
    *    @param signalIds Identify signals being queried.
    *    @return values for signals identified via @param  signalIds.
    *    Returns null if an error occurs while retrieving values.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    *
    *    There is a difference between available and supported signals.
    *    Available signals are something that this framework/software supports.
    *    Platform supported signals are subset of available signals and these are the
    *    signals that an OEM decides to support.
    */
    String[] getSignals(in int[] signalIds);

    /**
    *    Return value associated with signal specified via @param  signalId.
    *    @param signalId Identify the signal being queried.
    *    @return value for signal identified via @param  signalId.
    *    Returns null if an error occurs while retrieving value.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    */
    String getSignal(in int signalId);

    /**
    *    Set values associated with signals specified via @param  signalIds.
    *    @param signalIds Point to signals to be set.
    *    @param signalValues Point to values to be set.
    *    @return true if successful in setting signal values else returns false.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    *
    */
    boolean setSignals(in int[] signalIds, in String[] signalValues);

    /**
    *    Set value associated with signal specified via @param  signalId.
    *    @param signalId Point to the signal to be set.
    *    @param signalValue Point to value to be set.
    *    @return true if successful in setting signal values else returns false.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    *
    */
    boolean setSignal(in int signalId, in String signalValue);

    /**
    *    Return value associated with user value specified via @param signalId.
    *    @param signalId Identify the user value being queried.
    *    @return value for user value identified via @param signalId.
    *    Returns null if an error occurs while retrieving value.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    */
    String getUserValue(in int signalId);

    /**
    *    Retrieve data types for signals identified by @param signalIds.
    *    Refer to com.qualcomm.qti.VehicleSignalConstants for information on data types.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    *
    */
    int[] getSignalsDataType(in int[] signalIds);

    /**
    *    Retrieve data types for signal identified by @param signalId.
    *    Refer to com.qualcomm.qti.VehicleSignalConstants for information on data types.
    *    Please refer to com.qualcomm.qti.VehicleInterfaceSignals to find out
    *    list of available signals on this platform.
    */
    int getSignalDataType(in int signalId);
}
