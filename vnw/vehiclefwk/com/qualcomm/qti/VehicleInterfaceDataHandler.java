/*
 *    Copyright (c) 2014-2015 Qualcomm Technologies, Inc. All Rights Reserved.
 *    Qualcomm Technologies Proprietary and Confidential.
 *
 */

package com.qualcomm.qti;

/**
 * Interface contains callback to be called to notify change in data or when an
 * error occurs. Please note that this interface is work in progress, and there
 * might be few changes going forward.
 */
public interface VehicleInterfaceDataHandler {
    /**
    * Called by VehicleInterfaceData to notify client
    * @param notificationType Specify the type of notification.
    * @see VehicleFrameworkNotificationType for various notification types.
    * @param signalId Specify the signal id to be associated with the notification.
    */
    public void onNotify(VehicleFrameworkNotificationType notificationType, int signalId);

    /**
    * Called by VehicleInterfaceData to notify when an error occurs.
    * @param bCleanUpAndRestart provides a hint to the caller to take the
    *            appropriate action.
    * @param bCleanUpAndRestart = true indicates a fatal error and expects
    *            caller to clean up and restart.
    */
    public void onError(boolean bCleanUpAndRestart);
}

