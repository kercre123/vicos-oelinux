/*
 *    Copyright (c) 2015 Qualcomm Technologies, Inc. All Rights Reserved.
 *    Qualcomm Technologies Proprietary and Confidential.
 *
 */

package com.qualcomm.qti;

/**
 * Enum used by VehicleInterfaceData to specify the notification type to the caller.
 * Please note that this enum is work in progress, and there might be few changes going forward.
 */
public enum VehicleFrameworkNotificationType {
    /**
     * Notification to indicate signal value was refreshed.
     * New value may or may not be same as previously reported value.
     */
    VNW_SIGNAL_VALUE_REFRESHED,
    /**
     * Notification to indicate signal value changed since last notification.
     */
    VNW_SIGNAL_VALUE_CHANGED,
    /**
     * Notification to update signal value based on the interval set by the client.
     * New value may or may not be same as previously reported value.
     */
    VNW_PERIODIC_SIGNAL_VALUE_UPDATE,
    /**
     * Used when frame is not intended to be registered
     */
    VNW_SIGNAL_VALUE_NONE
}
