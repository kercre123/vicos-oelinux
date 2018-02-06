/*
*    Copyright (c) 2015 Qualcomm Technologies, Inc. All Rights Reserved.
*    Qualcomm Technologies Proprietary and Confidential.
*
*/
package com.qualcomm.qti.driver.distraction;

import android.os.Messenger;

interface IDriverDistractionService {
    /**
    *    Registers a messenger to be called to notify driver distraction status..
    *    @param msngr is the handle to the messenger.
    *    @return true if successful in registering the messenger else returns false.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    */
    boolean registerMessenger(in Messenger msngr);

    /**
    *    Unregisters the messenger registered earlier via successful call to registerMessenger.
    *    @param msngr is the handle to the Messenger.
    *    @return true if successful in removing the messenger else returns false.
    *    Please note that this API is work in progress and there can be few changes in
    *    subsequent revisions of this file.
    */
    boolean unregisterMessenger(in Messenger msngr);
}
