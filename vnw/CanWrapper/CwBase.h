/*
 * Copyright (c) 2014, 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
/*
 * CwBase.h
 *  Can Wrapper Common items
 *  Provides some Base definitions
 *  Now this file need not be inherited,
 *  it may simply be included.
 */

#ifndef CwBase_H_
#define CwBase_H_

/////////////////////////////////////////////////////////////////////////
//$ToDo$ - create a Macro configurable debug method
//        e.g. printf --> console for straight unix
//-----------------------------------------------------------------------
#include <inttypes.h>
#include <stdio.h>

#ifndef  LOG_TAG
#define LOG_TAG "**CANLIB:**"
#endif

#ifndef LINUX_AUTO_MAKE
#include <cutils/log.h>
#endif
//=====================================================================//
// LOGGING
//


#ifndef LINUX_AUTO_MAKE
    #ifdef LOG_TAG
        #define CWLOG(...)     __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
        #define CWERR(...)     __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
    #else
        #define CWLOG(...)
        #define CWERR(...)
    #endif
#else
    #define CWLOG(...) printf(__VA_ARGS__);
    #define CWERR(...) printf(__VA_ARGS__);
#endif

#define FRAME_FILTER_TYPE_ON_REFRESH    0
#define FRAME_FILTER_TYPE_ON_CHANGE     1
//=====================================================================//
/* prototypes we are matching ...
#define  LOGD(...)  __android_log_vprint(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_vprint(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
*/

//=====================================================================//
#define __EXPORTED_HEADERS__  // enable kernel type propagation
///////////////////////////////////////////////////////////////
// CwBase Class is a base object for every Class in CanWrapper -
// This includes CwFrame and CanWrapper which are User facing.
// It
// - acts as a namespace for common "values",
//   - e.g. IFACE_UNDEF, ID_NORM
// - adds some static methods for dealing with these values
//
class CwBase {
public:
    CwBase() {;}
    virtual ~CwBase() {;}
    //
    //----------------------------------------------------------
    // Useful CanWrapper package constants
public:
    // special ifNos (interface numbers)
    static const int IFACE_UNDEF = -1;  // is undefined
    static const int IFACE_ANY = -2;    // matches any interfaces
    //
    static const int MAX_DATA = 64;  // room for full FD frame
    //
    // can frame id masks
    static const int MASK11 = 0x7FF;        // ID_NORM's mask (2^11)-1
    static const int MASK29 = 0x1FFFFFFF;   // ID_EXT's mask  (2^29)-1
    //
    // can frame id types
    static const int ID_UNDEF = -1;
    static const int ID_ERR = 0;
    static const int ID_NORM = 1;
    static const int ID_EXT = 2;
    static const int ID_FD = 3;
    //
public:
    //-----------------------------------------------
    // static interface and mask helpers
    //-----------------------------------------------
    //
    // return mask for a given type - cannot be an error!
    static uint32_t typeToMask(int type = ID_NORM) {
        return (type == ID_NORM) ? MASK11 : MASK29;
    }
    //
    // is this an invalid, i.e. special, interface (one of ours, like IFACE_ANY)
    static int isIfaceSpecial(int ifNo) {return ifNo < 0;}
};
///////////////////////////////////////////////////////////////
#endif /* CwBase_H_ */
