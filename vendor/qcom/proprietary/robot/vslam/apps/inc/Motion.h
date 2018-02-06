/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


//==============================================================================
// Defines
//==============================================================================

//==============================================================================
// Includes
//==============================================================================

#include "Queue.h"
#include "MoveAlongPathPattern.h"

enum MOTION_PATTERN {
   EXIT_PATTERN = 0, // Pattern for exiting the thread
	TARGET_INIT_PATTERN, // Get first pose for target initilization
	TARGETLESS_INIT_PATTERN, //Get first pose for no target initilization
	SCALE_ESTIMATION_PATTERN, // Compute scale
   LOOP_CLOSURE_PATTERN // Complete a loop closure
};

enum MOTION_STATUS {
	SUCCESS,
	FAIL
};

extern queue_mt<MOTION_PATTERN> gMotionPatternQueue;

/**--------------------------------------------------------------------------
@brief
Set the motion pattern
@param MOTION_PATTERN
--------------------------------------------------------------------------**/
void setMotionPattern(MOTION_PATTERN);
