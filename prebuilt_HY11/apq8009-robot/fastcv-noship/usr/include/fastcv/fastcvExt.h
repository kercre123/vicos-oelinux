#ifndef FASTCVEXT_H
#define FASTCVEXT_H

/**=============================================================================

@file
   fastcvExt.h

@brief
   Public extended API

Copyright (c) 2014 Qualcomm Technologies, Inc. All Rights Reserved.
Qualcomm Technologies Proprietary and Confidential.

=============================================================================**/

//==============================================================================
// Included modules
//==============================================================================
#include "fastcv.h"

#ifdef __cplusplus
extern "C"
{
#endif

//==============================================================================
// Declarations
//==============================================================================

//------------------------------------------------------------------------------
/// @brief
///    Defines operational mode of interface to allow the end developer to
///    dictate how the target optimized implementation should behave.
//------------------------------------------------------------------------------
typedef enum
{
   /// Target-optimized implementation uses as much QDSP Unit as possible
   FASTCV_OP_EXT_QDSP,

   /// Target-optimized implementation uses as much Graphics Processing Unit as possible
   FASTCV_OP_EXT_GPU

} fcvOperationModeExt;

//---------------------------------------------------------------------------
/// @brief
///   Selects HW units for all routines at run-time.  Can be called anytime to
///   update choice.
///
/// @param mode
///   See enum for details.
///
/// @return
///   FASTCV_SUCCESS if successful.
///   FASTCV_EFAIL if unsuccessful.
///
/// @ingroup misc
//---------------------------------------------------------------------------

FASTCV_API fcvStatus
fcvSetOperationModeExt( fcvOperationModeExt modeExt );

#ifdef __cplusplus
}//extern "C"
#endif

#endif
