/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VIRTUAL_WHEEL_H__
#define __VIRTUAL_WHEEL_H__


#include <fstream>
#include "mvWEF.h"

class VirtualWheel
{
public:
   VirtualWheel(const std::string & wheelOdomName );
   ~VirtualWheel();

   bool GetWheelOdom( mvWEFPoseVelocityTime& wheelodom );

private:

   std::ifstream wheelodomStream;  

};
#endif