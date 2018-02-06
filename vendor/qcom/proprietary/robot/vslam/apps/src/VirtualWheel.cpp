/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "VirtualWheel.h"


#include "Queue.h"
#include "mvWEF.h"
#include <string>
#include "math.h"

VirtualWheel::VirtualWheel( const std::string & wheelOdomName )
{
   wheelodomStream.open( wheelOdomName.c_str() );
   if( !wheelodomStream.is_open() )
   {
      printf( "Cannot open file %s for wheel odom reading!\n", wheelOdomName.c_str() );
   }
   else
   {
      printf( "Open file %s for wheel odom reading!\n", wheelOdomName.c_str() );
   }
}

VirtualWheel::~VirtualWheel()
{

}

bool VirtualWheel::GetWheelOdom( mvWEFPoseVelocityTime& wheelodom )
{

   if( wheelodomStream >> wheelodom.timestampUs )
   {
      wheelodomStream >> wheelodom.pose.translation[0] >> wheelodom.pose.translation[1] >> wheelodom.pose.translation[2];
      double qx, qy, qz, qw;
      wheelodomStream >> qx >> qy >> qz >> qw;
      double qyy = qy*qy;
      wheelodom.pose.euler[0] = (float)atan2( 2 * (qx*qw + qy*qz), 1 - 2 * (qx*qx + qyy) );
      wheelodom.pose.euler[1] = (float)asin( 2 * (qy*qw - qx*qz) );
      wheelodom.pose.euler[2] = (float)atan2( 2 * (qz*qw + qx*qy), 1 - 2 * (qyy + qz*qz) );
      double velocityNotUsed;
      wheelodomStream >> wheelodom.velocityLinear;
      wheelodomStream >> velocityNotUsed; wheelodomStream >> velocityNotUsed; // velocityLinear along y and z axis
      wheelodomStream >> velocityNotUsed; wheelodomStream >> velocityNotUsed; // velocityAngular along x and y axis
      wheelodomStream >> wheelodom.velocityAngular;

      char linebreak = wheelodomStream.get();
      if( '\n' != linebreak )
      {
         printf( "Unsupported data format for wheelodom (LF expected but %c received)!\n", linebreak );
         return false;
      }
   }
   else
   {
      printf( "Cannot read out wheel timestamp from file!\n" );
      return false;
   }

   return !wheelodomStream.eof();
}


