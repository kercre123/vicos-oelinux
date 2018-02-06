/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "VSLAM_internal.h"
#include <string>
#include <fstream>
#include <sstream>

extern std::string Program_Root;

int ParseEngineParameters( const char * parameterFile, vslamparameterInternal & parameter )
{
   std::string fullName = Program_Root + parameterFile;
   std::ifstream cfg( fullName.c_str(), std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s. Set default value. \n", fullName.c_str() );
      parameter.minDistance = 0.2f;
      parameter.minDistanceAngle = 0.1f;
      parameter.minAngleAngle = 0.1f;
      parameter.minDelay = 6;
      return 0;
   }

   std::string line;
   std::string itemName;
   while( std::getline( cfg, line ) )
   {
      if( line.length() == 0 )
      {
         continue;
      }
      if( line[0] == '#' )
      {
         continue;
      }
      std::istringstream iss( line );
      itemName.clear();
      iss >> itemName;

      if( itemName.compare( "MinDistance" ) == 0 )
      {
         iss >> parameter.minDistance;
         printf( "MinDistane for keyframe creator:       %f\n", parameter.minDistance );
      }
      else if( itemName.compare( "MinDistanceAngle" ) == 0 )
      {
         iss >> parameter.minDistanceAngle;
         printf( "MinDistanceAngle for keyframe creator: %f\n", parameter.minDistanceAngle );
      }
      else if( itemName.compare( "MinAngleAngle" ) == 0 )
      {
         iss >> parameter.minAngleAngle;
         printf( "MinAngleAngle for keyframe creator: %f\n", parameter.minAngleAngle );
      }
      else if( itemName.compare( "MinDelay" ) == 0 )
      {
         iss >> parameter.minDelay;
         printf( "MinDelay for keyframe creator:       %d\n", parameter.minDelay );
      }
   }
   return 0;
}



