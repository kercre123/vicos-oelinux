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
      else if( itemName.compare( "MinAngle" ) == 0 )
      {
         iss >> parameter.minAngle;
         printf( "MinAngle for keyframe creator: %f\n", parameter.minAngle );
      }
      else if( itemName.compare( "CutoffDepth" ) == 0 )
      {
         iss >> parameter.cutoffDepth;
         printf( "CutoffDepth for keyframe creator: %f\n", parameter.cutoffDepth);
      }
	  else if (itemName.compare("ConvexFactor") == 0)
	  {
		  iss >> parameter.convexFactor;
		  printf("ConvexFactor for keyframe creator: %f\n", parameter.convexFactor);
	  }
	  else if (itemName.compare("DeadZone") == 0)
	  {
		  iss >> parameter.deadZone;
		  printf("DeadZone for keyframe creator: %f\n", parameter.deadZone);
	  }
      else if( itemName.compare( "MinDelay" ) == 0 )
      {
         iss >> parameter.minDelay;
         printf( "MinDelay for keyframe creator:       %d\n", parameter.minDelay );
      }
	  else if (itemName.compare("UseDynamicThreshold") == 0)
	  {
		  iss >> parameter.useDynamicThreshold;
		  printf("UseDynamicThreshold for keyframe creator:       %d\n", parameter.useDynamicThreshold);
	  }
   }
   return 0;
}



