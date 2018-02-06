/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "VSLAM.h"
#include "WEF.h"
#include "Camera_VSLAM.h"

#include <string>
#include <fstream>
#include <sstream>

#include "string.h"
#include "mainThread.h"

extern std::string Program_Root;
extern int32_t failPoseNum;
extern int initModePara;

void ReadMatrix( std::ifstream & file, float * matrix, int number )
{
   std::string line;
   std::getline( file, line );
   std::getline( file, line );
   std::getline( file, line );
   size_t index = line.find_first_of( '[' );
   size_t length = line.length();
   std::string matrixStr = line.substr( index + 1, length - index - 1 );
   while( std::getline( file, line ) )
   {
      size_t index1 = line.find_first_of( ']' );
      if( index1 == std::string::npos )
      {
         matrixStr = matrixStr + line;
      }
      else
      {
         matrixStr = matrixStr + line.substr( 0, index1 );
         break;
      }
   }

   std::istringstream iss( matrixStr );

   std::string numberStr;
   for( int i = 0; i < number; i++ )
   {
      std::getline( iss, numberStr, ',' );
      matrix[i] = std::stof( numberStr );
   }
}


bool GetCameraParameter( VSLAMCameraParams & cameraParameter, const char *cameraID )
{
   float p[12];
   std::string fullName = Program_Root + cameraID;
   std::ifstream cfg( fullName, std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", fullName.c_str() );
      return false;
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
      iss >> itemName;
      if( itemName.compare( "image_width:" ) == 0 )
      {
         iss >> cameraParameter.inputPixelWidth;
      }
      else if( itemName.compare( "image_height:" ) == 0 )
      {
         iss >> cameraParameter.inputPixelHeight;
      }
      else if( itemName.compare( "camera_matrix:" ) == 0 )
      {
         ReadMatrix( cfg, cameraParameter.inputCameraMatrix, 9 );
      }
      else if( itemName.compare( "distortion_coefficients:" ) == 0 )
      {
         ReadMatrix( cfg, cameraParameter.distortionCoefficient, 8 );
      }
      else if( itemName.compare( "projection_matrix:" ) == 0 )
      {
         ReadMatrix( cfg, p, 12 );
         cameraParameter.outputCameraMatrix[0] = p[0];
         cameraParameter.outputCameraMatrix[1] = p[1];
         cameraParameter.outputCameraMatrix[2] = p[2];
         cameraParameter.outputCameraMatrix[3] = p[4];
         cameraParameter.outputCameraMatrix[4] = p[5];
         cameraParameter.outputCameraMatrix[5] = p[6];
         cameraParameter.outputCameraMatrix[6] = p[8];
         cameraParameter.outputCameraMatrix[7] = p[9];
         cameraParameter.outputCameraMatrix[8] = p[10];
      }
   }
   return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <fstream>
#include <sstream>
int ParseVWSLAMConf( const char * parameterFile, VSLAMParameter & vslamPara, WEFParameter & wefPara )
{
   std::string fullName = Program_Root + parameterFile;
   std::ifstream cfg( fullName.c_str(), std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", fullName.c_str() );
      return -1;
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
      if( itemName.compare( "FailVSLAMPoseNum" ) == 0 )
      {
         iss >> failPoseNum;
         printf( "FailVSLAMPoseNum is:       %d\n", failPoseNum );
      }
      else if( itemName.compare( "MapPath" ) == 0 )
      {
         iss >> vslamPara.mapPath;
         int tmp = strlen( vslamPara.mapPath );
         if( vslamPara.mapPath[tmp - 1] != '/' && vslamPara.mapPath[tmp - 1] != '\\' && tmp < 198 )
         {
            vslamPara.mapPath[tmp] = '/';
         }
         printf( "Load existing map from:       %s\n", vslamPara.mapPath );
      }
      if( itemName.compare( "MaxKeyFrame" ) == 0 )
      {
         iss >> vslamPara.maxKeyFrame;
         printf( "Max number of keyframe: %d\n", vslamPara.maxKeyFrame );
      }
      else if( itemName.compare( "TargetImage" ) == 0 )
      {
         iss >> vslamPara.targetImagePath;
         printf( "Load target image:       %s\n", vslamPara.targetImagePath );
      }
      else if( itemName.compare( "TargetWidth" ) == 0 )
      {
         iss >> vslamPara.targetWidth;
         printf( "Target width = %f\n", vslamPara.targetWidth );
      }
      else if( itemName.compare( "TargetHeight" ) == 0 )
      {
         iss >> vslamPara.targetHeight;
         printf( "Target height = %f \n", vslamPara.targetHeight );
      }
      else if( itemName.compare( "VSLAMIniMode" ) == 0 )
      {
         if( initModePara == INIT_MODE_NONE )  //
         {
            std::string temp;
            iss >> temp;
            if( temp.compare( "TARGET_INIT" ) == 0 )
            {
               vslamPara.initMode = VSLAMInitMode::TARGET_INIT;
               printf( "Initialization mode:    target\n" );
            }
            else if( temp.compare( "TARGETLESS_INIT" ) == 0 )
            {
               vslamPara.initMode = VSLAMInitMode::TARGETLESS_INIT;
               printf( "Initialization mode:    scale free\n" );
            }
            else if( temp.compare( "RELOCALIZATION" ) == 0 )
            {
               vslamPara.initMode = VSLAMInitMode::RELOCALIZATION;
               printf( "Initialization mode:    loading map\n" );
            }
            else
            {
               printf( "Error!   Invalid initialization mode %s. Please check!\n", temp.c_str() );
            }
         }
         else
         {
            vslamPara.initMode = (VSLAMInitMode)initModePara;
         }
      }
      else if( itemName.compare( "vslamStateBadAsFail" ) == 0 )
      {
         iss >> wefPara.vslamStateBadAsFail;
         printf( "vslamStateBadAsFail:    %d\n", wefPara.vslamStateBadAsFail );
      }
      else if( itemName.compare( "WEF.Tvb" ) == 0 )
      {
         iss >> wefPara.poseVB.translation[0] >> wefPara.poseVB.translation[1] >> wefPara.poseVB.translation[2];
      }
      else if( itemName.compare( "WEF.Rvb" ) == 0 )
      {
         iss >> wefPara.poseVB.euler[0] >> wefPara.poseVB.euler[1] >> wefPara.poseVB.euler[2];
      }
      else if( itemName.compare( "wheelEnabled" ) == 0 )
      {
         iss >> vslamPara.wheelEnabled;
         printf( "WheelEnabled:       %d\n", vslamPara.wheelEnabled );
      }
      else if( itemName.compare( "loopClosureEnabled" ) == 0 )
      {
         iss >> vslamPara.loopClosureEnabled;
         printf( "loopClosureEnabled:       %d\n", vslamPara.loopClosureEnabled );
      }
      else if( itemName.compare( "targetHomography" ) == 0 )
      {
         for( size_t i = 0; i < sizeof( vslamPara.targetHomography ) / sizeof( vslamPara.targetHomography[0] ); i++ )
            iss >> vslamPara.targetHomography[i];
      }
   }

   //Sanity check
   if( vslamPara.initMode == VSLAMInitMode::TARGET_INIT )
   {
      if( vslamPara.targetImagePath[0] == 0 )
         printf( "Error!  Lack of target image path!\n" );
      if( vslamPara.targetWidth < 0.0f || vslamPara.targetHeight < 0.0f )
         printf( "Error!  Invalid target size!\n" );
   }
   if( vslamPara.initMode == VSLAMInitMode::RELOCALIZATION )
   {
      if( vslamPara.mapPath[0] == 0 )
         printf( "Error!  Lack of map path!\n" );
   }

   return 0;
}

int ParseCameraParameters( const char * parameterFile, VSLAMCameraParams & eagleCameraParams )
{
   std::string fullName = Program_Root + parameterFile;
   std::ifstream cfg( fullName.c_str(), std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", fullName.c_str() );
      return -1;
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
      if( itemName.compare( "ExposureTime" ) == 0 )
      {
         iss >> eagleCameraParams.exposure;
         printf( "Default exposure time = %f\n", eagleCameraParams.exposure );

      }
      else if( itemName.compare( "Gain" ) == 0 )
      {
         iss >> eagleCameraParams.gain;
         printf( "Default gain = %f\n", eagleCameraParams.gain );
      }
      else if( itemName.compare( "CaptureMode" ) == 0 )
      {
         std::string captureMode;
         iss >> captureMode;
         if( captureMode.compare( "PREVIEW" ) == 0 )
         {
            eagleCameraParams.captureMode = VSLAMCameraParams::PREVIEW;
            printf( "Capture mode:  Preview \n" );
         }
         else
         {
            eagleCameraParams.captureMode = VSLAMCameraParams::VIDEO;
            printf( "Capture mode:  Video \n" );
         }
      }
      else if( itemName.compare( "FrameRate" ) == 0 )
      {
         iss >> eagleCameraParams.frameRate;
         printf( "Set camera capture frame rate to = %f\n", eagleCameraParams.frameRate );
      }
      else if( itemName.compare( "InputFrameSkip" ) == 0 )
      {
         iss >> eagleCameraParams.skipFrame;
         eagleCameraParams.skipFrame++;
         printf( "Output only 1 of every %d frames\n", eagleCameraParams.skipFrame );
      }
      else if( itemName.compare( "FrameType" ) == 0 )
      {
         std::string frameType;
         iss >> frameType;
         if( frameType.compare( "RAW_FORMAT" ) == 0 )
         {
            eagleCameraParams.inputFormat = VSLAMCameraParams::RAW_FORMAT;
            printf( "Camera input format: RAW\n" );
         }
         else if( frameType.compare( "YUV_FORMAT" ) == 0 )
         {
            eagleCameraParams.inputFormat = VSLAMCameraParams::YUV_FORMAT;
            printf( "Camera input format: YUV\n" );
         }
         else
         {
            eagleCameraParams.inputFormat = VSLAMCameraParams::NV12_FORMAT;
            printf( "Camera input format: NV12\n" );
         }
      }
      else if( itemName.compare( "CorrectUpsideDown" ) == 0 )
      {
         iss >> eagleCameraParams.correctUpsideDown;
         if( eagleCameraParams.correctUpsideDown )
         {
            printf( "Rotate the image 180 degree. \n" );
         }
         else
         {
            printf( "Do not rotate the image. \n" );
         }
      }
      else if( itemName.compare( "CPAMode" ) == 0 )
      {
         std::string cpaMode;
         iss >> cpaMode;
         if( cpaMode.compare( "CPA_MODE_DISABLED" ) == 0 )
         {
            eagleCameraParams.useCPA = 0;
            printf( "CPA disabled\n" );
         }
         else if( cpaMode.compare( "CPA_MODE_COST" ) == 0 )
         {
            eagleCameraParams.cpaConfiguration.cpaType = MVCPA_MODE_COST;
            eagleCameraParams.useCPA = 1;
            printf( "CPA enabled\n" );
         }
         else if( cpaMode.compare( "CPA_MODE_HISTOGRAM" ) == 0 )
         {
            eagleCameraParams.cpaConfiguration.cpaType = MVCPA_MODE_HISTOGRAM;
            eagleCameraParams.useCPA = 1;
            printf( "CPA enabled\n" );
         }
      }
      else if( itemName.compare( "CPAExposureMin" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.histogram.exposureMin;
         printf( "CPA min exporsure %f\n", eagleCameraParams.cpaConfiguration.histogram.exposureMin );
      }
      else if( itemName.compare( "CPAExposureSoftMax" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.histogram.exposureSoftMax;
         printf( "CPA soft-max exporsure %f\n", eagleCameraParams.cpaConfiguration.histogram.exposureSoftMax );
      }
      else if( itemName.compare( "CPAExposureMax" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.histogram.exposureMax;
         printf( "CPA max exporsure %f\n", eagleCameraParams.cpaConfiguration.histogram.exposureMax );
      }
      else if( itemName.compare( "CPAGainMin" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.histogram.gainMin;
         printf( "CPA min gain %f\n", eagleCameraParams.cpaConfiguration.histogram.gainMin );
      }
      else if( itemName.compare( "CPAGainSoftMax" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.histogram.gainSoftMax;
         printf( "CPA soft-max gain %f\n", eagleCameraParams.cpaConfiguration.histogram.gainSoftMax );
      }
      else if( itemName.compare( "CPAGainMax" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.histogram.gainMax;
         printf( "CPA max gain %f\n", eagleCameraParams.cpaConfiguration.histogram.gainMax );
      }
      else if( itemName.compare( "CPALogEGPStepSizeMin" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.histogram.logEGPStepSizeMin;
         printf( "CPA min step %f\n", eagleCameraParams.cpaConfiguration.histogram.logEGPStepSizeMin );
      }
      else if( itemName.compare( "CPALogEGPStepSizeMax" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.histogram.logEGPStepSizeMax;
         printf( "CPA max step %f\n", eagleCameraParams.cpaConfiguration.histogram.logEGPStepSizeMax );
      }
      else if( itemName.compare( "ExposureCost" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.legacyCost.exposureCost;
         printf( "CPA exposure cost = %f\n", eagleCameraParams.cpaConfiguration.legacyCost.exposureCost );
      }
      else if( itemName.compare( "GainCost" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.legacyCost.gainCost;
         printf( "CPA gain cost = %f\n", eagleCameraParams.cpaConfiguration.legacyCost.gainCost );
      }
      else if( itemName.compare( "CPAFilterSize" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.legacyCost.filterSize;
         printf( "CPA filter Size = %d\n", eagleCameraParams.cpaConfiguration.legacyCost.filterSize );
      }
      else if( itemName.compare( "enableHistogramCost" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.legacyCost.enableHistogramCost;
         printf( "CPA enableHistogramCost = %d\n", eagleCameraParams.cpaConfiguration.legacyCost.enableHistogramCost );
      }
      else if( itemName.compare( "systemBrightnessMargin" ) == 0 )
      {
         iss >> eagleCameraParams.cpaConfiguration.legacyCost.systemBrightnessMargin;
         printf( "CPA systemBrightnessMargin = %f\n", eagleCameraParams.cpaConfiguration.legacyCost.systemBrightnessMargin );
      }
      else if( itemName.compare( "thresholdSaturated" ) == 0 )
      {
         unsigned int temp;
         iss >> temp;
         eagleCameraParams.cpaConfiguration.legacyCost.thresholdSaturated = temp > 255 ? 255 : temp;
         printf( "CPA thresholdSaturated = %d\n", eagleCameraParams.cpaConfiguration.legacyCost.thresholdSaturated );
      }
      else if( itemName.compare( "thresholdUnderflowed" ) == 0 )
      {
         unsigned int temp;
         iss >> temp;
         if( temp >= eagleCameraParams.cpaConfiguration.legacyCost.thresholdSaturated )
         {
            printf("Error: please make sure thresholdUnderflowed < thresholdSaturated!\n");
            exit(0);
         }
         eagleCameraParams.cpaConfiguration.legacyCost.thresholdUnderflowed = temp;
         printf( "CPA thresholdUnderflowed = %d\n", eagleCameraParams.cpaConfiguration.legacyCost.thresholdUnderflowed );
      }
      else if( itemName.compare( "VSLAMFrameWidth" ) == 0 )
      {
         iss >> eagleCameraParams.outputPixelWidth;
         printf( "undistorted image  Width = %d\n", eagleCameraParams.outputPixelWidth );
      }
      else if( itemName.compare( "VSLAMFrameHeight" ) == 0 )
      {
         iss >> eagleCameraParams.outputPixelHeight;
         printf( "undistorted image  height = %d\n", eagleCameraParams.outputPixelHeight );
      }
      else if( itemName.compare( "Camera" ) == 0 )
      {
         std::string cameraID;
         iss >> cameraID;
         GetCameraParameter( eagleCameraParams, cameraID.c_str() );
         printf( "Using camera ID:       %s\n", cameraID.c_str() );
      }
      else if( itemName.compare( "CameraFunc" ) == 0 )
      {
         std::string cameraFunc;
         iss >> cameraFunc;
         if( cameraFunc.compare( "CAM_FUNC_OPTIC_FLOW" ) == 0 )
         {
            eagleCameraParams.func = VSLAMCameraParams::CAM_FUNC_OPTIC_FLOW;
            printf( "Using CAM_FUNC_OPTIC_FLOW camera function\n" );
         }
         else
         {
            printf( "Camera func not found, using default fnc: CAM_FUNC_OPTIC_FLOW" );
         }
      }
   }
   return 0;
}


