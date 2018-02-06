/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__


#ifdef ROS_BASED
#define OPENCV_SUPPORTED
#endif

#ifdef WIN32
#define OPENCV_SUPPORTED
#endif

#include "WEF.h"

#ifdef OPENCV_SUPPORTED
#include <opencv2/opencv.hpp>
#endif

typedef struct _vslamStatus
{
   float32_t _BrightnessMean;
   float32_t _BrightnessVar;
   int32_t _KeyframeNum;
   int32_t _MatchedMapPointNum;
   int32_t _MisMatchedMapPointNum;

} vslamStatus;

enum TokenFusionInput
{
   kReadingVSLAM = 0,
   kReadingWheel,
   kGettingFPose,
   kScaleEsVSLAM
};

class Visualiser
{
public:
   Visualiser( const char * outputPath );
   virtual ~Visualiser();


   virtual void PublishOriginalImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight ) = 0;
   virtual void PublishUndistortedImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight ) = 0;
   virtual void ShowPoints( const mvWEFPoseStateTime & pose, const uint8_t * image, int imageWidth, int imageHeight ) = 0;
   virtual void PublishRobotPose( const mvWEFPoseVelocityTime & pose ) = 0;
   virtual void PublishCameraPose( const mvWEFPoseStateTime & pose, const uint8_t * image = NULL, int imageWidth = 0, int imageHeight = 0 ) = 0;
   virtual void PublishCorrectedCameraPose( const mvWEFPoseStateTime & VSLAMPoseCorrected ) = 0;
   virtual void PublishExposureGain( float32_t exposure, float32_t gain, int exposureValue, int gainValue, float mean_brightness ) = 0;

   virtual void RecordWheelOdom( const mvWEFPoseVelocityTime & WEPose );
   virtual void RecordVSLAMOdom( const mvWEFPoseStateTime & VSLAMPose, TokenFusionInput token );
   virtual void RecordFusedPose();
protected:

#ifdef OPENCV_SUPPORTED
   void DrawLabelledImage( const mvWEFPoseStateTime & pose, const uint8_t * image, int imageWidth, int imageHeight, cv::Mat & view );
   void GetOriginalImage( const uint8_t * image, cv::Mat & view, uint32_t widthFrame, uint32_t heightFrame );
   void GetVSLAMStatus( const uint8_t * image, int imageWidth, int imageHeight );
#endif
   FILE* openLogFile( const char* nameLogFile );
   vslamStatus GetStatus()
   {
      return status;
   }

   vslamStatus status;
   FILE *fpLogWEFInput = nullptr;
};

void RtoQuaternion( const float32_t matrix[3][4], double quaternion[4] );

#endif //__VISUALIZATION_H__
