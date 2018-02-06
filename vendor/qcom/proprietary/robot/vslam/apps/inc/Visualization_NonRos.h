/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VISUALIZATION_NONROS_H__
#define __VISUALIZATION_NONROS_H__

#include <Visualization.h>
#include "mvWEF.h"

class Visualiser_NonRos: public Visualiser
{
public:
   Visualiser_NonRos(const char * outputPath);
   virtual ~Visualiser_NonRos();

   virtual void PublishOriginalImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight );
   virtual void PublishUndistortedImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight );
   virtual void ShowPoints( const mvWEFPoseStateTime & pose, const uint8_t * image, int imageWidth, int imageHeight );
   virtual void PublishRobotPose( const mvWEFPoseVelocityTime & pose );
   virtual void PublishCameraPose( const mvWEFPoseStateTime & pose, const uint8_t * image = NULL, int imageWidth = 0, int imageHeight = 0 );
   virtual void PublishCorrectedCameraPose( const mvWEFPoseStateTime & VSLAMPoseCorrected );
   virtual void PublishExposureGain( float32_t exposure, float32_t gain, int exposureValue, int gainValue, float mean_brightness )
   {};

protected:
   FILE *fpLogWEFOutput = nullptr;

   FILE *fpLogVSLAM = nullptr;
   FILE *fpLogVSLAMCorrected = nullptr;
   
   
};
#endif //__VISUALIZATION_H__
