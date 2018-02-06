/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "Visualization.h"
#include <VSLAM_internal.h>
#include "mvVSLAM.h"
#include "math.h"

Visualiser::Visualiser( const char * outputPath )
{
   std::string output( outputPath );
   fpLogWEFInput = openLogFile( (output + "fusion_input.csv").c_str() );
}

Visualiser:: ~Visualiser()
{
   if( fpLogWEFInput )
   {
      fclose( fpLogWEFInput );
   }
}


void Visualiser::RecordWheelOdom( const mvWEFPoseVelocityTime & WEPose )
{
   if( nullptr != fpLogWEFInput )
      fprintf( fpLogWEFInput, "%d\t%lld\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n",
               kReadingWheel, WEPose.timestampUs,
               WEPose.pose.translation[0], WEPose.pose.translation[1], WEPose.pose.translation[2],
               WEPose.pose.euler[0], WEPose.pose.euler[1], WEPose.pose.euler[2],
               WEPose.velocityLinear, WEPose.velocityAngular );
}

void Visualiser::RecordVSLAMOdom( const mvWEFPoseStateTime & VSLAMPose, TokenFusionInput token )
{
   if( nullptr != fpLogWEFInput )
      fprintf( fpLogWEFInput, "%d\t%lld\t%d\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n",
               token, VSLAMPose.timestampUs, VSLAMPose.poseWithState.poseQuality,
               VSLAMPose.poseWithState.pose.matrix[0][0],
               VSLAMPose.poseWithState.pose.matrix[0][1],
               VSLAMPose.poseWithState.pose.matrix[0][2],
               VSLAMPose.poseWithState.pose.matrix[0][3],
               VSLAMPose.poseWithState.pose.matrix[1][0],
               VSLAMPose.poseWithState.pose.matrix[1][1],
               VSLAMPose.poseWithState.pose.matrix[1][2],
               VSLAMPose.poseWithState.pose.matrix[1][3],
               VSLAMPose.poseWithState.pose.matrix[2][0],
               VSLAMPose.poseWithState.pose.matrix[2][1],
               VSLAMPose.poseWithState.pose.matrix[2][2],
               VSLAMPose.poseWithState.pose.matrix[2][3] );
}

void Visualiser::RecordFusedPose()
{
   if( nullptr != fpLogWEFInput )
      fprintf( fpLogWEFInput, "%d\n", kGettingFPose );
}

FILE* Visualiser::openLogFile( const char* nameLogFile )
{
   FILE *fpLog = fopen( nameLogFile, "w" );
   if( nullptr == fpLog )
   {
      printf( "Cannot open file for writting: %s\n", nameLogFile );
   }
   else
   {
      printf( "Open file for writting: %s\n", nameLogFile );
   }

   return fpLog;
}

#ifdef OPENCV_SUPPORTED
#include <opencv2/opencv.hpp>
void Visualiser::DrawLabelledImage( const mvWEFPoseStateTime & poseWithTime, const uint8_t * image, int widthFrame, int heightFrame, cv::Mat & rview )
{
   rview = cv::Mat( heightFrame, widthFrame, CV_8UC1 );
   memcpy( rview.data, image, heightFrame* widthFrame );

   cv::cvtColor( rview, rview, CV_GRAY2BGR );
   //char fileName[200];
   //sprintf(fileName, "image_%lld.bmp", poseWithTime.timestampUs);
   //imwrite( fileName, rview );

   mvVSLAMTrackingPose pose = poseWithTime.poseWithState;
   mvVSLAM* pVSlamObj = parameter.pVSlamObj;

   int newObservNum = mvVSLAM_HasTrackedObservation( pVSlamObj );
   if( newObservNum > 0 )
   {
      MV_TrackedObservation* observBuf = new MV_TrackedObservation[newObservNum];
      newObservNum = mvVSLAM_GetObservation( pVSlamObj, observBuf, newObservNum );

      if( pose.poseQuality != MV_VSLAM_TRACKING_STATE_FAILED )
      {
         cv::Point2f imagePoint;
         for( int i = 0; i < newObservNum; i++ )
         {
            imagePoint.x = observBuf[i].x;
            imagePoint.y = observBuf[i].y;
            if( observBuf[i].s == MV_TrackedObservation::MATCHING_OK )
            {
               circle( rview, imagePoint, 4, cv::Scalar( 0, 255, 0 ) ); //green
               //printf("A good feature\n");
            }
            else
            {
               circle( rview, imagePoint, 4, cv::Scalar( 0, 0, 255 ) ); //red
               //printf("A bad feature\n");
            }

         }
      }
      delete[] observBuf;
   }

   //Mat rview0;
   //cv::flip( rview, rview0, -1 );
   //rview = rview0;

   GetVSLAMStatus( image, widthFrame, heightFrame );

   char strFrame[50];
   cv::Scalar color( 255, 0, 0 );
   if( MV_VSLAM_TRACKING_STATE_FAILED == poseWithTime.poseWithState.poseQuality ||
       MV_VSLAM_TRACKING_STATE_INITIALIZING == poseWithTime.poseWithState.poseQuality )
   {
      color = cv::Scalar( 0, 0, 255 );
   }
   else if( poseWithTime.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_BAD
       || poseWithTime.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_APPROX )
   {
      color = cv::Scalar( 0, 255, 255 );
   }
   else if( MV_VSLAM_TRACKING_STATE_RELOCATED == poseWithTime.poseWithState.poseQuality )
   {
      color = cv::Scalar( 0, 255, 0 );
   }
   sprintf( strFrame, "BR:%3d, VR: %3.1f, KF:%5d", (int32_t)status._BrightnessMean, status._BrightnessVar, status._KeyframeNum );
   putText( rview, std::string( strFrame ), cv::Point2f( 350, 450 ), cv::FONT_HERSHEY_COMPLEX, 0.6, color );
   sprintf( strFrame, "Mismatched: %3d, Matched: %3d", status._MisMatchedMapPointNum, status._MatchedMapPointNum );
   putText( rview, std::string( strFrame ), cv::Point2f( 250, 30 ), cv::FONT_HERSHEY_COMPLEX, 0.6, color );

}

void Visualiser::GetVSLAMStatus( const uint8_t * image, int widthFrame, int heightFrame )
{
   cv::Mat mean;
   cv::Mat stdDev;
   cv::Mat rview = cv::Mat( heightFrame, widthFrame, CV_8UC1 );
   memcpy( rview.data, image, heightFrame* widthFrame );
   cv::meanStdDev( rview, mean, stdDev );

   status._BrightnessMean = (float32_t)mean.at<double>( 0, 0 );
   status._BrightnessVar = (float32_t)stdDev.at<double>( 0, 0 );

   mvVSLAM* pVSlamObj = parameter.pVSlamObj;
   status._KeyframeNum = mvVSLAM_GetMapSize( pVSlamObj );

   int newObservNum = mvVSLAM_HasTrackedObservation( pVSlamObj );

   status._MatchedMapPointNum = 0;
   status._MisMatchedMapPointNum = 0;
   if( newObservNum > 0 )
   {
      MV_TrackedObservation* observBuf = new MV_TrackedObservation[newObservNum];
      newObservNum = mvVSLAM_GetObservation( pVSlamObj, observBuf, newObservNum );
    
      for( int i = 0; i < newObservNum; i++ )
      {
         if( observBuf[i].s == MV_TrackedObservation::MATCHING_OK )
         {
            status._MatchedMapPointNum++;
         }
         else
         {
            status._MisMatchedMapPointNum++;
         }
      }
      delete[] observBuf;
   }
}

void Visualiser::GetOriginalImage( const uint8_t * image, cv::Mat & view, uint32_t widthFrame, uint32_t heightFrame )
{
   view = cv::Mat( heightFrame, widthFrame, CV_8UC1 );

   uint32_t all = widthFrame * heightFrame;
   uint32_t iRaw = 0;
   uint32_t i = 0;
   uint8_t * dst = (uint8_t *)view.data;
   for( i = 0, iRaw = 0; i < all; i += 4, iRaw += 5 )
   {
      memcpy( dst + i, image + iRaw, 4 );
   }
}
#endif


void RtoQuaternion( const float32_t matrix[3][4], double quaternion[4] )
{
   float trace = matrix[0][0] + matrix[1][1] + matrix[2][2];

   if( trace > 0 )
   {
      double S = sqrt( trace + 1.0 ) * 2; // S=4*quaternion[0] 
      quaternion[0] = 0.25 * S;
      quaternion[1] = (matrix[2][1] - matrix[1][2]) / S;
      quaternion[2] = (matrix[0][2] - matrix[2][0]) / S;
      quaternion[3] = (matrix[1][0] - matrix[0][1]) / S;
   }
   else if( (matrix[0][0] > matrix[1][1])&(matrix[0][0] > matrix[2][2]) )
   {
      double S = sqrt( 1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2] ) * 2; // S=4*quaternion[1] 
      quaternion[0] = (matrix[2][1] - matrix[1][2]) / S;
      quaternion[1] = 0.25 * S;
      quaternion[2] = (matrix[0][1] + matrix[1][0]) / S;
      quaternion[3] = (matrix[0][2] + matrix[2][0]) / S;
   }
   else if( matrix[1][1] > matrix[2][2] )
   {
      double S = sqrt( 1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2] ) * 2; // S=4*quaternion[2]
      quaternion[0] = (matrix[0][2] - matrix[2][0]) / S;
      quaternion[1] = (matrix[0][1] + matrix[1][0]) / S;
      quaternion[2] = 0.25 * S;
      quaternion[3] = (matrix[1][2] + matrix[2][1]) / S;
   }
   else
   {
      double S = sqrt( 1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1] ) * 2; // S=4*quaternion[3]
      quaternion[0] = (matrix[1][0] - matrix[0][1]) / S;
      quaternion[1] = (matrix[0][2] + matrix[2][0]) / S;
      quaternion[2] = (matrix[1][2] + matrix[2][1]) / S;
      quaternion[3] = 0.25 * S;
   }
}

