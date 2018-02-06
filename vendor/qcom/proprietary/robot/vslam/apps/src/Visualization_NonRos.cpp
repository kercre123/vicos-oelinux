/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "Visualization_NonRos.h"
#include <VSLAM_internal.h>
#include "mvVSLAM.h"

Visualiser_NonRos::Visualiser_NonRos( const char * outputPath ): Visualiser(outputPath)
{
   std::string output( outputPath );
   fpLogWEFOutput = openLogFile( (output + "fusion_output.csv").c_str() );
   if( nullptr != fpLogWEFOutput )
      fprintf( fpLogWEFOutput, "timestampUs,x,y,z,roll,pitch,yaw,velocityLinear,velocityAngular\n" );

   fpLogVSLAM = openLogFile( (output + "vslam_output.csv").c_str() );

   fpLogVSLAMCorrected = openLogFile( (output + "vslam_corrected_output.csv").c_str() );

   

   if( nullptr != fpLogVSLAM )
      fprintf( fpLogVSLAM, "timestampUs,x,y,z\n" );
   if( nullptr != fpLogVSLAMCorrected )
      fprintf( fpLogVSLAMCorrected, "timestampUs,state,x,y,z,#PMatched,#PMisMtchd\n" );

}


Visualiser_NonRos:: ~Visualiser_NonRos()
{
   if( fpLogWEFOutput )
   {
      fclose( fpLogWEFOutput );
   }
   if( fpLogVSLAM )
   {
      fclose( fpLogVSLAM );
   }
   if( fpLogVSLAMCorrected )
   {
      fclose( fpLogVSLAMCorrected );
   }
}

void Visualiser_NonRos::ShowPoints( const mvWEFPoseStateTime & poseWithTime, const uint8_t * image, int imageWidth, int imageHeight )
{

#ifdef OPENCV_SUPPORTED
   const mvVSLAMTrackingPose & pose = poseWithTime.poseWithState;

   cv::Mat rview;
   GetVSLAMStatus( image, imageWidth, imageHeight );
   DrawLabelledImage( poseWithTime, image, imageWidth, imageHeight, rview );

   cv::imshow( "source", rview );
   cv::waitKey( 1 );
#endif //OPENCV_SUPPORTED
}

void Visualiser_NonRos::PublishRobotPose( const mvWEFPoseVelocityTime & fpose )
{
   if( nullptr != fpLogWEFOutput )
      fprintf( fpLogWEFOutput, "%lld,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f\n",
               fpose.timestampUs,
               fpose.pose.translation[0], fpose.pose.translation[1], fpose.pose.translation[2],
               fpose.pose.euler[0], fpose.pose.euler[1], fpose.pose.euler[2],
               fpose.velocityLinear, fpose.velocityAngular );
}

void Visualiser_NonRos::PublishOriginalImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight )
{
#ifdef OPENCV_SUPPORTED
   cv::Mat view;
   GetOriginalImage( image, view, imageWidth,  imageHeight);
   cv::imshow( "original image", view );
   cv::waitKey( 1 );
#endif //OPENCV_SUPPORTED
}

void Visualiser_NonRos::PublishUndistortedImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight )
{
}


void Visualiser_NonRos::PublishCameraPose( const mvWEFPoseStateTime & VSLAMPose, const uint8_t * image, int /*imageWidth*/, int /*imageHeight*/)
{
   if( nullptr != fpLogVSLAM )
      fprintf( fpLogVSLAM, "%lld,%.6f,%.6f,%.6f\n",
               VSLAMPose.timestampUs,
               VSLAMPose.poseWithState.pose.matrix[0][3],
               VSLAMPose.poseWithState.pose.matrix[1][3],
               VSLAMPose.poseWithState.pose.matrix[2][3] );
}

void Visualiser_NonRos::PublishCorrectedCameraPose( const mvWEFPoseStateTime & VSLAMPoseCorrected )
{
   if( nullptr != fpLogVSLAMCorrected )
   {
      vslamStatus status = GetStatus();
      fprintf( fpLogVSLAMCorrected, "%lld,%d,%.6f,%.6f,%.6f,%d,%d\n",
               VSLAMPoseCorrected.timestampUs,
               VSLAMPoseCorrected.poseWithState.poseQuality,
               VSLAMPoseCorrected.poseWithState.pose.matrix[0][3],
               VSLAMPoseCorrected.poseWithState.pose.matrix[1][3],
               VSLAMPoseCorrected.poseWithState.pose.matrix[2][3],
               status._MatchedMapPointNum, status._MisMatchedMapPointNum );
   }
}
