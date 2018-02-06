/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "Motion.h"
#include "WEF.h"
#include "ScaleEstimation.h"
#include "mvVWSLAM_app.h"
#ifndef WIN32
#include "math.h"
#endif

queue_mt<MOTION_PATTERN> gMotionPatternQueue;
using namespace MoveAlongPathPatternSpace;

/**--------------------------------------------------------------------------
@brief
   Queue for wheel encoder pose, vslam pose and fusion pose
--------------------------------------------------------------------------**/
extern queue_mt<mvWEFPoseVelocityTime> gMotionWEPoseQueue;
extern queue_mt<mvWEFPoseStateTime> gMotionVSLAMPoseQueue;
extern queue_mt<mvPose6DRT> gMotionFsuionPoseQueue;

Pose3D getFusionPose()
{
   Pose3D tmpPose = { (float32_t)0.0, 0.0, 0.0 };
   mvPose6DRT fusionPose;
   gMotionFsuionPoseQueue.wait_and_pop( fusionPose );
   {
      //printf( "\n%lld\n", fusionPose.timestampUs );
      tmpPose.x = fusionPose.matrix[0][3];
      tmpPose.y = fusionPose.matrix[0][1];

      float rotation[9] = { fusionPose.matrix[0][0],fusionPose.matrix[0][1],fusionPose.matrix[0][2],
         fusionPose.matrix[1][0],fusionPose.matrix[1][1],fusionPose.matrix[1][2],
         fusionPose.matrix[2][0],fusionPose.matrix[2][1],fusionPose.matrix[2][2]
      };
      float euler[3];
      eulerFromSO3( rotation, euler );
      tmpPose.yaw = euler[2];
   }

   return tmpPose;
}


Pose3D getVSLAMPose( MV_VSLAM_TRACKING_STATE &poseState )
{
   Pose3D tmpPose = { (float32_t)0.0, 0.0, 0.0 };
   mvWEFPoseStateTime vslamPose;
   gMotionVSLAMPoseQueue.wait_and_pop( vslamPose );
   {
      float32_t t[3];
      float32_t r[3][3];

      t[0] = vslamPose.poseWithState.pose.matrix[0][3];
      t[1] = vslamPose.poseWithState.pose.matrix[1][3];
      t[2] = vslamPose.poseWithState.pose.matrix[2][3];

      for( int32_t row = 0; row < 3; row++ )
      {
         for( int32_t col = 0; col < 3; col++ )
         {
            r[row][col] = vslamPose.poseWithState.pose.matrix[row][col];
         }
      }

      tmpPose.x = t[0];
      tmpPose.y = t[1];
      tmpPose.yaw = t[2]; // need to update it

      poseState = vslamPose.poseWithState.poseQuality;
   }

   return tmpPose;
}


Pose3D getWheelEncoderPose()
{
   Pose3D tmpPose = { (float32_t)0.0, 0.0, 0.0 };
   mvWEFPoseVelocityTime wePose;
   gMotionWEPoseQueue.wait_and_pop( wePose );
   {
      tmpPose.x = wePose.pose.translation[0];
      tmpPose.y = wePose.pose.translation[1];
      tmpPose.yaw = wePose.pose.euler[2];
   }

   return tmpPose;
}


void setMotionPattern( MOTION_PATTERN motionPattern )
{
   if( motionPattern == MOTION_PATTERN::EXIT_PATTERN )
   {
      return;
   }

   //motionPattern = LOOP_CLOSURE_PATTERN;

   MoveAlongPathPattern pathPattern;
   string pathPatternFileName;

   switch( motionPattern )
   {
      case TARGET_INIT_PATTERN:
         pathPatternFileName = "pathPattern4TargtInit.txt";
         break;
      case TARGETLESS_INIT_PATTERN:
         pathPatternFileName = "pathPattern4TargetLessInit.txt";
         break;
      case SCALE_ESTIMATION_PATTERN:
         pathPatternFileName = "pathPattern4ScaleEstimation.txt";
         break;
      case LOOP_CLOSURE_PATTERN:
         pathPatternFileName = "pathPattern4LoopClosure.txt";
         break;
      default:
         printf( "In the function fo setMotionPattern, should not call the DEFAULT case!\n" );
         break;
   }

   pathPattern.qPathPatternFile = pathPatternFileName;
   Pose3D startPos;
   bool isGetFirstPose = false;
   while( !isGetFirstPose && THREAD_RUNNING )
   {
      // This should be the fusion pose (fused or wheel only) in the nav coordination
      if( gScaleEstimator.lastGoodvslamPose.timestampUs == -1 )
         startPos = getWheelEncoderPose();
      else
         startPos = getFusionPose();
      if( startPos.x != 0.0 || startPos.y != 0.0 || startPos.yaw != 0.0 )
      {
         isGetFirstPose = true;
         printf( "Get the first pose when loading the path pattern!\n" );
      }
   }
   if( !THREAD_RUNNING )
   {
      return;
   }
   pathPattern.loadPathPattern( startPos );

   static Velocity outputSpeed = { 0.0, 0.0, 0.0 };
   bool isFinishedFlag = false;
   int failFrameCnt_TargetlessInit = 0;
   switch( motionPattern )
   {
      case TARGET_INIT_PATTERN:
         printf( "In the case of TARGET_INIT_PATTERN case to get a VSLAM pose!\n" );
         break;
      case TARGETLESS_INIT_PATTERN:
         printf( "In the case of TARGETLESS_INIT_PATTERN case to get a VSLAM pose!\n" );
         while( !isFinishedFlag && THREAD_RUNNING )
         {

            if( gScaleEstimator.poseQuality == MV_VSLAM_TRACKING_STATE_GREAT ||
                gScaleEstimator.poseQuality == MV_VSLAM_TRACKING_STATE_GOOD ||
                gScaleEstimator.poseQuality == MV_VSLAM_TRACKING_STATE_OK )
            {
               isFinishedFlag = true;
            }
            else
            {
               failFrameCnt_TargetlessInit++;
               if( failFrameCnt_TargetlessInit > 300 )
               {
                  gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );
                  break;
               }
            }

            VSLAM_MASTER_SLEEP( 30 );
#ifdef MoveRobotAlongGivenPath
            Pose3D fusionPose = getFusionPose();
            while( !(fusionPose.x != 0.0 || fusionPose.y != 0.0 || fusionPose.yaw != 0.0) )
            {
               fusionPose = getFusionPose();
            }
            printf( "x = %f y = %f yaw = %f\n", fusionPose.x, fusionPose.y, fusionPose.yaw );

            isFinishedFlag = pathPattern.runPathPattern( fusionPose, outputSpeed );

            if( !isFinishedFlag )
            {
               Pose3D vslamPose = getVSLAMPose();
               if( !(vslamPose.x == 0.0 && vslamPose.y == 0.0 && vslamPose.yaw == 0.0) )
               {
                  isFinishedFlag = true;
               }
            }
#endif
         }

         if( !THREAD_RUNNING )
         {
            return;
         }
         break;
      case SCALE_ESTIMATION_PATTERN:
         printf( "In the case of SCALE_ESTIMATION_PATTERN and would goto LOOP_CLOSURE_PATTERN to give out the cmd_el!\n" );
      case LOOP_CLOSURE_PATTERN:
         printf( "In the case of LOOP_CLOSURE_PATTERN!\n" );
#ifndef MoveRobotAlongGivenPath
         int cnt;
         cnt = 0;
         Pose3D startPose, endPose;
         startPose.x = 0.0; startPose.y = 0.0; startPose.yaw = 0.0;
         endPose.x = 0.0; endPose.y = 0.0; endPose.yaw = 0.0;
#endif
         while( !isFinishedFlag && THREAD_RUNNING )
         {
#ifdef MoveRobotAlongGivenPath
            //isFinishedFlag = pathPattern.runPathPattern(fusionPose, outputSpeed);
#else
            MV_VSLAM_TRACKING_STATE poseState;
            Pose3D vlsamPose = getVSLAMPose( poseState );
            //         while( !(vlsamPose.x != 0.0 || vlsamPose.y != 0.0 || vlsamPose.yaw != 0.0) )
                  ////while (!(vlsamPose.x != 0.0 || vlsamPose.y != 0.0 || vlsamPose.yaw != 0.0))
            //         {
            //            vlsamPose = getVSLAMPose(poseState);
            //         }

                     // Compute the distance
            Pose3D wePose = getWheelEncoderPose();
            while( !(wePose.x != 0.0 || wePose.y != 0.0 || wePose.yaw != 0.0) )
            {
               wePose = getWheelEncoderPose();
            }

            float32_t distSquare = 0.0;
            if( startPose.x == 0.0 && startPose.y == 0.0 &&startPose.yaw == 0.0 )
               startPose = wePose;
            else
            {
               endPose = wePose;

               Pose3D diffPose = endPose - startPose;
               distSquare = diffPose.x * diffPose.x + diffPose.y * diffPose.y;
            }

            static int failFrameCnt = 0;

            bool isVSLAMPoseHighQuality = MV_VSLAM_TRACKING_STATE_GREAT == poseState
               || MV_VSLAM_TRACKING_STATE_GOOD == poseState
               || MV_VSLAM_TRACKING_STATE_OK == poseState;
            if( !isVSLAMPoseHighQuality )
            {
               failFrameCnt++;
            }
            else
            {
               failFrameCnt = 0;
            }

            if( failFrameCnt > 100 && cnt != 0 )
            {
               isFinishedFlag = true;
               gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );
               failFrameCnt = 0;
               printf( "Stop the current motion pattern for scale estimation for continuing failing to get a good VSLAM pose!\n" );
            }

            if( isVSLAMPoseHighQuality )
            {
               cnt++;
               if( cnt > 150 && distSquare > 1.f )
               {
                  isFinishedFlag = true;
                  gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SCALE_ESTIMATOR );
                  printf( "Finish the path pattern by CNT method for PC debugging!\n" );
               }
            }
#endif
         }
         break;
      default:
         printf( "In the function of setMotionPattern, should not call the DEFAULT case!\n" );
         break;
   }
}

