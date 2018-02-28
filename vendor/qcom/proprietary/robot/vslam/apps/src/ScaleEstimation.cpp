/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "ScaleEstimation.h"
#include "mvVWSLAM_app.h"
#include <Visualization.h>
#include <cmath>
#include <numeric>
#include <algorithm>

ScaleEstimator gScaleEstimator;
extern std::string Program_Root;
extern class Visualiser * visualiser;

void eulerToSO3( const float* euler, float * rotation )
{
   float cr = cos( euler[0] );
   float sr = sin( euler[0] );
   float cp = cos( euler[1] );
   float sp = sin( euler[1] );
   float cy = cos( euler[2] );
   float sy = sin( euler[2] );
   rotation[0 * 3 + 0] = cy*cp;
   rotation[0 * 3 + 1] = cy*sp*sr - sy*cr;
   rotation[0 * 3 + 2] = cy*sp*cr + sy*sr;
   rotation[1 * 3 + 0] = sy*cp;
   rotation[1 * 3 + 1] = sy*sp*sr + cy*cr;
   rotation[1 * 3 + 2] = sy*sp*cr - cy*sr;
   rotation[2 * 3 + 0] = -sp;
   rotation[2 * 3 + 1] = cp*sr;
   rotation[2 * 3 + 2] = cp*cr;
}


void eulerFromSO3( const float *rotation, float * euler )
{
   if( fabs( rotation[2 * 3 + 0] < 1 ) )
   {
      euler[1] = -asin( rotation[2 * 3 + 0] );
      float scale = 1.F / cos( euler[1] );
      euler[0] = atan2( rotation[2 * 3 + 1] * scale, rotation[2 * 3 + 2] * scale );
      euler[2] = atan2( rotation[1 * 3 + 0] * scale, rotation[0 * 3 + 0] * scale );
   }
   else
   {
      euler[0] = atan2( rotation[2 * 3 + 1], rotation[2 * 3 + 2] );
      euler[1] = rotation[2 * 3 + 0] < 0 ? (float)AR_PI_OVER_2_D : (float)-AR_PI_OVER_2_D;
      euler[2] = 0.F;
   }
}


float32_t getOriDist( float32_t ori1, float32_t ori2 )
{
   float32_t dist = ori1 - ori2;

   if( dist < -2 * PI + 0.0005 )
   {
      printf( "error value for the effective range of angle!\n" );
   }
   else if( dist < -1 * PI )
   {
      dist += (float)(2 * PI);
   }
   else if( dist < PI )
   {
      ;
   }
   else if( dist <= 2 * PI )
   {
      dist = (float)(2 * PI - dist);
   }
   else
   {
      printf( "error value for the effective range of angle!\n" );
   }

   return dist;
}
  

void ScaleEstimator::setScaleEstimationStatus( ScaleEstimationStatus status )
{
   std::lock_guard<std::mutex> lk( mut );
   scaleEstimationStatus = status;
}


// dst = src1 * src2
void matricComputation( float32_t *src1, float32_t *src2, float32_t * dst )
{
   for( int m = 0; m < 9; m++ )
   {
      int i = m % 3;
      int j = m - i * 3;

      dst[i * 3 + j] = 0;
      for( int k = 0; k < 3; k++ )
      {
         dst[i * 3 + j] += (src1[i * 3 + k] * src2[k * 3 + j]);
      }
   }
}

// dst = rot*(src1-src2)
void translationComputation( float32_t *rot, float32_t *src1, float32_t *src2, float32_t * dst )
{
   float32_t diff[3];
   for( int i = 0; i < 3; i++ )
   {
      diff[i] = src1[i] - src2[2];
   }

   for( int i = 0; i < 3; i++ )
   {
      dst[i] = 0;
      for( int k = 0; k < 3; k++ )
      {
         dst[i] += (rot[i * 3 + k] * diff[i]);
      }
   }
}

// rotation matrix determination
void rotationMetrixDetermination()
{
   ;
}

float32_t trace( float32_t * rotationMetrix )
{
   return rotationMetrix[0] + rotationMetrix[4] + rotationMetrix[8];
}

// angle conversion (from rotation matrix to axis-angle)
void rotationMetrix2axisAngle( float32_t * rotationMetrix, float32_t * axisAngle )
{
   float32_t a0 = rotationMetrix[7] - rotationMetrix[5];
   float32_t a1 = rotationMetrix[2] - rotationMetrix[6];
   float32_t a2 = rotationMetrix[3] - rotationMetrix[1];

   float32_t uMagnitude = sqrt( a0*a0 + a1*a1 + a2*a2 );
   float32_t sinTheta = uMagnitude / 2.0f;
   float32_t cosTheta = (trace( rotationMetrix ) - 1) / 2.0f;
   float32_t theta = atan2( sinTheta, cosTheta );

   float32_t scale = theta / uMagnitude;
   axisAngle[0] = a0 * scale;
   axisAngle[1] = a1 * scale;
   axisAngle[2] = a2 * scale;
}


ScaleEstimationStatus ScaleEstimator::getScaleEstimationStatus()
{
   std::lock_guard<std::mutex> lk( mut );
   return scaleEstimationStatus;
}

void ScaleEstimator::setReinitTransformAndScalar()
{
   bool result = estimateScale( 0.04f );
   failScaleFlag = !result;

   wePoseQ.clear();
   vslamPoseQ.clear();
}


#include <iostream>
#include <fstream>
struct framePose
{
   int64_t timestamp;
   int32_t quality;
   float32_t location[3];
   float32_t rotation[9];
};

void invMatrix( float32_t * data, float32_t * inv )
{
   float32_t det;
   det = (data[0] * (data[4] * data[8] - data[5] * data[7]) +
           data[1] * (data[5] * data[6] - data[3] * data[8]) +
           data[2] * (data[3] * data[7] - data[4] * data[6]));
   if( det == 0.0f )
      return;

   float32_t mult;
   mult = 1.0f / det;
   inv[0] = (data[4] * data[8] - data[5] * data[7]) * mult;
   inv[1] = (data[2] * data[7] - data[1] * data[8]) * mult;
   inv[2] = (data[1] * data[5] - data[2] * data[4]) * mult;
   inv[3] = (data[5] * data[6] - data[3] * data[8]) * mult;
   inv[4] = (data[0] * data[8] - data[2] * data[6]) * mult;
   inv[5] = (data[2] * data[3] - data[0] * data[5]) * mult;
   inv[6] = (data[3] * data[7] - data[4] * data[6]) * mult;
   inv[7] = (data[1] * data[6] - data[0] * data[7]) * mult;
   inv[8] = (data[0] * data[4] - data[1] * data[3]) * mult;
}

void multiVector( float *M, float* v, float *vv )
{
   vv[0] = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
   vv[1] = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
   vv[2] = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
}

void multiMatrix( float32_t * left, float32_t * right, float32_t *result )
{
   result[0] = left[0] * right[0] + left[1] * right[3] + left[2] * right[6];
   result[1] = left[0] * right[1] + left[1] * right[4] + left[2] * right[7];
   result[2] = left[0] * right[2] + left[1] * right[5] + left[2] * right[8];
   result[3] = left[3] * right[0] + left[4] * right[3] + left[5] * right[6];
   result[4] = left[3] * right[1] + left[4] * right[4] + left[5] * right[7];
   result[5] = left[3] * right[2] + left[4] * right[5] + left[5] * right[8];
   result[6] = left[6] * right[0] + left[7] * right[3] + left[8] * right[6];
   result[7] = left[6] * right[1] + left[7] * right[4] + left[8] * right[7];
   result[8] = left[6] * right[2] + left[7] * right[5] + left[8] * right[8];
}

struct Point3D
{
   float32_t value[3];
};


void copyVSLAMMatrixData( mvWEFPoseStateTime pose, float32_t * rotation, float32_t * translation )
{
   rotation[0] = pose.poseWithState.pose.matrix[0][0];
   rotation[1] = pose.poseWithState.pose.matrix[0][1];
   rotation[2] = pose.poseWithState.pose.matrix[0][2];
   rotation[3] = pose.poseWithState.pose.matrix[1][0];
   rotation[4] = pose.poseWithState.pose.matrix[1][1];
   rotation[5] = pose.poseWithState.pose.matrix[1][2];
   rotation[6] = pose.poseWithState.pose.matrix[2][0];
   rotation[7] = pose.poseWithState.pose.matrix[2][1];
   rotation[8] = pose.poseWithState.pose.matrix[2][2];

   translation[0] = pose.poseWithState.pose.matrix[0][3];
   translation[1] = pose.poseWithState.pose.matrix[1][3];
   translation[2] = pose.poseWithState.pose.matrix[2][3];
}


void copyWheelMatrixData( mvWEFPoseVelocityTime pose, float32_t * rotation, float32_t * translation )
{
   float32_t euler[3];
   euler[0] = pose.pose.euler[0];
   euler[1] = pose.pose.euler[1];
   euler[2] = pose.pose.euler[2];

   eulerToSO3( euler, rotation );
   translation[0] = pose.pose.translation[0];
   translation[1] = pose.pose.translation[1];
   translation[2] = pose.pose.translation[2];
}


void addVector( float32_t * pose1, float32_t * pose2, float32_t * sum )
{
   sum[0] = pose1[0] + pose2[0];
   sum[1] = pose1[1] + pose2[1];
   sum[2] = pose1[2] + pose2[2];
}


void addVector( float32_t * pose1, float32_t * pose2, float32_t scale, float32_t * sum )
{
   sum[0] = pose1[0] + pose2[0] * scale;
   sum[1] = pose1[1] + pose2[1] * scale;
   sum[2] = pose1[2] + pose2[2] * scale;
}


//#define debug_tranformation_flag
bool ScaleEstimator::determineTransform()
{
   bool result = false;
   float32_t tempR[9];
   float32_t tempT[3];

#ifdef debug_tranformation_flag
   FILE * transformDebug = fopen( "keyPoiont.txt", "wb" );
#endif

   // Check the data and at leaset should have two
   if( vslamPoseQ.size() < 2 || wePoseQ.size() < 2 )
      return result;

   // Determine the transform from latest wheel pose to the lastWheelPose
   // Pw1wk = Pw1w0*Pw0wk
   // Rw1wk = Rw1w0*Rw0wk
   // tw1wk = tw1w0 + Rw1w0*tw0wk = -Rw1w0*tw0w1 + Rw1w0*tw0wk
   mvWEFPoseVelocityTime PoseW0W1 = lastwheelPose; //wePoseQ[0];
   mvWEFPoseVelocityTime PoseW0Wk = wePoseQ[wePoseQ.size() - 1];
   float32_t Rw0w1[9];
   float32_t Rw0wk[9];
   float32_t Rw1w0[9];
   float32_t Tw0w1[3];
   float32_t Tw0wk[3];

   float32_t Rw1wk[9];
   float32_t Tw1wk[3];
   copyWheelMatrixData( PoseW0W1, Rw0w1, Tw0w1 );
   copyWheelMatrixData( PoseW0Wk, Rw0wk, Tw0wk );
   invMatrix( Rw0w1, Rw1w0 );
   multiMatrix( Rw1w0, Rw0wk, Rw1wk );

   float32_t Tw1w0[3];
   multiVector( Rw1w0, Tw0w1, Tw1w0 );
   Tw1w0[0] = -1.0f * Tw1w0[0];
   Tw1w0[1] = -1.0f * Tw1w0[1];
   Tw1w0[2] = -1.0f * Tw1w0[2];
   multiVector( Rw1w0, Tw0wk, tempT );
   addVector( Tw1w0, tempT, Tw1wk );

#ifdef debug_tranformation_flag 
   fprintf( transformDebug, "Pw0w1 = " );
   float32_t *T = Tw0w1;
   float32_t *R = Rw0w1;
   for( int i = 0; i < 3; i++ )
   {
      fprintf( transformDebug, "%f ", T[i] );
   }
   for( int i = 0; i < 9; i++ )
   {
      fprintf( transformDebug, "%f ", R[i] );
   }
   fprintf( transformDebug, "\n" );


   fprintf( transformDebug, "Pw0wk (time:microsecond) %lld ", PoseW0Wk.timestampUs );
   T = Tw0wk;
   R = Rw0wk;
   for( int i = 0; i < 3; i++ )
   {
      fprintf( transformDebug, "%f ", T[i] );
   }
   for( int i = 0; i < 9; i++ )
   {
      fprintf( transformDebug, "%f ", R[i] );
   }
   fprintf( transformDebug, "\n" );
#endif

#ifdef debug_tranformation_flag
   float32_t debug_Rw0w1_euler[3];
   eulerFromSO3( Rw0w1, debug_Rw0w1_euler ); // debug
   printf( "Pw0w1: %f %f %f %f %f %f\n", Tw0w1[0], Tw0w1[1], Tw0w1[2], debug_Rw0w1_euler[0], debug_Rw0w1_euler[1], debug_Rw0w1_euler[2] );
   float32_t debug_Rw0wk_euler[3];
   eulerFromSO3( Rw0wk, debug_Rw0wk_euler ); // debug
   printf( "Pw0wk: %f %f %f %f %f %f\n", Tw0wk[0], Tw0wk[1], Tw0wk[2], debug_Rw0wk_euler[0], debug_Rw0wk_euler[1], debug_Rw0wk_euler[2] );
   float32_t debug_Rw1wk_euler[3];
   eulerFromSO3( Rw1wk, debug_Rw1wk_euler ); // debug
   printf( "Pw1wk: %f %f %f %f %f %f\n", Tw1wk[0], Tw1wk[1], Tw1wk[2], debug_Rw1wk_euler[0], debug_Rw1wk_euler[1], debug_Rw1wk_euler[2] );
#endif

   // Determine the transform from latest vslam pose to the lastGoodvslamPose
   // Pw0v1 = Pw0W1*Pwv
   // Rw0v1 = Rw0w1*Rwv
   // Tw0v1 = Tw0w1+Rw0w1*Tw1v1 = Tw0w1+Rw0w1*Twv = Tw0w1+Rw0w1*(-Rwv*Tvw)   
   float32_t RvwinEuler[3];// = { 0.0565476299,-1.0169635365,-1.5850496780 };
   float32_t Tvw[3];// = { -0.0182023867, -0.0544309457, -0.1556299096 };
   RvwinEuler[0] = mPoseVBr[0]; RvwinEuler[1] = mPoseVBr[1]; RvwinEuler[2] = mPoseVBr[2];
   Tvw[0] = mPoseVBt[0]; Tvw[1] = mPoseVBt[1]; Tvw[2] = mPoseVBt[2];
   float32_t Rvw[9];
   float32_t Rwv[9];
   float32_t tempTw[3];
   float32_t Rw0v1[9], Tw0v1[3];
   eulerToSO3( RvwinEuler, Rvw );
   invMatrix( Rvw, Rwv );
   multiVector( Rwv, Tvw, tempTw );
   tempTw[0] = -1.0f*tempTw[0];
   tempTw[1] = -1.0f*tempTw[1];
   tempTw[2] = -1.0f*tempTw[2];

   multiMatrix( Rw0w1, Rwv, Rw0v1 );
   multiVector( Rw0w1, tempTw, tempT );
   addVector( Tw0w1, tempT, Tw0v1 );

   // Pw0vk = Pw0Wk*Pwv
   // Rw0vk = Rw0wk*Rwv
   // Tw0vk = Tw0wk+Rw0wk*Twkvk = Tw0wk+Rw0wk*Twv = Tw0wk+Rw0wk*(-Rwv*Tvw)
   float32_t Rw0vk[9], Tw0vk[3];
   multiMatrix( Rw0wk, Rwv, Rw0vk );
   multiVector( Rw0wk, tempTw, tempT ); addVector( Tw0wk, tempT, Tw0vk );

   // Rv1vk = Rv1w0*Rw0vk = inv(Rw0w1*Rwv)*Rw0wk*Rwv = Rvw*Rw1w0*Rw0wk*Rwv = Rvw*Rw1wk*Rwv 
   // Tv1vk = Tv1w0+Rv1w0*Tw0vk = -Rv1w0*Tw0v1+Rv1w0*Tw0vk = Rv1w0*(Tw0vk-Tw0v1)
   float32_t Rv1vk[9];
   float32_t Tv1vk[3];
   multiMatrix( Rvw, Rw1wk, tempR );
   multiMatrix( tempR, Rwv, Rv1vk );
   addVector( Tw0vk, Tw0v1, -1.0, tempT );
   float32_t Rv1w0[9]; invMatrix( Rw0v1, Rv1w0 ); multiVector( Rv1w0, tempT, Tv1vk );

#ifdef debug_tranformation_flag

   float32_t debug_Rw0v1_euler[3];
   eulerFromSO3( Rw0v1, debug_Rw0v1_euler ); // debug
   printf( "Pw0v1: %f %f %f %f %f %f\n", Tw0v1[0], Tw0v1[1], Tw0v1[2], debug_Rw0v1_euler[0], debug_Rw0v1_euler[1], debug_Rw0v1_euler[2] );

   float32_t debug_Rw0vk_euler[3];
   eulerFromSO3( Rw0vk, debug_Rw0vk_euler ); // debug
   printf( "Pw0vk: %f %f %f %f %f %f\n", Tw0vk[0], Tw0vk[1], Tw0vk[2], debug_Rw0vk_euler[0], debug_Rw0vk_euler[1], debug_Rw0vk_euler[2] );

   float32_t debug_Rv1vk_euler[3];
   eulerFromSO3( Rv1vk, debug_Rv1vk_euler ); // debug
   printf( "Pv1vk: %f %f %f %f %f %f\n", Tv1vk[0], Tv1vk[1], Tv1vk[2], debug_Rv1vk_euler[0], debug_Rv1vk_euler[1], debug_Rv1vk_euler[2] );
#endif

   // Transform the latest vslam pose to the predefined coordination
   // Psvk = Psv1*Pv1vk
   // Rsvk = Rsv1*Rv1vk
   // Tsvk = Tsv1+Rsv1*Tv1vk  
   float32_t Rsv1[9];
   float32_t Tsv1[3];
   float32_t Rsvk[9];
   float32_t Tsvk[3];
   mvWEFPoseStateTime PoseSv1 = lastGoodvslamPose; // vslamPoseQ[0];
   copyVSLAMMatrixData( PoseSv1, Rsv1, Tsv1 );
   multiMatrix( Rsv1, Rv1vk, Rsvk );
   multiVector( Rsv1, Tv1vk, tempT );
   addVector( Tsv1, tempT, Tsvk );

#ifdef debug_tranformation_flag
   mvWEFPoseVelocityTime gtPose; // this variable is used for debugging
   float32_t debug_Rsvk_euler[3];
   eulerFromSO3( Rsvk, debug_Rsvk_euler ); // debug
   printf( "Psvk: %f %f %f %f %f %f\n", Tsvk[0], Tsvk[1], Tsvk[2], debug_Rsvk_euler[0], debug_Rsvk_euler[1], debug_Rsvk_euler[2] );
#endif

   // SV1 V1VK V0VK->SV0
   // Compute the transform to the API for transform the targetless map
   // Psv0 = Psvk*Pvkv0
   // Rsv0 = Rsvk*Rvkv0     
   // Tsv0 = Tsvk+Rsvk*Tvkv0 = Tsvk+Rsvk*(-Rvkv0*Tv0vk) = Tsvk - Rsv0*Tv0vk
   float32_t Rv0vk[9];
   float32_t Tv0vk[3];
   float32_t Rvkv0[9];
   float32_t Rsv0[9];
   float32_t Tsv0[3];

   // Determine based on the computed Psvk
   mvWEFPoseStateTime Posev0vk = vslamPoseQ[vslamPoseQ.size() - 1];
   copyVSLAMMatrixData( Posev0vk, Rv0vk, Tv0vk );
   Tv0vk[0] *= scale; Tv0vk[1] *= scale; Tv0vk[2] *= scale;
   invMatrix( Rv0vk, Rvkv0 );
   multiMatrix( Rsvk, Rvkv0, Rsv0 );

   multiVector( Rsv0, Tv0vk, tempT );
   addVector( Tsvk, tempT, -1.0, Tsv0 );

   // Rotation matrix to axis-angle conversion  
   //Start pose of the scale estimation in the VSLAM coordination 
   float32_t mPoseVWt[3];
   float32_t mPoseVWr[3];
   rotationMetrix2axisAngle( Rsv0, mPoseVWr );
   mPoseVWt[0] = Tsv0[0]; mPoseVWt[1] = Tsv0[1]; mPoseVWt[2] = Tsv0[2];

   // Save the determined value to mPoseSP
   mPoseSP[0][0] = Rsv0[0];
   mPoseSP[0][1] = Rsv0[1];
   mPoseSP[0][2] = Rsv0[2];
   mPoseSP[1][0] = Rsv0[3];
   mPoseSP[1][1] = Rsv0[4];
   mPoseSP[1][2] = Rsv0[5];
   mPoseSP[2][0] = Rsv0[6];
   mPoseSP[2][1] = Rsv0[7];
   mPoseSP[2][2] = Rsv0[8];

   mPoseSP[0][3] = mPoseVWt[0];
   mPoseSP[1][3] = mPoseVWt[1];
   mPoseSP[2][3] = mPoseVWt[2];
#ifdef debug_tranformation_flag
   fclose( transformDebug );
   printf( "mPoseVWt: %f %f %f\n", mPoseVWt[0], mPoseVWt[1], mPoseVWt[2] );
   printf( "mPoseVWr: %f %f %f\n", mPoseVWr[0], mPoseVWr[1], mPoseVWr[2] );
#endif

   result = true;
   return result;
}

float32_t distWe( mvWEFPoseVelocityTime pose1, mvWEFPoseVelocityTime  pose2)
{
   float32_t dist = sqrt( (pose1.pose.translation[0] - pose2.pose.translation[0]) * (pose1.pose.translation[0] - pose2.pose.translation[0])
                                          + (pose1.pose.translation[1] - pose2.pose.translation[1]) * (pose1.pose.translation[1] - pose2.pose.translation[1])
                                          + (pose1.pose.translation[2] - pose2.pose.translation[2]) * (pose1.pose.translation[2] - pose2.pose.translation[2]) );
   return dist;
}

float32_t distVSLAM( mvWEFPoseStateTime pose1, mvWEFPoseStateTime  pose2 )
{
   float32_t x1 = pose1.poseWithState.pose.matrix[0][3];
   float32_t y1 = pose1.poseWithState.pose.matrix[1][3];
   float32_t z1 = pose1.poseWithState.pose.matrix[2][3];
    
   float32_t x2 = pose2.poseWithState.pose.matrix[0][3];
   float32_t y2 = pose2.poseWithState.pose.matrix[1][3];
   float32_t z2 = pose2.poseWithState.pose.matrix[2][3];

   float32_t dist = sqrt( (x1 - x2) * (x1 - x2)
                          + (y1 - y2) * (y1 - y2)
                          + (z1 - z2) * (z1 - z2) );
   return dist;
}

ScaleVerificationStatus ScaleEstimator::scaleVerification()
{
   ScaleVerificationStatus status = ScaleVerificationStatus::ScaleVerificationOngoing;
   if( wePoseQ.size() < 2 )
   {
      return status;
   }
  
   mvWEFPoseVelocityTime firstWheelPose = wePoseQ[0];
   mvWEFPoseVelocityTime lastWheelPose = wePoseQ[wePoseQ.size() - 1];
    
   mvWEFPoseStateTime firstVSLAMPose = vslamPoseQ[0];
   mvWEFPoseStateTime lastVSLAMPose = vslamPoseQ[vslamPoseQ.size() - 1];

   float32_t weDist = distWe( firstWheelPose , lastWheelPose );
   float32_t vslamDist = distVSLAM( firstVSLAMPose, lastVSLAMPose );

   if( weDist > scaleVerificationV.smallDistThreshold && !scaleVerificationV.isVerifiedSmall)
   { 
      scaleVerificationV.isVerifiedSmall = true;

      float32_t localScale = (float32_t)(weDist / vslamDist);
      float32_t tmp = fabs( localScale / scale - 1.0f );
      if( fabs( localScale / scale - 1.0 ) > scaleVerificationV.scaleRatioThreshold )
      {
         //scaleVerificationV.failTimes++;
      }
      else
      {
         scaleVerificationV.passTimes++;
      }

      printf( "SMALL: scaleVerificationV.failCnt = %d scaleVerificationV.passCnt = %d verScale = %f estScale = %f scaleRatio = %f\n",
              scaleVerificationV.failTimes, scaleVerificationV.passTimes, localScale, scale, tmp );

      std::string dataUsage = "smallScaleVerification";
      visualiser->RecordPoseForScaleEstimation( vslamPoseQ, wePoseQ, dataUsage );
      printf( "%s\n", dataUsage.c_str() ); 
   }
   else if( weDist > scaleVerificationV.largeDistThreshold )
   {
      float32_t localScale = (float32_t)(weDist / vslamDist);
      float32_t tmp = fabs( localScale / scale - 1.0f );
      if( fabs( localScale / scale - 1.0 ) > scaleVerificationV.scaleRatioThreshold )
      {
         scaleVerificationV.failTimes++;
      }
      else
      {
         scaleVerificationV.passTimes++;
      }

      printf( "LARGER: scaleVerificationV.failCnt = %d scaleVerificationV.passCnt = %d verScale = %f estScale = %f scaleRatio = %f\n",
              scaleVerificationV.failTimes, scaleVerificationV.passTimes, localScale, scale, tmp );

      std::string dataUsage = "largeScaleVerification";
      visualiser->RecordPoseForScaleEstimation( vslamPoseQ, wePoseQ, dataUsage );
      printf( "%s\n", dataUsage.c_str() );
      //wePoseQ.clear();
      //vslamPoseQ.clear();
   }
      
   if( (scaleVerificationV.failTimes + scaleVerificationV.passTimes) >= scaleVerificationV.verfiNum )
   {
      if(scaleVerificationV.passTimes > scaleVerificationV.failTimes )
         status = ScaleVerificationStatus::ScaleVerificationPass;
      else
         status = ScaleVerificationStatus::ScaleVerificationFail; 
   }

   //if( weDist > scaleVerificationV.largeDistThreshold )
   //{
   //   float32_t localScale = (float32_t)(weDist / vslamDist);
   //   float32_t tmp = fabs( localScale / scale - 1.0 );
   //   if( fabs( localScale / scale - 1.0 ) > scaleVerificationV.scaleRatioThreshold )
   //   {
   //      scaleVerificationV.failTimes++; 
   //   }
   //   else
   //   {
   //      scaleVerificationV.passTimes++; 
   //   }

   //   printf("scaleVerificationV.failCnt = %d scaleVerificationV.passCnt = %d verScale = %f estScale = %f scaleRatio = %f\n", 
   //           scaleVerificationV.failTimes, scaleVerificationV.passTimes, localScale, scale, tmp);

   //   std::string dataUsage = "scaleVerification";
   //   visualiser->RecordPoseForScaleEstimation( vslamPoseQ, wePoseQ, dataUsage );
   //   printf("%s\n", dataUsage);
   //   wePoseQ.clear();
   //   vslamPoseQ.clear();
   //}

   //if( (scaleVerificationV.failTimes + scaleVerificationV.passTimes) >= scaleVerificationV.verfiNum )
   //{
   //   if(scaleVerificationV.passTimes > scaleVerificationV.failTimes )
   //      status = ScaleVerificationStatus::ScaleVerificationPass;
   //   else
   //      status = ScaleVerificationStatus::ScaleVerificationFail; 
   //}
    
   return status; 
}

bool ScaleEstimator::estimateScale( const float minDist2 )
{
   mvWEFPoseStateTime vslamPose1, vslamPose2;
   mvWEFPoseVelocityTime wePose1, wePose2;

   double weDistance = 0;
   double vslamDistance = 0;
   // processing the vslam pose data
   bool hasPoseFlag = vslamPoseQ.size() > 1;
   if( hasPoseFlag == false )
      return false;

   // processing the wheel pose data
   hasPoseFlag = wePoseQ.size() > 1;
   if( hasPoseFlag == false )
      return false;

#ifdef debug_file_
   // for debug
   FILE * debugDist = fopen( "dist.txt", "wb" );
   fprintf( debugDist, "lastGoodVslamPose %lld %f %f %f\n", gScaleEstimator.lastGoodvslamPose.timestampUs,
            gScaleEstimator.lastGoodvslamPose.poseWithState.pose.matrix[0][3],
            gScaleEstimator.lastGoodvslamPose.poseWithState.pose.matrix[1][3],
            gScaleEstimator.lastGoodvslamPose.poseWithState.pose.matrix[2][3] );

   fprintf( debugDist, "lastGoodWheelPose %lld %f %f %f\n", gScaleEstimator.lastwheelPose.timestampUs,
            gScaleEstimator.lastwheelPose.pose.translation[0],
            gScaleEstimator.lastwheelPose.pose.translation[1],
            gScaleEstimator.lastwheelPose.pose.translation[2] );

   for( int i = 0; i < wePoseQ.size(); i++ )
   {
      fprintf( debugDist, "wePosexyz %lld %f %f %f\n", wePoseQ[i].timestampUs, wePoseQ[i].pose.translation[0],
               wePoseQ[i].pose.translation[1], wePoseQ[i].pose.translation[2] );
   }
   for( int i = 0; i < vslamPoseQ.size(); i++ )
   {
      fprintf( debugDist, "vslamPosexyz %lld %f %f %f\n", vslamPoseQ[i].timestampUs, vslamPoseQ[i].poseWithState.pose.matrix[0][3],
               vslamPoseQ[i].poseWithState.pose.matrix[1][3], vslamPoseQ[i].poseWithState.pose.matrix[2][3] );
   }
   fclose( debugDist );
#endif

   if( 2 > vslamPoseQ.size() || 2 > wePoseQ.size() )
   {
      return false;
   }

   if( true )
   {
      std::string dataUsage = "scaleEstimation";
      visualiser->RecordPoseForScaleEstimation( vslamPoseQ, wePoseQ, dataUsage );
      posePair* posePairQ = new posePair[vslamPoseQ.size()];

      unsigned int posePairQLen = 0;
      bool successScaleCalFlag = mvWEF_EstimateScale( (mvWEFPoseStateTime*)&(*vslamPoseQ.begin()), vslamPoseQ.size(), (mvWEFPoseVelocityTime*)&(*wePoseQ.begin()), wePoseQ.size(), minDist2, &scale, posePairQ, &posePairQLen );
      bool successTransformCalFlag = true;
      if( successScaleCalFlag )
      {
         if( gScaleEstimator.lastGoodvslamPose.timestampUs != -1 )
         {
            vslamPoseQ.clear();
            wePoseQ.clear();

            vslamPoseQ.push_back( posePairQ[0].pose );
            wePoseQ.push_back( posePairQ[0].wePose );
            vslamPoseQ.push_back( posePairQ[posePairQLen - 1].pose );
            wePoseQ.push_back( posePairQ[posePairQLen - 1].wePose );
            successTransformCalFlag = determineTransform();
         }
         else
         {
            mPoseSP[0][0] = 1.0;
            mPoseSP[0][1] = 0.0;
            mPoseSP[0][2] = 0.0;
            mPoseSP[1][0] = 0.0;
            mPoseSP[1][1] = 1.0;
            mPoseSP[1][2] = 0.0;
            mPoseSP[2][0] = 0.0;
            mPoseSP[2][1] = 0.0;
            mPoseSP[2][2] = 1.0;

            mPoseSP[0][3] = 0.0;
            mPoseSP[1][3] = 0.0;
            mPoseSP[2][3] = 0.0;

            successTransformCalFlag = true;
         }
      }
      delete[] posePairQ;

      if( successTransformCalFlag && successScaleCalFlag )
         return true;
      else
         return false;
   }
   else
   {
      std::vector<mvWEFPoseStateTime> vslamPoseQfiter;
      std::vector<mvWEFPoseVelocityTime> wePoseQfiter;
      std::vector<float> scaleLowLevel;

      for( size_t cnt = 0; cnt < vslamPoseQ.size(); cnt++ )
      {
         size_t k = 0;
         bool isFoundFlag = false;
         int64_t curVSLAMtime = vslamPoseQ[cnt].timestampUs;
         for( ; k < wePoseQ.size(); k++ )
         {
            int64_t curWEtime = wePoseQ[k].timestampUs;
            int64_t timeDelta = curVSLAMtime - curWEtime;
            if( timeDelta < 50000 && timeDelta > -50000 ) // in microsecond
            {
               isFoundFlag = true;
               break;
            }
         }

         if( isFoundFlag )
         {
            vslamPoseQfiter.push_back( vslamPoseQ[cnt] );
            wePoseQfiter.push_back( wePoseQ[k] );
         }
      }

      wePoseQ.clear();
      vslamPoseQ.clear();
      wePoseQ = wePoseQfiter;
      vslamPoseQ = vslamPoseQfiter;

      wePose1 = wePoseQ[0];
      wePose2 = wePoseQ[wePoseQ.size() - 1];
      weDistance = 0;
      for( int i = 0; i < 3; i++ )
      {
         double coordinateDelta = wePose1.pose.translation[i] - wePose2.pose.translation[i];
         weDistance += coordinateDelta*coordinateDelta;
      }
      weDistance = sqrt( weDistance );

      vslamPose1 = vslamPoseQ[0];
      vslamPose2 = vslamPoseQ[vslamPoseQ.size() - 1];
      vslamDistance = 0;
      for( int i = 0; i < 3; i++ )
      {
         double coordinateDelta = vslamPose1.poseWithState.pose.matrix[i][3] - vslamPose2.poseWithState.pose.matrix[i][3];
         vslamDistance += coordinateDelta*coordinateDelta;
      }
      vslamDistance = sqrt( vslamDistance );

      if( vslamDistance != 0 )
      {
         scale = (float32_t)(weDistance / vslamDistance);
      }

      bool successTransformCalFlag = true;
      if( gScaleEstimator.lastGoodvslamPose.timestampUs != -1 )
      {
         successTransformCalFlag = determineTransform();
      }
      else
      {
         mPoseSP[0][0] = 1.0;
         mPoseSP[0][1] = 0.0;
         mPoseSP[0][2] = 0.0;
         mPoseSP[1][0] = 0.0;
         mPoseSP[1][1] = 1.0;
         mPoseSP[1][2] = 0.0;
         mPoseSP[2][0] = 0.0;
         mPoseSP[2][1] = 0.0;
         mPoseSP[2][2] = 1.0;

         mPoseSP[0][3] = 0.0;
         mPoseSP[1][3] = 0.0;
         mPoseSP[2][3] = 0.0;

         successTransformCalFlag = true;
      }
      if( vslamDistance != 0 && successTransformCalFlag )
         return true;
      else
         return false;
   }
}
