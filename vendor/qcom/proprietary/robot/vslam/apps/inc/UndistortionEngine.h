/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VSLAM_CAMERA_INTERFACE_H__
#define __VSLAM_CAMERA_INTERFACE_H__

#include "mv.h"

const char QLEVEL_0 = 5;
const uint16_t QLEVEL = (1 << QLEVEL_0);
const uint16_t QLEVEL_H = (QLEVEL >> 1);


struct indBL_t
{
   int indBL1;
   int indBL2;
   int indBL3;
   int indBL4;
};

//struct wind_t
//{
//   unsigned char wx;
//   unsigned char wy;
//};



class UndistortionEngine
{
public:

   UndistortionEngine();

   virtual ~UndistortionEngine();

   //virtual void setCaptureParams( const BlurCameraParams & params );
   bool init( float32_t cameraMatrix[9], float32_t distCoeffs[8], float32_t newCameraMatrix[9],
              int32_t inptWidth, int32_t inputHeight, int32_t outputPixelWidth, int outputPixelHeight );
   void undistort( unsigned char * __restrict src, unsigned char * __restrict dst );

protected:
   int32_t outputHeight;
   int32_t outputWidth;

   void RemapBL_2( unsigned char * __restrict src, unsigned char * __restrict dst, struct indBL_t * __restrict idx,
                   unsigned char * __restrict wx, unsigned char * __restrict wy, int w, int h );
   struct indBL_t * __restrict indBL;
   //struct wind_t  * __restrict wind;
   uint8_t * __restrict wx;
   uint8_t * __restrict wy;
   
   void PrepareRemap( float32_t cameraMatrix[9], float32_t distCoeffs[8], float32_t newCameraMatrix[9],
                      int32_t inptWidth, int32_t inputHeight, int32_t outputWidth, int32_t outputHeight );
   void GetRemapindexBL( float32_t * mapX, float32_t * mapY, int outputWidth, int32_t outputHeight, int32_t inputWidth, int32_t inputHeight, struct indBL_t *idx );
   void GetRemapindexBL_RawFormat( float32_t * mapX, float32_t * mapY, int outputWidth, int32_t outputHeight, int32_t inputWidth, int32_t inputHeight, struct indBL_t *idx );
   
};

#endif //__VSLAM_CAMERA_INTERFACE_H__
