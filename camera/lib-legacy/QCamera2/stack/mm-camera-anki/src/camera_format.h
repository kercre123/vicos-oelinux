/**
 * File: camera_format.h
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: camera formats supported by Anki/Victor
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __anki_camera_format_h__
#define __anki_camera_format_h__

typedef enum {
  ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10,
  ANKI_CAM_FORMAT_RAW = ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10,
  ANKI_CAM_FORMAT_RGB888,
  ANKI_CAM_FORMAT_YUV,
  ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10_2MP,
  ANKI_CAM_FORMAT_RAW_2MP = ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10_2MP,
  ANKI_CAM_FORMAT_RGB888_2MP,
  ANKI_CAM_FORMAT_YUV_2MP,
} anki_camera_pixel_format_t;

#endif // __anki_camera_format_h__
