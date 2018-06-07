/**
 * File: camera_params.h
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: camera capture params available through server/process
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __anki_camera_params_h__
#define __anki_camera_params_h__

#include "camera_format.h"

struct anki_camera_capture_params {
  uint8_t fps_reduction;
  anki_camera_pixel_format_t pixel_format;
};


#endif // __anki_camera_params_h__
