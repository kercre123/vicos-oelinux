#include "wrapper.h"

extern "C" {
#include "mm_camera_interface.h"
}

uint8_t my_get_num_of_cameras()
{
  return get_num_of_cameras();
}