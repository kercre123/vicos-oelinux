#ifndef _mm_pdoran_util_h_
#define _mm_pdoran_util_h_

#include <string>

extern "C" {
#include "mm_qcamera_app.h"
}

namespace anki
{

void dump_frame(mm_camera_buf_def_t* frame, const std::string& path);

} /* namespace anki */

#endif