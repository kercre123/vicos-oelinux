#include "util.h"

extern "C" {
#include "mm_qcamera_dbg.h"
}

namespace anki
{

void dump_frame(mm_camera_buf_def_t* frame, const std::string& path)
{
  int offset = 0;
  int file_fd;
  int j;
  file_fd = open(path.c_str(), O_RDWR | O_CREAT, 0777);
  if (file_fd < 0) {
      CDBG_ERROR("%s: cannot open file %s \n", __func__, path.c_str());
  } else {
    for (j = 0; j < frame->num_planes; j++) {
      CDBG("%s: saving file from address: %p, data offset: %d, "
            "length: %d \n", __func__, frame->buffer,
            frame->planes[j].data_offset, frame->planes[j].length);
      write(file_fd, (uint8_t *)frame->buffer + offset, frame->planes[j].length);
      offset += (int)frame->planes[j].length;
    }
    close(file_fd);
  }
}

} /* namespace anki */