#ifndef _mm_pdoran_camera_yuv_h_
#define _mm_pdoran_camera_yuv_h_

#include "camera.h"

#include <memory>

namespace anki
{

/**
 * @brief YUV Camera
 */
class CameraYUV : public Camera
{
public:
  CameraYUV();
  ~CameraYUV();

protected:
  void onStart() override;
  void onStop() override;

private:
  class Impl;
  std::unique_ptr<Impl> _impl;
};

} /* namespace anki */

#endif