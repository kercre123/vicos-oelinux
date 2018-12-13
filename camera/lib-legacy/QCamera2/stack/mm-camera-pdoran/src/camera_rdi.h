#ifndef _mm_pdoran_camera_rdi_h_
#define _mm_pdoran_camera_rdi_h_

#include <memory>

#include "camera.h"

namespace anki
{

/**
 * @brief SBGGR10P Camera
 */
class CameraRDI : public Camera
{
public:
  CameraRDI();
  ~CameraRDI();
  
protected:
  void onStart() override;
  void onStop() override;

private:
  class Impl;
  std::unique_ptr<Impl> _impl;
};

} /* namespace anki */

#endif