#ifndef _mm_pdoran_camera_h_
#define _mm_pdoran_camera_h_

#include <memory>

class Camera
{
public:
  Camera();
  ~Camera();
  void start();
  void stop();

private:
  class Impl;
  std::unique_ptr<Impl> _impl;
};

#endif