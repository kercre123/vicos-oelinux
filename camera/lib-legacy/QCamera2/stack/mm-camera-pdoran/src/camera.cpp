#include "camera.h"

#include <atomic>
#include <thread>

// TODO: Remove, these are for debugging experiments
#include <chrono>
#include <iostream>

extern "C" {
#include "mm_camera_interface.h"
}

//======================================================================================================================
// Camera Private Implementation
class Camera::Impl
{
public:
  Impl();
  ~Impl();
  void start();
  void stop();

private:
  void run();

  std::atomic<bool> _isRunning;
  std::thread _thread;
};

Camera::Impl::Impl()
  : _isRunning(false)
  , _thread()
{
}

Camera::Impl::~Impl()
{
}

void Camera::Impl::start()
{
  _thread = std::thread(&Camera::Impl::run, this);
  _isRunning = true;
}

void Camera::Impl::stop()
{
  _isRunning = false;
  _thread.join();
}

void Camera::Impl::run()
{
  uint8_t num_cameras = get_num_of_cameras();
  std::cerr<<"Num Cameras: "<<static_cast<uint32_t>(num_cameras)<<std::endl;

  int counter = 0;
  while (_isRunning)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout<<"Camera "<<(counter++)<<std::endl;
  }
}

//======================================================================================================================
// Camera Public Implementation
Camera::Camera()
  : _impl(std::unique_ptr<Camera::Impl>(new Camera::Impl())) // std::make_unique requires c++14
{
}

Camera::~Camera()
{
}

void Camera::start() { _impl->start(); }
void Camera::stop() { _impl->stop(); }