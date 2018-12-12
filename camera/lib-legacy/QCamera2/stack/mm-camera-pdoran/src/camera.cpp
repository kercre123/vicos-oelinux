#include "camera.h"

#include <atomic>
#include <thread>

// TODO: Remove, these are for debugging experiments
#include <chrono>
#include <iostream>

extern "C" {
#include "mm_camera_interface.h"
#include <hardware/camera.h>
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

  for (uint32_t i = 0; i < num_cameras; ++i)
  {
    std::cerr<<"------------------------------------------------------------"<<std::endl;
    camera_info* info = get_cam_info(i);
    if (info == nullptr)
    {
      std::cerr<<"Info["<<i<<"]: null"<<std::endl;
    }
    else
    {
      std::cerr<<"Info["<<i<<"]: "<<std::endl;
      std::cerr<<"facing:                 "<<info->facing<<std::endl;
      std::cerr<<"orientation:            "<<info->orientation<<std::endl;
      std::cerr<<"device_version:         "<<info->device_version<<std::endl;
      // std::cerr<<"static_camera_characteristics: "<<std::endl; // TODO: camera_metadata
      std::cerr<<"resource_cost:          "<<info->resource_cost<<std::endl;
      std::cerr<<"conflicting_devices:    "<<std::endl;
      for (uint32_t j = 0; j < info->conflicting_devices_length; ++j)
      {
        std::cerr<<"    "<<info->conflicting_devices[j]<<std::endl;
      }
    }
    std::cerr<<"------------------------------"<<std::endl;
    mm_camera_vtbl_t* vtbl = camera_open(i);
    if (vtbl == nullptr)
    {
      std::cerr<<"VTBL["<<i<<"]: null"<<std::endl;
    }
    else
    {
      std::cerr<<"VTBL["<<i<<"]: "<<std::endl;
      std::cerr<<"handle:                 "<<vtbl->camera_handle<<std::endl;
#if 1
      {
        vtbl->ops->map_buf(vtbl->camera_handle, CAM_MAPPING_BUF_TYPE_CAPABILITY, fd, size)
        if (vtbl->query_capabaility(vtbl->camera_handle) == 0)
        {
          // success
        }
        else
        {
          // failure
        }
      }
#endif
      if (vtbl->ops->close_camera(vtbl->camera_handle) == 0)
      {
        // success
        std::cerr<<"CLOSED CAMERA["<<i<<"]"<<std::endl;
      }
      else
      {
        //failure
        std::cerr<<"FAILED TO CLOSE CAMERA["<<i<<"]"<<std::endl;
      }
    }
    std::cerr<<"------------------------------------------------------------"<<std::endl;
  }

  int counter = 0;
  while (_isRunning)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout<<"sleeping "<<(counter++)<<std::endl;
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