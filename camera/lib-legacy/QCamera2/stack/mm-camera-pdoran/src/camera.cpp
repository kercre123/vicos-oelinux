#include "camera.h"

#include "wrapper.h"

// TODO: Remove, these are for debugging experiments
#include <chrono>
#include <iostream>

Camera::Camera()
{
}

void Camera::start()
{
  _thread = std::thread(&Camera::run, this);
  _isRunning = true;
}

void Camera::stop()
{
  _isRunning = false;
  _thread.join();
}

void Camera::run()
{
  uint8_t num_cameras = my_get_num_of_cameras();
  std::cerr<<"Num Cameras: "<<static_cast<uint32_t>(num_cameras)<<std::endl;

  int counter = 0;
  while (_isRunning)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout<<"Camera "<<(counter++)<<std::endl;
  }
}