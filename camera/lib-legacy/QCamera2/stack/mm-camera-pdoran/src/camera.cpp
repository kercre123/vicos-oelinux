#include "camera.h"

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
  int counter = 0;
  while (_isRunning)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout<<"Camera "<<(counter++)<<std::endl;
  }
}