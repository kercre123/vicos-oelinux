#include "application.h"

#include "camera.h"

// TODO: Remove, these are for debugging experiments
#include <thread>
#include <chrono>
#include <iostream>

Application::Application()
  : _camera(nullptr)
{
}

Application::~Application()
{
}

int Application::exec(const Args& args)
{
  std::cerr<<"Hello, World!"<<std::endl;
  _camera = std::unique_ptr<Camera>(new Camera());
  _camera->start();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  _camera->stop();
  return 0;
}