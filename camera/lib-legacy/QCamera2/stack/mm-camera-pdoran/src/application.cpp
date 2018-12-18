#include "application.h"

#include "camera_rdi.h"
#include "camera_yuv.h"
#include "event.h"

#include <csignal>

namespace anki {

//======================================================================================================================
// Signal and Global Exit Event setup
static Event gEvent;

static void handle_signal(int signum)
{
  gEvent.set();
}

static void setup_signal_handler()
{
  struct sigaction new_action, old_action;

  new_action.sa_handler = handle_signal;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;

  (void) sigaction(SIGINT, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN) {
    (void) sigaction(SIGINT, &new_action, NULL);
  }
  (void) sigaction(SIGHUP, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN) {
    (void) sigaction(SIGHUP, &new_action, NULL);
  }
  (void) sigaction(SIGTERM, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN) {
    (void) sigaction(SIGTERM, &new_action, NULL);
  }
}

//======================================================================================================================
// Application Implementation

Application::Application()
  : _camera(nullptr)
{
}

Application::~Application()
{
}

int Application::exec(const Args& args)
{
  // Setup signals for exiting
  setup_signal_handler();

  // Create and start a new camera
  if (args.camera == "RAW")
    _camera.reset(new CameraRDI());
  else if (args.camera == "YUV")
    _camera.reset(new CameraYUV());
  else
    return 1;

  Camera::Params params;
  params.dump = args.dump;
  params.directory = args.output;
  _camera->setParams(params);
  
  _camera->start();

  // Wait for an exit signal (e.g. CTRL+C)
  gEvent.wait();

  // Stop the camera
  _camera->stop();

  return 0;
}

} /* namespace anki */