#include "camera.h"

namespace anki {

Camera::Camera()
{
}

Camera::~Camera()
{
}

void Camera::start()
{
  onStart();
}

void Camera::stop()
{
  onStop();
}

} /* namespace anki */