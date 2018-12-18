#include "camera.h"

namespace anki {

Camera::Params::Params()
: dump(false)
, directory("")
{

}

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

void Camera::setParams(const Params& params)
{
  _params = params;
}
const Camera::Params& Camera::getParams() const
{
  return _params;
}

} /* namespace anki */