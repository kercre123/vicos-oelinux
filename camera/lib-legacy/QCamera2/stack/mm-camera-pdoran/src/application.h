#ifndef _mm_pdoran_application_h_
#define _mm_pdoran_application_h_

#include <memory>

// Forward declarations
class Camera;

class Application
{
public:
  struct Args
  {
  };

  Application();
  virtual ~Application();
  int exec(const Args& args);

private:
  std::unique_ptr<Camera> _camera;
};

#endif