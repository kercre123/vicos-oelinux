#ifndef _mm_pdoran_application_h_
#define _mm_pdoran_application_h_

#include <memory>

namespace anki
{

// Forward declarations
class Camera;

class Application
{
public:
  struct Args
  {
    /**
     * @brief Camera Type (RAW, YUV)
     */
    std::string camera;

    /**
     * @brief Output path
     */
    std::string output;

    /**
     * @brief Dump images
     */
    bool dump;
  };

  Application();
  virtual ~Application();
  int exec(const Args& args);

private:
  std::unique_ptr<Camera> _camera;
};

} /* namespace anki */

#endif