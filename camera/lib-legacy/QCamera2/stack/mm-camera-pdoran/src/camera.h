#ifndef _mm_pdoran_camera_h_
#define _mm_pdoran_camera_h_

#include <string>

namespace anki
{

/**
 * @brief Camera Interface
 */
class Camera
{
public:
  class Params{
  public:
    Params();
    bool dump;
    std::string directory;
  };

  Camera();
  virtual ~Camera();
  void start();
  void stop();

  void setParams(const Params& params);
  const Params& getParams() const;

protected:
  virtual void onStart() = 0;
  virtual void onStop() = 0;

  Params _params;

};

} /* namespace anki */

#endif