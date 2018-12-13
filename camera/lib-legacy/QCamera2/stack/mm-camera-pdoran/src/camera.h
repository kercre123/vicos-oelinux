#ifndef _mm_pdoran_camera_h_
#define _mm_pdoran_camera_h_

namespace anki
{

/**
 * @brief Camera Interface
 */
class Camera
{
public:
  Camera();
  virtual ~Camera();
  void start();
  void stop();

protected:
  virtual void onStart() = 0;
  virtual void onStop() = 0;

};

} /* namespace anki */

#endif