#ifndef _mm_pdoran_camera_h_
#define _mm_pdoran_camera_h_

#include <atomic>
#include <thread>

class Camera
{
public:
  Camera();
  void start();
  void stop();

private:
  void init();
  void deinit();

  void run();

  std::atomic<bool> _isRunning;
  std::thread _thread;
};

#endif