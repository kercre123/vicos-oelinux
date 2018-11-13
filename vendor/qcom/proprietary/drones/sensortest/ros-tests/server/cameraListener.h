#include "camera.h"
#include "camera_parameters.h"
#include <iostream>
#include <pthread.h>
#include <queue>
#include <boost/thread/mutex.hpp>
#include "logTypes.h"

using namespace camera;

class CameraTest : public ICameraListener
{
public:

  CameraTest();
  ~CameraTest();
  
  int initialize(int camId);
  
  /* listener methods */
  virtual void onError();
  virtual void onPreviewFrame(ICameraFrame* frame);
  virtual void onVideoFrame(ICameraFrame* frame);
  virtual void onPictureFrame(ICameraFrame* frame);
  bool insertFrame(ICameraFrame* frame);
  ICameraFrame* getNextFrame();
  static int dumpToFile(uint8_t* data, uint32_t size, char* name, uint64_t timestamp);
private:
  ICameraDevice* camera_;
  CameraParams params_;
  pthread_t listenerThread;
  std::queue<ICameraFrame*> frames;
  ICameraFrame* lastDequeuedFrame;
  boost::mutex mtx;
  static const int MAX_FRAME_BUFFERS = 50;
  bool publisherReady;
};

CameraTest::CameraTest() :
  camera_(NULL),
  lastDequeuedFrame(0),
  publisherReady(false)
{}

CameraTest::~CameraTest()
{
  if (camera_) {
    camera_->stopPreview();
  }
  if (lastDequeuedFrame) {
    lastDequeuedFrame->releaseRef();
  }
}

int CameraTest::initialize(int camId)
{
  int rc = EXIT_FAILURE;
  int n = getNumberOfCameras();
  if (n < 0) {
    std::cout << "getNumberOfCameras() failed, rc= " << n << std::endl;
    return EXIT_FAILURE;
  }
  if (n < 1) {
    printf("No cameras found.\n");
    return EXIT_FAILURE;
  }
  
  std::cout << "Number of Cameras = " << n <<  std::endl;
  std::cout << "Using camera id: " << camId << std::endl;
  
  rc = ICameraDevice::createInstance(camId, &camera_);
  if (rc != 0) {
    std::cout << "Could not open camera " << camId << ". rc = " << rc << std::endl;
    return rc;
  }
  camera_->addListener(this);
  
  rc = params_.init(camera_);
  if (rc != 0) {
    std::cout << "Failed to init parameters\n";
    ICameraDevice::deleteInstance(&camera_);
    return rc;
  }
  printf("params = %s\n", params_.toString().c_str());
  std::cout << "start preview\n";
  camera_->startPreview();
  sleep(5);
  return EXIT_SUCCESS;
}


void CameraTest::onPreviewFrame(ICameraFrame* frame)
{
  // printf("%s:%d F_ts = %lld exposure time = %lld gain = %d \n",__func__,__LINE__,frame->timeStamp, \
  // 	 params_.getFrameExposureTime(frame),				\
  // 	 params_.getFrameGainValue(frame));
  if (!publisherReady) {
    ROS_INFO_STREAM("onPreviewFrame: Publisher not ready. Dropping");
    return;
  }
  std::stringstream ss;
  ss << frame->timeStamp;
  ROSLogs::addLog("onPreviewFrame", ss.str());
  frame->acquireRef();
  insertFrame(frame);
}

bool CameraTest::insertFrame(ICameraFrame* frame) {
  ROS_DEBUG_STREAM("Inserting frame with FD: " << frame->fd << ". Addr: " << frame << ". Vector length: " << frames.size());
  // if (frames.size() >= MAX_FRAME_BUFFERS) {
  //   std::cout << "--------Buffer size exceeded. Removing a frame\n";
  //   getNextFrame();
  // }
  mtx.lock();
  frames.push(frame);
  mtx.unlock();
  std::stringstream ss;
  ss << frame->timeStamp;
  ROSLogs::addLog("PreviewFrameInserted", ss.str());
}

ICameraFrame* CameraTest::getNextFrame() {
  ICameraFrame* f = 0;
  ICameraFrame* frameToRelease = 0;
  publisherReady = true;
  mtx.lock();
  if (frames.size() == 0) {
    mtx.unlock();
    return 0;
  }
  f = frames.front();
  ROS_DEBUG_STREAM("In getNextFrame: Got front frame: " << f);
  frames.pop();
  if (lastDequeuedFrame != 0) {
    frameToRelease = lastDequeuedFrame;
  }
  lastDequeuedFrame = f;
  ROS_DEBUG_STREAM("In getNextFrame: Set lastDequeuedFrame to " << lastDequeuedFrame);
  mtx.unlock();
  if (frameToRelease) {
    ROS_DEBUG_STREAM("Releasing Ref on frame with FD: " << frameToRelease->fd << ". Addr: " << frameToRelease);
    frameToRelease->releaseRef();
  }
  ROS_DEBUG_STREAM("Popped frame with FD: " << f->fd << ". Addr: " << f << ". Vector length: " << frames.size());
  return f;
}
  
void CameraTest::onPictureFrame(ICameraFrame* frame)
{
  return;
}
void CameraTest::onVideoFrame(ICameraFrame* frame)
{
  return;
}

void CameraTest::onError()
{
  std::cout << "Camera error!" << std::endl;
  exit(EXIT_FAILURE);
}

int CameraTest::dumpToFile(uint8_t* data, uint32_t size, char* name, uint64_t timestamp)
{
  FILE* fp;
  fp = fopen(name, "wb");
  if (!fp) {
    std::cout << "fopen failed for " << name;
    return -1;
  }
  fwrite(data, size, 1, fp);
  std::cout << "Saved frame to file: " <<  name << std::endl;
  fclose(fp);
  return 0;
}
