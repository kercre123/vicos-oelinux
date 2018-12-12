#include "camera.h"

#include <atomic>
#include <thread>
#include <mutex>

extern "C" {
#include "mm_qcamera_app.h"
#include "mm_qcamera_dbg.h"
}

// TODO: Remove, these are for debugging experiments
#include <chrono>
#include <iostream>

namespace anki {

//======================================================================================================================
// Camera Private Implementation
class Camera::Impl
{
public:
  Impl();
  ~Impl();
  void start();
  void stop();

private:
  class Params{
  public:
    Params();
    
    uint8_t num_bufs;
  };
  
  void run();

  bool init();
  bool init_lib();
  bool init_channel();
  bool init_stream();
  void deinit();

  /**
   * @brief Static callback to provide to qualcomm; calls non-static callback
   */
  static void static_callback(mm_camera_super_buf_t *bufs, void *userdata);

  /**
   * @brief Non-Static callback for camera
   */
  void callback(mm_camera_super_buf_t *bufs);

  Params _params;
  
  std::atomic<bool> _isRunning;
  std::thread _thread;
  std::mutex _mutex;
  int _count;

  mm_camera_lib_handle _lib_handle;
  mm_camera_channel_t* _channel;
  mm_camera_stream_t* _stream;
};

Camera::Impl::Params::Params()
  : num_bufs(RDI_BUF_NUM) // from mm_qcamera_app.h
{

}

Camera::Impl::Impl()
  : _params() 
  , _isRunning(false)
  , _thread()
  , _lib_handle()
  , _channel(nullptr)
  , _stream(nullptr)
{
  memset(&_lib_handle,0,sizeof(mm_camera_lib_handle));
}

Camera::Impl::~Impl()
{
}

void Camera::Impl::start()
{
  _thread = std::thread(&Camera::Impl::run, this);
  _isRunning = true;
}

void Camera::Impl::stop()
{
  _isRunning = false;
  _thread.join();
}

void Camera::Impl::run()
{
  bool initialized = init();
  if (!initialized){
    return;
  }

  while (_isRunning)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    {
      std::lock_guard<std::mutex> lck(_mutex);
      std::cout<<"FPS: "<<_count<<std::endl;
      _count = 0;
    }
  }

  deinit();
}

bool Camera::Impl::init()
{
  int rc;

  if (!init_lib())
    return false;
  if (!init_channel())
    return false;
  if (!init_stream())
    return false;

  // Now start the channel
  rc = mm_app_start_channel(&_lib_handle.test_obj, _channel);
  if (rc != MM_CAMERA_OK)
    return false;

  return true;
}

bool Camera::Impl::init_lib()
{
  int rc;

  rc = mm_camera_lib_open(&_lib_handle, 0);
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:mm_camera_lib_open() err=%d\n", __func__, rc);
    return false;
  }

  int num_cameras = _lib_handle.app_ctx.num_cameras;
  if (num_cameras <= 0) {
    CDBG_ERROR("%s: No camera sensors reported!", __func__);
    return false;
  }
  return true;
}

bool Camera::Impl::init_channel()
{
  _channel = mm_app_add_channel(&_lib_handle.test_obj,
                               MM_CHANNEL_TYPE_RDI,
                               NULL,
                               NULL,
                               NULL);
  if (_channel == NULL) {
    CDBG_ERROR("%s: add channel failed", __func__);
    return false;
  }
  return true;
}

bool Camera::Impl::init_stream()
{
  int rc;
  mm_camera_buf_notify_t stream_cb = &Camera::Impl::static_callback;
  void* userdata = static_cast<void*>(this);

  // Create a stream
  mm_camera_stream_t* stream = mm_app_add_stream(&_lib_handle.test_obj, _channel);
  if (stream == NULL) {
    CDBG_ERROR("%s: add stream failed\n", __func__);
    return false;
  }

  // TODO: Verify RDI is a supported format
  // BRC: Supported raw formats based on capabilities
  // CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR
  // CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_10BPP_BGGR
  // CAM_FORMAT_BAYER_QCOM_RAW_10BPP_BGGR
  // CAM_FORMAT_YUV_422_NV16
  // CAM_FORMAT_YUV_422_NV61
  // Only CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR seems to actually bypass ISP
  cam_format_t fmt = CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR;

  cam_capability_t *cam_cap = (cam_capability_t *)(_lib_handle.test_obj.cap_buf.buf.buffer);

  // Set stream

  stream->s_config.mem_vtbl.get_bufs = mm_app_stream_initbuf;
  stream->s_config.mem_vtbl.put_bufs = mm_app_stream_deinitbuf;
  stream->s_config.mem_vtbl.clean_invalidate_buf = mm_app_stream_clean_invalidate_buf;
  stream->s_config.mem_vtbl.invalidate_buf = mm_app_stream_invalidate_buf;
  stream->s_config.mem_vtbl.user_data = (void *)stream;
  stream->s_config.stream_cb = stream_cb;
  stream->s_config.userdata = userdata;
  stream->num_of_bufs = _params.num_bufs;
  
  stream->s_config.stream_info = (cam_stream_info_t *)stream->s_info_buf.buf.buffer;
  memset(stream->s_config.stream_info, 0, sizeof(cam_stream_info_t));
  stream->s_config.stream_info->stream_type = CAM_STREAM_TYPE_RAW;
  stream->s_config.stream_info->streaming_mode = CAM_STREAMING_MODE_CONTINUOUS;
  stream->s_config.stream_info->num_of_burst = 0;

  stream->s_config.stream_info->fmt = fmt;
  stream->s_config.stream_info->dim.width = cam_cap->raw_dim.width;
  stream->s_config.stream_info->dim.height = cam_cap->raw_dim.height;
  stream->s_config.padding_info = cam_cap->padding_info;

  rc = mm_app_config_stream(&_lib_handle.test_obj, _channel, stream, &stream->s_config);
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:config rdi stream err=%d\n", __func__, rc);
    return false;
  }

  _stream = stream;

  return true;
}

void Camera::Impl::deinit()
{
  // TODO: Implement deinit
}

void Camera::Impl::static_callback(mm_camera_super_buf_t *bufs, void *userdata)
{
  Camera::Impl* camera = static_cast<Camera::Impl*>(userdata);
  camera->callback(bufs);
}

void Camera::Impl::callback(mm_camera_super_buf_t *bufs)
{
  // std::cerr<<"PDORAN non-static callback"<<std::endl;

  // TODO: Consider thread safety in the future when we actually start to do stuff with the bufs
  std::lock_guard<std::mutex> lck(_mutex);
  
  // TODO: See if there are other ways of enqueing the buffers rather than going to the ops vtable
  ++_count;

  for (uint32_t i = 0; i < bufs->num_bufs; i++) {
    if (MM_CAMERA_OK != _lib_handle.test_obj.cam->ops->qbuf(bufs->camera_handle,bufs->ch_id,bufs->bufs[i]))
    {
      CDBG_ERROR("%s: Failed in Qbuf\n", __func__);
    }
  }
}

//======================================================================================================================
// Camera Public Implementation
Camera::Camera()
  : _impl(std::unique_ptr<Camera::Impl>(new Camera::Impl())) // std::make_unique requires c++14
{
}

Camera::~Camera(){ }
void Camera::start() { _impl->start(); }
void Camera::stop() { _impl->stop(); }

} /* namespace anki */