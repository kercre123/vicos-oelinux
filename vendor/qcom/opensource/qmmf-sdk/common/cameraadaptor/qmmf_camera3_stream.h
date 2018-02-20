/*
 * Copyright (c) 2016 The Linux Foundation. All rights reserved.
 * Not a Contribution.
 */

/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CAMERA3STREAM_H_
#define CAMERA3STREAM_H_

#include <pthread.h>
#include <hardware/camera_common.h>
#include <hardware/camera3.h>
#ifdef TARGET_USES_GRALLOC1
#include <hardware/gralloc1.h>
#else
#include <hardware/gralloc.h>
#endif
#include <utils/String8.h>
#include <utils/Vector.h>
#include <utils/KeyedVector.h>

#include "qmmf_camera3_types.h"

using namespace android;

namespace qmmf {

namespace cameraadaptor {

class Camera3Monitor;

#ifdef TARGET_USES_GRALLOC1
   typedef gralloc1_device_t* mem_alloc_device;
   typedef gralloc1_error_t   mem_alloc_error;
#else
   typedef alloc_device_t*    mem_alloc_device;
   typedef int32_t            mem_alloc_error;
#endif

class IMemAllocator {
 public:
   IMemAllocator(mem_alloc_device device) : device_(device) {};
   virtual ~IMemAllocator() {};

   static IMemAllocator* CreateMemAllocator(mem_alloc_device device);

   mem_alloc_device GetDevice() { return device_; }

   virtual mem_alloc_error AllocBuffer(buffer_handle_t *buf,
                                       int32_t width,
                                       int32_t height,
                                       int32_t format,
                                       int32_t usage,
                                       uint32_t *stride) = 0;

   virtual mem_alloc_error FreeBuffer(buffer_handle_t buf) = 0;

   virtual mem_alloc_error GetStrideAndHeightFromHandle(
       struct private_handle_t* const priv_handle,
       int32_t* stride,
       int32_t* height) = 0;

 private:
   mem_alloc_device          device_;
};

#ifdef TARGET_USES_GRALLOC1
class Gralloc1Allocator : public IMemAllocator {
  public:
   Gralloc1Allocator(mem_alloc_device device);
   ~Gralloc1Allocator() {};

   mem_alloc_error AllocBuffer(buffer_handle_t *buf,
                               int32_t width,
                               int32_t height,
                               int32_t format,
                               int32_t usage,
                               uint32_t *stride) override;

   mem_alloc_error FreeBuffer(buffer_handle_t buf) override;

   mem_alloc_error GetStrideAndHeightFromHandle(
       struct private_handle_t* const priv_handle,
       int32_t* stride,
       int32_t* height) override;

  private:
   int32_t (*CreateDescriptor)(mem_alloc_device device,
                               gralloc1_buffer_descriptor_t* pCreatedDescriptor);

   int32_t (*DestroyDescriptor)(mem_alloc_device device,
                                gralloc1_buffer_descriptor_t descriptor);

   int32_t (*SetDimensions)(mem_alloc_device device,
                            gralloc1_buffer_descriptor_t descriptor,
                            uint32_t width, uint32_t height);

   int32_t (*SetFormat)(mem_alloc_device device,
                        gralloc1_buffer_descriptor_t descriptor,
                        int32_t format);

   int32_t (*SetProducerUsage)(mem_alloc_device device,
                               gralloc1_buffer_descriptor_t descriptor,
                               uint64_t usage);

   int32_t (*SetConsumerUsage)(mem_alloc_device device,
                               gralloc1_buffer_descriptor_t descriptor,
                               uint64_t usage);

   int32_t (*Allocate)(mem_alloc_device device, uint32_t numDescriptors,
                       const gralloc1_buffer_descriptor_t* pDescriptors,
                       buffer_handle_t* pAllocatedBuffers);

   int32_t (*GetStride)(mem_alloc_device device, buffer_handle_t buffer,
                        uint32_t* pStride);

   int32_t (*Release)(mem_alloc_device device, buffer_handle_t buffer);

   int32_t (*Lock)(mem_alloc_device device, buffer_handle_t buffer,
                   uint64_t producerUsage, uint64_t consumerUsage,
                   const gralloc1_rect_t* accessRegion, void** outData,
                   int32_t acquireFence);

   int32_t (*UnLock)(mem_alloc_device device, buffer_handle_t buffer,
                     int32_t* outReleaseFence);

   mem_alloc_error (*Perform)(mem_alloc_device device, int32_t operation, ...);
};
#else
class GrallocAllocator : public IMemAllocator {
 public:
   GrallocAllocator(mem_alloc_device device);
   ~GrallocAllocator() {};

   mem_alloc_error AllocBuffer(buffer_handle_t *buf,
                               int32_t width,
                               int32_t height,
                               int32_t format,
                               int32_t usage,
                               uint32_t *stride) override;

   mem_alloc_error FreeBuffer(buffer_handle_t buf) override;

   mem_alloc_error GetStrideAndHeightFromHandle(
       struct private_handle_t* const priv_handle,
       int32_t* stride,
       int32_t* height) override;
};
#endif  // TARGET_USES_GRALLOC1

class Camera3Stream : public camera3_stream {

 public:
  Camera3Stream(int id, size_t maxSize,
                const CameraStreamParameters &outputConfiguration,
                mem_alloc_device device, Camera3Monitor &monitor);
  virtual ~Camera3Stream();

  camera3_stream *BeginConfigure();
  int32_t EndConfigure();
  int32_t AbortConfigure();
  bool IsConfigureActive();

  int32_t BeginPrepare();
  int32_t PrepareBuffer();
  int32_t EndPrepare();
  bool IsPrepareActive();

  int32_t GetBuffer(camera3_stream_buffer *buffer);
  int32_t ReturnBuffer(const StreamBuffer &buffer);
  void ReturnBufferToClient(const camera3_stream_buffer &buffer,
                            int64_t timestamp, int64_t frame_number);

  int32_t Close();

  int GetId() const { return id_; }
  static Camera3Stream *CastTo(camera3_stream *stream) {
    return static_cast<Camera3Stream *>(stream);
  }
  bool IsStreamActive();

  int32_t TearDown();

 private:
  int32_t ConfigureLocked();
  int32_t GetBufferLocked(camera3_stream_buffer *buffer = NULL);
  int32_t ReturnBufferLocked(const StreamBuffer &buffer);
  uint32_t GetBufferCountLocked() { return total_buffer_count_; }
  uint32_t GetPendingBufferCountLocked() { return pending_buffer_count_; }
  int32_t CloseLocked();

  int32_t EndPrepareLocked();
  int32_t PopulateMetaInfo(CameraBufferMetaData &info,
                           struct private_handle_t *priv_handle);

  /**Not allowed */
  Camera3Stream(const Camera3Stream &);
  Camera3Stream &operator=(const Camera3Stream &);

  IMemAllocator* mem_alloc_interface_;

  uint32_t current_buffer_stride_;
  const int32_t id_;
  // Should be zero for non-blob formats
  const uint32_t max_size_;

  typedef enum Status_t {
    STATUS_ERROR,
    STATUS_INTIALIZED,
    STATUS_CONFIG_ACTIVE,
    STATUS_RECONFIG_ACTIVE,
    STATUS_CONFIGURED,
    STATUS_PREPARE_ACTIVE,
  } Status;

  pthread_mutex_t lock_;

  Status status_;
  uint32_t total_buffer_count_;
  uint32_t pending_buffer_count_;

  StreamCallback callbacks_;
  uint32_t old_usage_, client_usage_;
  uint32_t old_max_buffers_, client_max_buffers_;
  pthread_cond_t output_buffer_returned_signal_;
  static const int64_t BUFFER_WAIT_TIMEOUT = 1e9;  // 1 sec.

  KeyedVector<buffer_handle_t, bool> gralloc_buffers_;
  buffer_handle_t *gralloc_slots_;
  uint32_t gralloc_buffer_allocated_;

  Camera3Monitor &monitor_;
  int32_t monitor_id_;

  bool is_stream_active_;
  uint32_t prepared_buffers_count_;
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3STREAM_H_ */
