/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {

namespace recorder {

class IBufferConsumer;

// Buffer Producer interface.
class IBufferProducer : public RefBase {
 public:
  virtual ~IBufferProducer() {}

  // This method would provide the buffer to all connected consumers.
  virtual void NotifyBuffer(StreamBuffer& buffer) = 0;

  // By using this method consumers would return buffer back to producer once
  // they done with buffer.
  virtual void NotifyBufferReturned(StreamBuffer& buffer) = 0;

  // By using this method consumer can be added to producer's list of consumer.
  virtual void AddConsumer(const sp<IBufferConsumer>& consumer) = 0;

  // Consumer can be removed at any point of time.
  virtual void RemoveConsumer(sp<IBufferConsumer>& consumer) = 0;

  // To provide number of connected consumers.
  int32_t GetNumConsumer() {
      Mutex::Autolock autoLock(lock_);
      return buffer_consumers_.size();
  }

 protected:
  // Lock to protect list of consumer.
  Mutex lock_;

  // Lock to protect return buffer sequence, buffer would come back to produces
  // from different thread context
  Mutex buffer_return_lock_;

  // List of consumers.
  Vector< sp<IBufferConsumer>> buffer_consumers_;
};

class IBufferConsumer : public RefBase {
 public:
  virtual ~IBufferConsumer() {}

  // Consumer's method to handle incoming buffer.
  virtual void OnFrameAvailable(StreamBuffer& buffer) = 0;

  // Set handle of producer, would be used to return buffers back to producer.
  virtual void SetProducerHandle(sp<IBufferProducer>& producer) = 0;

  sp<IBufferProducer>& GetProducerHandle() { return buffer_producer_; }

 protected:
  sp<IBufferProducer> buffer_producer_;

};

template <typename _type>
class BufferProducerImpl : public IBufferProducer {
 public:
  BufferProducerImpl(_type* source);

  ~BufferProducerImpl();

  void NotifyBuffer(StreamBuffer& buffer);

  void NotifyBufferReturned(StreamBuffer& Buffer);

  void AddConsumer(const sp<IBufferConsumer>& consumer);

  void RemoveConsumer(sp<IBufferConsumer>& consumer);

 private:
  _type* source_;

  TSKeyedVector<buffer_handle_t, uint32_t >  buffer_map_;
};

template <typename _type>
class BufferConsumerImpl : public IBufferConsumer {

 public:
  BufferConsumerImpl(_type* source);

  ~BufferConsumerImpl();

  void OnFrameAvailable(StreamBuffer& buffer);

  void SetProducerHandle(sp<IBufferProducer>& producer) {
    assert(producer.get() != NULL);
    buffer_producer_ = producer;
  }

 private:
  _type * source_;

};

template <typename _type>
BufferProducerImpl<_type>::BufferProducerImpl(_type* source)
    : source_(source) {
    QMMF_INFO("%s:%s: Enter", TAG, __func__);

    QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

template <typename _type>
BufferProducerImpl<_type>::~BufferProducerImpl() {
    QMMF_INFO("%s:%s: Enter", TAG, __func__);
    QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

template <typename _type>
void BufferProducerImpl<_type>::NotifyBuffer(StreamBuffer& buffer) {
  Mutex::Autolock autoLock(lock_);
  //Check for any consumer present. Notify them
  //about the new incoming buffer and keep reference count.
  if (!buffer_consumers_.isEmpty()) {
      buffer_map_.Add(buffer);
      buffer_map_.ReplaceValueFor(buffer, buffer_consumers_.size());

      QMMF_VERBOSE("%s:%s: (%p) buffer(0x%p) map size(%d) and it's ref count(%d)"
          , TAG, __func__ , this, buffer.handle, buffer_map_.Size(),
          buffer_map_.ValueFor(buffer));

      QMMF_VERBOSE("%s:%s: Notify buffer to %d-consumers", TAG, __func__,
          buffer_consumers_.size());

      for(auto& iter : buffer_consumers_) {
          (iter)->OnFrameAvailable(buffer);
      }
  } else {
      QMMF_WARN("%s:%s: No consumer connected to ProducerImpl, Return Buffer",
          TAG, __func__);
      source_->NotifyBufferReturned(buffer);
  }
}

template <typename _type>
void BufferProducerImpl<_type>::NotifyBufferReturned(StreamBuffer& buffer) {

  Mutex::Autolock autoLock(buffer_return_lock_);

   if (!buffer_map_.IsExist(buffer)) {
    QMMF_INFO("%s:%s: Warning Buffer is already returned (%p)",
        TAG, __func__, buffer.handle);
    return;
  }

  assert(buffer_map_.ValueFor(buffer) > 0);

  if(buffer_map_.ValueFor(buffer) == 1) {
    buffer_map_.RemoveItem(buffer);
    // Return buffer back to actual owner.
    source_->NotifyBufferReturned(buffer);
  } else {
    // Hold this buffer, do not return until its ref count is 1.
    QMMF_INFO("%s:%s: Hold buffer Refcount is not 1", TAG, __func__);
    uint32_t value = buffer_map_.ValueFor(buffer);
    buffer_map_.ReplaceValueFor(buffer, --value);
  }
}

template <typename _type>
void BufferProducerImpl<_type>::AddConsumer(const sp<IBufferConsumer>&
                                            consumer) {

  assert(consumer.get() != NULL);
  Mutex::Autolock autoLock(lock_);
  buffer_consumers_.add(consumer);
  QMMF_VERBOSE("%s:%s: Consumer(%p) added successfully!", TAG, __func__,
      consumer.get());
}

template <typename _type>
void BufferProducerImpl<_type>::RemoveConsumer(sp<IBufferConsumer>& consumer) {

  assert(consumer.get() != NULL);
  Mutex::Autolock autoLock(lock_);

  Vector<sp<IBufferConsumer> >::iterator iter = buffer_consumers_.begin();
  for(; iter != buffer_consumers_.end(); ++iter) {
    QMMF_INFO("%s:%s: (%p) iter=%p & consumer=%p", TAG, __func__, this,
        (*iter).get(), consumer.get());
    if((*iter).get() == consumer.get()) {
      QMMF_INFO("%s:%s: Consumer(%p) Removed!", TAG,__func__, consumer.get());
      iter = buffer_consumers_.erase(iter);
      break;
    }
  }
}

template <typename _type>
BufferConsumerImpl<_type>::BufferConsumerImpl(_type* source)
    : source_(source)
{
    QMMF_INFO("%s:%s: Enter", TAG, __func__);
    QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

template <typename _type>
BufferConsumerImpl<_type>::~BufferConsumerImpl()
{
    QMMF_INFO("%s:%s: Enter", TAG, __func__);
    QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

template <typename _type>
void BufferConsumerImpl<_type>::OnFrameAvailable(StreamBuffer& buffer)
{
    source_->OnFrameAvailable(buffer);
}

};

};