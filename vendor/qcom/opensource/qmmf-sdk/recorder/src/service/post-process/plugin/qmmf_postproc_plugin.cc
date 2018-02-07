/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#define TAG "ReprocessPlugin"

#include "recorder/src/service/qmmf_recorder_utils.h"

#include "qmmf_postproc_plugin.h"

namespace qmmf {

namespace recorder {

template <typename T>
inline PostProcPlugin<T>::PostProcPlugin(T* source) {

  BufferProducerImpl<T> *producer_impl;
  producer_impl = new BufferProducerImpl<T>(source);
  buffer_producer_impl_ = producer_impl;

  BufferConsumerImpl<T> *impl;
  impl = new BufferConsumerImpl<T>(source);
  buffer_consumer_impl_ = impl;

}

template <typename T>
PostProcPlugin<T>::~PostProcPlugin() {

  buffer_consumer_impl_.clear();
  buffer_producer_impl_.clear();
}

template <typename T>
void PostProcPlugin<T>::AttachConsumer(sp<IBufferConsumer>& consumer) {
  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);
}

template <typename T>
void PostProcPlugin<T>::DetachConsumer(sp<IBufferConsumer>& consumer) {
  buffer_producer_impl_->RemoveConsumer(consumer);
}

template <typename T>
void PostProcPlugin<T>::NotifyBufferReturn(StreamBuffer& buffer) {
  buffer_consumer_impl_->GetProducerHandle()->NotifyBufferReturned(buffer);
}

template <typename T>
void PostProcPlugin<T>::NotifyBuffer(StreamBuffer& buffer) {
  buffer_producer_impl_->NotifyBuffer(buffer);
}

template <typename T>
sp<IBufferConsumer>& PostProcPlugin<T>::GetConsumerIntf() {
  return buffer_consumer_impl_;
}

template <typename T>
uint32_t PostProcPlugin<T>::GetNumConsumer() {
  return buffer_producer_impl_->GetNumConsumer();
}

}; // namespace recoder

}; // namespace qmmf

