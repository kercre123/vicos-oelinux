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

#pragma once

#include <utils/RefBase.h>

#include "common/qmmf_common_utils.h"

namespace qmmf {

namespace recorder {

using android::RefBase;
using android::sp;

class IBufferProducer;

class IBufferConsumer;

template <typename T>
class PostProcPlugin {

 public:

  PostProcPlugin(T* source);

  virtual ~PostProcPlugin();

  void AttachConsumer(sp<IBufferConsumer>& consumer);

  void NotifyBufferReturn(StreamBuffer& buffer);

  void NotifyBuffer(StreamBuffer& buffer);

  uint32_t GetNumConsumer();

  sp<IBufferConsumer>& GetConsumerIntf();

  void DetachConsumer(sp<IBufferConsumer>& consumer);

  virtual void OnFrameAvailable(StreamBuffer& buffer) = 0;

  virtual void NotifyBufferReturned(StreamBuffer& buffer) = 0;

 private:

  sp<IBufferProducer>      buffer_producer_impl_;

  sp<IBufferConsumer>      buffer_consumer_impl_;

};

}; //namespace recorder

}; //namespace qmmf
