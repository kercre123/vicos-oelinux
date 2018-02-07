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

#define TAG "PlayerCommon"

#include <string.h>
#include <cutils/properties.h>

#include "common/qmmf_log.h"
#include "player/src/service/qmmf_player_common.h"

namespace qmmf {
namespace player {

extern "C" void DebugAudioTrackCreateParam (const char* _func_,
                                            AudioTrackCreateParam& track_params)
{
  QMMF_VERBOSE("%s:%s bit_depth = %u", TAG, _func_, track_params.bit_depth);
  QMMF_VERBOSE("%s:%s channels = %u", TAG, _func_, track_params.channels);
  QMMF_VERBOSE("%s:%s codec = %u", TAG, _func_, track_params.codec);
  QMMF_VERBOSE("%s:%s sample_rate = %u", TAG, _func_, track_params.sample_rate);
  QMMF_VERBOSE("%s:%s out_device = %u", TAG, _func_, track_params.out_device);
}

extern "C" void DebugVideoTrackCreateParam (const char* _func_,
                                            VideoTrackCreateParam& track_params)
{
  QMMF_VERBOSE("%s:%s width = %u", TAG, _func_, track_params.width);
  QMMF_VERBOSE("%s:%s height = %u", TAG, _func_, track_params.height);
  QMMF_VERBOSE("%s:%s codec = %u", TAG, _func_, track_params.codec);
  QMMF_VERBOSE("%s:%s out_device = %u", TAG, _func_, track_params.out_device);
}

extern "C" void DebugAudioSinkParam (const char* _func_,
                                     AudioTrackParams& track_params)
{
  QMMF_VERBOSE("%s:%s INPARAM: bit_depth[%u]", TAG, __func__,
      track_params.params.bit_depth);
  QMMF_VERBOSE("%s:%s INPARAM: channels[%u]", TAG, __func__,
      track_params.params.channels);
  QMMF_VERBOSE("%s:%s INPARAM: sample_rate[%u]", TAG, __func__,
      track_params.params.sample_rate);
}

extern "C" void DebugQueueInputBuffer(const char* _func_,
                                      std::vector<AVCodecBuffer>& buffers)
{
  for (uint32_t i =0 ; i<buffers.size(); i++)
  {
    QMMF_DEBUG("%s:%s: fd %d", TAG, __func__,buffers[i].buf_id);
    QMMF_DEBUG("%s:%s: filled_length %d", TAG, __func__,buffers[i].filled_length);
    QMMF_DEBUG("%s:%s: frame_length %d", TAG, __func__,buffers[i].frame_length);
    QMMF_DEBUG("%s:%s: vaddr 0x%p", TAG, __func__,buffers[i].data);
  }
}

extern "C" uint32_t GetPlayerDecodeProfileProperty()
{
  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.player.decode.profile", prop, "0");
  return (uint32_t) atoi(prop);
}

};  // namespace player
};  // namespace qmmf
