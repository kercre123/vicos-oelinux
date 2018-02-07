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

#include "qmmf_vam_interface.h"

#define UNUSED __attribute__((unused))

namespace qmmf {

namespace vaminterface {

using std::vector;

class VAMInstance : public VAMInterface {
public:
  VAMInstance(VAMCb *vam_cb) : vam_cb_(vam_cb) {};
  virtual ~VAMInstance() {};

private:
  int32_t InitVAM(UNUSED const uint32_t session_id,
                        UNUSED const uint32_t track_id) override {
    return NO_ERROR;
  }
  int32_t CloseVAM() override {
    return NO_ERROR;
  }
  int32_t StartVAM(UNUSED CameraBufferMetaData &meta_data) override {
    return NO_ERROR;
  }
  int32_t VAMEnroll(UNUSED VAMEnrollmentInfo &enroll_info) override {
    return NO_ERROR;
  }
  int32_t VAMDisenroll(UNUSED uint32_t event_type,
                       UNUSED const char *id) override {
    return NO_ERROR;
  }
  int32_t VAMConfig(UNUSED const char *json_config) override {
    return NO_ERROR;
  }
  int32_t VAMRemoveConfig(UNUSED const char *json_config) override {
    return NO_ERROR;
  }
  int32_t QueueVAMBuffers(uint32_t track_id, uint32_t session_id,
                          vector<BufferDescriptor> &buffers,
                          UNUSED vector<MetaData> &meta_data) override;
  int32_t InitVAMVLog(UNUSED uint32_t framerate) override {
      return NO_ERROR;
  }
  int32_t DeinitVAMVLog() override {
      return NO_ERROR;
  }
  int32_t ProcessVLogBuffers(std::vector<BufferDescriptor> &buffers) override {
      return NO_ERROR;
  }
  int32_t CheckDBParams(const VAMDatabaseCmdParams *params) {
      return NO_ERROR;
  }
  int32_t DatabaseCommand(UNUSED const VAMDatabaseCmdParams *params,
                          UNUSED VAMDatabaseCmdResult *result) override {
      return NO_ERROR;
  }

  VAMCb *vam_cb_;
};

int32_t VAMInstance::QueueVAMBuffers(uint32_t track_id,
                                     uint32_t session_id,
                                     vector<BufferDescriptor> &buffers,
                                     UNUSED vector<MetaData> &meta_data) {
  int32_t ret = NO_ERROR;

  for (size_t i = 0; i < buffers.size(); i++) {
    ret = vam_cb_->ReturnTrackBuffer(track_id,
                                     session_id,
                                     buffers[i]);
    if (NO_ERROR != ret) {
      ALOGE("%s: ReturnTrackBuffer failed: %d!", __func__, ret);
    }
  }

  return ret;
}

VAMInterface *VAMInterfaceFactory::NewInstance(VAMCb *vam_cb) {
  VAMInstance *new_instance = new VAMInstance(vam_cb);
  return static_cast<VAMInterface *>(new_instance);
}

} //namespace vaminterface ends here

} //namespace qmmf ends here
