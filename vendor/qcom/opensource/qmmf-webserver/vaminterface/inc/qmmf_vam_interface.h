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

#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include <utils/KeyedVector.h>
#include <qmmf-sdk/qmmf_recorder_params.h>

namespace qmmf {

namespace vaminterface {

using namespace android;
using namespace recorder;

typedef struct VAMEnrollmentInfo_t {
  VAMEnrollmentInfo_t() = delete;
  VAMEnrollmentInfo_t(char *in_id, char *in_display_name, char *in_img_id,
                      uint8_t *in_data, uint32_t in_object_type,
                      uint32_t in_event_type, uint32_t in_image_format,
                      uint32_t in_image_width, uint32_t in_image_height) :
      id(in_id), display_name(in_display_name), img_id(in_img_id),
      data(in_data), object_type(in_object_type), event_type(in_event_type),
      image_format(in_image_format), image_width(in_image_width),
      image_height(in_image_height) {};

  char *id;
  char *display_name;
  char *img_id;
  uint8_t *data;
  uint32_t object_type;
  uint32_t event_type;
  uint32_t image_format;
  uint32_t image_width;
  uint32_t image_height;
} VAMEnrollmentInfo;

typedef enum VAMDatabaseCommand_t {
    GET_ALL_EVENT_IDS = 0,
    GET_ALL_FRAME_IDS,
    GET_ALL_RULES_IDS,
    GET_EVENT,
    GET_METADATA_FOR_FRAME,
    GET_METADATA_FOR_EVENT,
    GET_SNAPSHOT,
    GET_VIDEO,
    GET_RULE,

    GET_EVENTS_FOR_TS_LIMIT,
    GET_LAST_EVENTS,
    GET_EVENTS_FOR_FRAME,
    GET_EVENTS_FOR_RULE,
    GET_EVENTS_FOR_RULE_TS_LIMIT,

    REMOVE_EVENT,
    REMOVE_RULE,

    GET_CURRENT_SESSION,
    GET_ALL_SESSIONS,

    GET_SNAPSHOT_ID_FOR_EVENT,
    GET_SNAPSHOT_FOR_EVENT,
    GET_VIDEO_FOR_EVENT,
    GET_THUMBNAIL,
    GET_THUMBNAIL_FOR_EVENT,

    GET_EVENTS_TIMESTAMPS,
    GET_EVENT_COUNT,
    SET_EVENT_PAGE_SIZE,
    GET_EVENT_PAGE_SIZE,
    GET_EVENT_PAGE,
    GET_EVENTS_BY_INDEX_RANGE,
    GET_EVENTS_BY_INDEX_RANGE_BEFORE_TS,
    GET_EVENTS_BY_INDEX_RANGE_AFTER_TS,

    FRDB_GET_IMAGE_IDS = 100,
    FRDB_GET_IMAGE_IDS_FOR_FEATURE,
    FRDB_GET_IMAGE_IDS_FOR_DISPLAY_NAME,
    FRDB_GET_FEATURE_IDS,
    FRDB_GET_FEATURE_IDS_FOR_IMAGE,
    FRDB_GET_FEATURE_IDS_FOR_DISPLAY_NAME,
    FRDB_GET_ENROLL_INFO,
    FRDB_GET_ENROLL_IMAGE_DATA,
    FRDB_REMOVE_ENROLLED_IMAGE,
} VAMDatabaseCommand;

typedef struct VAMDatabaseCmdParams_t {
    char *session;
    uint32_t command;
    char *id;
    uint64_t pts;
    uint64_t pts1;
    uint32_t *event_type;
    uint32_t num_event_types;
    uint32_t *index_range;
    uint32_t num_index_ranges;
    uint32_t page_index;
    uint32_t max_count;
} VAMDatabaseCmdParams;

typedef struct VAMDatabaseCmdResult_t {
    char *data;
    uint32_t data_size;
    uint32_t elements_count;
    uint32_t command;
    uint32_t status;
} VAMDatabaseCmdResult;

class VAMCb {
public:
  virtual int32_t SendVAMMeta(uint32_t session_id, uint32_t track_id,
                              const char *metaString, size_t size,
                              int64_t pts) = 0;
  virtual int32_t ReturnTrackBuffer(uint32_t track_id,
                                    uint32_t session_id,
                                    BufferDescriptor &buffer) = 0;
  virtual ~VAMCb() {};
};

class VAMInterface {
public:
  virtual int32_t InitVAM(const uint32_t session_id,
                                const uint32_t track_id) = 0;
  virtual int32_t CloseVAM() = 0;

  virtual int32_t StartVAM(CameraBufferMetaData &meta_data) = 0;

  virtual int32_t VAMEnroll(VAMEnrollmentInfo &enroll_info) = 0;
  virtual int32_t VAMDisenroll(uint32_t event_type, const char *id) = 0;
  virtual int32_t VAMConfig(const char *json_config) = 0;
  virtual int32_t VAMRemoveConfig(const char *json_config) = 0;
  virtual int32_t QueueVAMBuffers(uint32_t track_id, uint32_t session_id,
                                  std::vector<BufferDescriptor> &buffers,
                                  std::vector<MetaData> &meta_data) = 0;
  virtual int32_t InitVAMVLog(uint32_t framerate) = 0;
  virtual int32_t DeinitVAMVLog() = 0;
  virtual int32_t ProcessVLogBuffers(std::vector<BufferDescriptor> &buffers) = 0;
  virtual int32_t CheckDBParams(const VAMDatabaseCmdParams *params) = 0;
  virtual int32_t DatabaseCommand(const VAMDatabaseCmdParams *params,
                                  VAMDatabaseCmdResult *result) = 0;
  virtual ~VAMInterface() {};
};

class VAMInterfaceFactory {
public:
  VAMInterfaceFactory() = delete;

  static VAMInterface *NewInstance(VAMCb *vam_cb);
};

}

}
