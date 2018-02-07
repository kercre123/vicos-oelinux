/*
* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#include <inttypes.h>
#include <sys/time.h>
#include <math.h>
#include <log/log.h>
#include <dlfcn.h>
#include <utils/Errors.h>
#include "qmmf_http_interface_gtest.h"

#define LIB_PATH "libhttp_interface.so.0"
#define MODULE_NAME "QMMF_MODULE"

namespace qmmf {

namespace httpinterface {

HTTPInterfaceGtest::HTTPInterfaceGtest() {}

HTTPInterfaceGtest::~HTTPInterfaceGtest() {}

void HTTPInterfaceGtest::SetUp() {
  void *handle;
  struct qmmf_http_interface_t *ops;
  int32_t status = NO_ERROR;

  handle = dlopen(LIB_PATH, RTLD_NOW);
  if (NULL == handle) {
      printf("%s: Unable to open web interface library: %s!\n",
          __func__, dlerror());
      status = errno;
      goto EXIT;
  }

  ops = (struct qmmf_http_interface_t *) dlsym(handle, MODULE_NAME);
  if (NULL == ops) {
      printf("%s: Cannot find web interface: %s!\n",
          __func__, dlerror());
      status = errno;
      goto EXIT;
  }

  status = ops->open(&http_intf_.qmmf_mod);
  if (0 != status) {
      printf("%s: Module open failed: %d", __func__, status);
      goto EXIT;
  }

  http_intf_.ops = ops;
  http_intf_.lib_handle = handle;

  return;

EXIT:

  if (handle) {
      dlclose(handle);
  }

  ASSERT_TRUE(NO_ERROR != status);
}

void HTTPInterfaceGtest::TearDown() {
  ASSERT_FALSE(NULL == http_intf_.lib_handle);
  ASSERT_FALSE(NULL == http_intf_.ops);

  auto ret = http_intf_.ops->close(&http_intf_.qmmf_mod);
  dlclose(http_intf_.lib_handle);

  ASSERT_TRUE(NO_ERROR == ret);
}

TEST_F(HTTPInterfaceGtest, MP4_1080P_AAC_MONO_AAC) {
  size_t camera_id = 0;
  size_t video_track_id = 1;
  size_t audio_track_id = 101;
  struct qmmf_camera_start_param_t start_parm;
  struct qmmf_video_track_param_t video_parm;
  struct qmmf_audio_track_param_t audio_parm;
  uint32_t session_id;

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.connect(&http_intf_.qmmf_mod));

  memset(&start_parm, 0, sizeof(start_parm));
  start_parm.zsl_width = 1920;
  start_parm.zsl_height = 1080;
  start_parm.frame_rate = 30;
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.start_camera(&http_intf_.qmmf_mod,
                                                           camera_id,
                                                           start_parm));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_session(&http_intf_.qmmf_mod,
                                                             &session_id));

  memset(&video_parm, 0, sizeof(video_parm));
  video_parm.camera_id = camera_id;
  video_parm.width = 1920;
  video_parm.height = 1080;
  video_parm.framerate = 30;
  video_parm.codec = CODEC_AVC;
  video_parm.bitrate = 10000000;
  video_parm.output = TRACK_OUTPUT_MP4;
  video_parm.track_id = video_track_id;
  video_parm.session_id = session_id;
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_video_track(
      &http_intf_.qmmf_mod, video_parm));

  memset(&audio_parm, 0, sizeof(audio_parm));
  audio_parm.sample_rate = 48000;
  audio_parm.num_channels = 1;
  audio_parm.bit_depth = 16;
  audio_parm.codec = CODEC_AAC;
  audio_parm.bitrate = 128000;
  audio_parm.output = AUDIO_TRACK_OUTPUT_MP4;
  audio_parm.track_id = audio_track_id;
  audio_parm.session_id = session_id;
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_audio_track(
      &http_intf_.qmmf_mod, audio_parm));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.start_session(
      &http_intf_.qmmf_mod, session_id));

  //Record for 2 sec.
  sleep(2);

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.stop_session(
      &http_intf_.qmmf_mod, session_id, 1));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_video_track(
      &http_intf_.qmmf_mod, session_id, video_track_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_audio_track(
      &http_intf_.qmmf_mod, session_id, audio_track_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_session(
      &http_intf_.qmmf_mod, session_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.stop_camera(&http_intf_.qmmf_mod,
                                                          camera_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.disconnect(
      &http_intf_.qmmf_mod));
}

TEST_F(HTTPInterfaceGtest, VAM_1080p) {
  size_t camera_id = 0;
  size_t video_track_id = 1;
  size_t vam_track_id = 2;
  struct qmmf_camera_start_param_t start_parm;
  struct qmmf_video_track_param_t video_parm;
  uint32_t session_id, session_id_vam;

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.connect(&http_intf_.qmmf_mod));

  memset(&start_parm, 0, sizeof(start_parm));
  start_parm.zsl_width = 1920;
  start_parm.zsl_height = 1080;
  start_parm.frame_rate = 30;
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.start_camera(&http_intf_.qmmf_mod,
                                                           camera_id,
                                                           start_parm));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_session(&http_intf_.qmmf_mod,
                                                             &session_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_session(&http_intf_.qmmf_mod,
                                                             &session_id_vam));

  memset(&video_parm, 0, sizeof(video_parm));
  video_parm.camera_id = camera_id;
  video_parm.width = 1920;
  video_parm.height = 1080;
  video_parm.framerate = 30;
  video_parm.codec = CODEC_AVC;
  video_parm.bitrate = 10000000;
  video_parm.output = TRACK_OUTPUT_RTSP;
  video_parm.track_id = video_track_id;
  video_parm.session_id = session_id;
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_video_track(
      &http_intf_.qmmf_mod, video_parm));

  video_parm.codec = CODEC_YUV;
  video_parm.output = TRACK_OUTPUT_VAM;
  video_parm.track_id = vam_track_id;
  video_parm.session_id = session_id_vam;
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_video_track(
      &http_intf_.qmmf_mod, video_parm));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.start_session(
      &http_intf_.qmmf_mod, session_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.start_session(
      &http_intf_.qmmf_mod, session_id_vam));

  sleep(30);

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.stop_session(
      &http_intf_.qmmf_mod, session_id, 1));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.stop_session(
      &http_intf_.qmmf_mod, session_id_vam, 1));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_video_track(
      &http_intf_.qmmf_mod, session_id, video_track_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_video_track(
      &http_intf_.qmmf_mod, session_id_vam, vam_track_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_session(
      &http_intf_.qmmf_mod, session_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_session(
      &http_intf_.qmmf_mod, session_id_vam));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.stop_camera(&http_intf_.qmmf_mod,
                                                          camera_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.disconnect(
      &http_intf_.qmmf_mod));
}

TEST_F(HTTPInterfaceGtest, SetTrackParams) {
  size_t camera_id = 0;
  size_t video_track_id = 1;
  struct qmmf_camera_start_param_t start_parm;
  struct qmmf_video_track_param_t video_parm;
  uint32_t session_id;
  qmmf_track_param track_param;

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.connect(&http_intf_.qmmf_mod));

  memset(&start_parm, 0, sizeof(start_parm));
  start_parm.zsl_width = 1920;
  start_parm.zsl_height = 1080;
  start_parm.frame_rate = 30;
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.start_camera(&http_intf_.qmmf_mod,
                                                           camera_id,
                                                           start_parm));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_session(&http_intf_.qmmf_mod,
                                                             &session_id));

  memset(&video_parm, 0, sizeof(video_parm));
  video_parm.camera_id = camera_id;
  video_parm.width = 1920;
  video_parm.height = 1080;
  video_parm.framerate = 30;
  video_parm.codec = CODEC_AVC;
  video_parm.bitrate = 10000000;
  video_parm.output = TRACK_OUTPUT_MP4;
  video_parm.track_id = video_track_id;
  video_parm.session_id = session_id;
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.create_video_track(
      &http_intf_.qmmf_mod, video_parm));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.start_session(
      &http_intf_.qmmf_mod, session_id));

  float fps = 15.0;
  memset(&track_param, 0, sizeof(track_param));
  track_param.type = param_type::kFrameRateType;
  track_param.param = &fps;
  track_param.param_size = sizeof(fps);

  //Record for 5 sec.
  sleep(5);

  //SetVideoTrackParam
  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.set_video_track_param(
      &http_intf_.qmmf_mod, session_id, video_track_id, track_param));

  //Record for 5 sec.
  sleep(5);

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.stop_session(
      &http_intf_.qmmf_mod, session_id, 1));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_video_track(
      &http_intf_.qmmf_mod, session_id, video_track_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.delete_session(
      &http_intf_.qmmf_mod, session_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.stop_camera(&http_intf_.qmmf_mod,
                                                          camera_id));

  ASSERT_TRUE(NO_ERROR == http_intf_.qmmf_mod.disconnect(
      &http_intf_.qmmf_mod));
}

}  // namespace httpinterface ends here

}  // namespace qmmf ends here
