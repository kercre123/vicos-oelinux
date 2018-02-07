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

#include <stdio.h>
#include <unistd.h>
#include <string>

#include "qmmf_video_time_lapse.h"

const uint32_t kDefaultCameraId = 0;
const uint32_t kDefaultTimeLapseInterval = 500;
const uint32_t kDefaultWidth = 3840;
const uint32_t kDefaultHeight = 2160;
const uint32_t kDefaultFPS = 30;
const uint32_t kDefaultTimeLpaseMode =
    static_cast<uint32_t>(TimeLapseMode::kVideoTimeLapse);
const uint32_t kDefaultVideoEncodeFormat =
    static_cast<uint32_t>(VideoEncodeFormat::kAVC);
const uint32_t kDefaultImageEncodeFormat =
    static_cast<uint32_t>(ImageEncodeFormat::kJPEG);
const uint32_t kDefaultRecDuration = 120;

enum Arguments : char {
  kCameraId = 'c',
  kTimeLapseMode = 'm',
  kTimeLapseInterval = 'i',
  kWidth = 'w',
  kHeight = 'h',
  kVideoEncodeFormat = 'v',
  kImageEncodeFormat = 'p',
  kFPS = 'f',
  kRecDuration = 'd',
};

const char kArgs[] = {Arguments::kCameraId,
                      ':',
                      Arguments::kTimeLapseMode,
                      ':',
                      Arguments::kTimeLapseInterval,
                      ':',
                      Arguments::kWidth,
                      ':',
                      Arguments::kHeight,
                      ':',
                      Arguments::kVideoEncodeFormat,
                      ':',
                      Arguments::kImageEncodeFormat,
                      ':',
                      Arguments::kFPS,
                      ':',
                      Arguments::kRecDuration,
                      ':',
                      '\n'};

struct UsageDescription {
  Arguments arg;
  std::string desc;
  uint32_t default_value;
};

const UsageDescription kDescription[] = {
    {Arguments::kCameraId, "camera_id ", kDefaultCameraId},
    {Arguments::kTimeLapseMode, "time_lapse_mode ", kDefaultTimeLpaseMode},
    {Arguments::kTimeLapseInterval, "time_lapse_interval [ms.] ",
     kDefaultTimeLapseInterval},
    {Arguments::kWidth, "width ", kDefaultWidth},
    {Arguments::kHeight, "height ", kDefaultHeight},
    {Arguments::kVideoEncodeFormat, "video_encode_format ",
     kDefaultVideoEncodeFormat},
    {Arguments::kImageEncodeFormat, "image_encode_format ",
     kDefaultImageEncodeFormat},
    {Arguments::kFPS, "fps ", kDefaultFPS},
    {Arguments::kRecDuration, "record_duration [sec] ", kDefaultRecDuration},
};

void print_usage() {
  std::string usage_str("Usage: ");
  std::string defaults_str("\nDefaults:\n");
  size_t arg_count = sizeof(kDescription) / sizeof(kDescription[0]);
  for (size_t i = 0; i < arg_count; i++) {
    std::ostringstream default_string;
    default_string << kDescription[i].desc.c_str();
    default_string << " - default value: {";
    default_string << kDescription[i].default_value << "}" << '\n';
    defaults_str.append(default_string.str());

    std::ostringstream usage_string;
    usage_string << "-";
    usage_string << static_cast<char>(kDescription[i].arg) << " ";
    usage_string << kDescription[i].desc.c_str();
    usage_str.append(usage_string.str());
  }
  usage_str.append(defaults_str);

  printf("%s\n", usage_str.c_str());
}

void print_params(const TimeLapseParams &params) {
  printf("CameraId: %u\n", params.camera_id);
  printf("Time Lapse Mode: %u\n", params.time_lapse_mode);
  printf("Time lapse interval: %u[ms]\n", params.time_lapse_interval);
  printf("Width: %u\n", params.width);
  printf("Height: %u\n", params.height);
  printf("FPS: %u\n", params.fps);
  printf("Video Encode Format: %u \n",
         static_cast<uint32_t>(params.video_encode_format));
  printf("Image Encode Format: %u \n",
         static_cast<uint32_t>(params.image_encode_format));
  printf("Record Duration: %u[sec]\n", params.record_duration);
}

int main(int argc, char *argv[]) {
  int val, opt;
  int32_t ret;

  TimeLapseParams params = {
      kDefaultCameraId,
      kDefaultTimeLpaseMode,
      kDefaultTimeLapseInterval,
      kDefaultWidth,
      kDefaultHeight,
      static_cast<VideoEncodeFormat>(kDefaultVideoEncodeFormat),
      static_cast<ImageEncodeFormat>(kDefaultImageEncodeFormat),
      kDefaultFPS,
      kDefaultRecDuration};

  if (argc > 1) {
    if ((argc >= 2) &&
        ((strcmp(argv[1], "--help") == 0) || (strcmp(argv[1], "--h") == 0))) {
      print_usage();
      exit(EXIT_SUCCESS);
    }

    while ((opt = getopt(argc, argv, kArgs)) != -1) {
      switch (opt) {
        case Arguments::kCameraId:
          val = atoi(optarg);
          if (0 > val) {
            printf("%s: Invalid Camera ID: %d\n", __func__, val);
            exit(EXIT_FAILURE);
          }
          params.camera_id = static_cast<decltype(params.camera_id)>(val);
          break;
        case Arguments::kTimeLapseMode:
          val = atoi(optarg);
          if (0 > val) {
            printf("%s: Invalid Time Lapse Mode: %d\n", __func__, val);
            exit(EXIT_FAILURE);
          }
          params.time_lapse_mode =
              static_cast<decltype(params.time_lapse_mode)>(val);
          break;
        case Arguments::kTimeLapseInterval:
          val = atoi(optarg);
          if (0 >= val) {
            printf("%s: Invalid Time Lapse Period: %d\n", __func__, val);
            exit(EXIT_FAILURE);
          }
          params.time_lapse_interval =
              static_cast<decltype(params.time_lapse_interval)>(val);
          break;
        case Arguments::kWidth:
          val = atoi(optarg);
          if (0 >= val) {
            printf("%s: Invalid Width: %d\n", __func__, val);
            exit(EXIT_FAILURE);
          }
          params.width = static_cast<decltype(params.width)>(val);
          break;
        case Arguments::kHeight:
          val = atoi(optarg);
          if (0 >= val) {
            printf("%s: Invalid Height: %d\n", __func__, val);
            exit(EXIT_FAILURE);
          }
          params.height = static_cast<decltype(params.height)>(val);
          break;
        case Arguments::kVideoEncodeFormat:
          val = atoi(optarg);
          if (0 > val) {
            printf("%s: Invalid Video Encode Format: %d\n", __func__,
                   static_cast<uint32_t>(val));
            exit(EXIT_FAILURE);
          }
          params.video_encode_format =
              static_cast<decltype(params.video_encode_format)>(val);
          break;
        case Arguments::kImageEncodeFormat:
          val = atoi(optarg);
          if (0 > val) {
            printf("%s: Invalid Image Encode Format: %d\n", __func__,
                   static_cast<uint32_t>(val));
            exit(EXIT_FAILURE);
          }
          params.image_encode_format =
              static_cast<decltype(params.image_encode_format)>(val);
          break;
        case Arguments::kFPS:
          val = atoi(optarg);
          if (0 >= val) {
            printf("%s: Invalid FPS: %d\n", __func__, val);
            exit(EXIT_FAILURE);
          }
          params.fps = static_cast<decltype(params.fps)>(val);
          break;
        case Arguments::kRecDuration:
          val = atoi(optarg);
          if (0 >= val) {
            printf("%s: Invalid Record Duration: %d\n", __func__, val);
            exit(EXIT_FAILURE);
          }
          params.record_duration =
              static_cast<decltype(params.record_duration)>(val);
          break;
        default:
          print_usage();
          exit(EXIT_FAILURE);
      }
    }
  }
  print_params(params);

  TimeLapse timelapse(params);
  ret = timelapse.Start();
  if (ret != 0) {
    printf("%s: Exiting Application:\n", __func__);
    exit(EXIT_FAILURE);
  }
  sleep(params.record_duration);
  return timelapse.Stop();
}
