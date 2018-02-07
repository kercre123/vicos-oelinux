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

#include <unistd.h>
#include <stdio.h>
#include <utils/String8.h>

#include "qmmf_time_lapse.h"

using namespace qmmf::timelapse;
using namespace android;

const uint32_t kDefaultCameraId       = 0;
const uint32_t kDefaultSnapshotWidth  = 3840;
const uint32_t kDefaultSnapshotHeight = 2160;
const uint32_t kDefaultPreviewWidth   = 640;
const uint32_t kDefaultPreviewHeight  = 480;
const uint32_t kDefaultLapsePeriod    = 500; // [ms.]
const uint32_t kDefaultLapseCount     = 30;

enum Arguments : char {
  kCameraId       = 'c',
  kSnapshotWidth  = 'w',
  kSnapshotHeight = 'h',
  kPreviewWidth   = 'p',
  kPreviewHeight  = 'o',
  kLapsePeriod    = 'l',
  kLapseCount     = 'n',
};

const char kArgs[] = {
    Arguments::kCameraId, ':',
    Arguments::kSnapshotWidth,':',
    Arguments::kSnapshotHeight,':',
    Arguments::kPreviewWidth,':',
    Arguments::kPreviewHeight,':',
    Arguments::kLapsePeriod,':',
    Arguments::kLapseCount,':',
    '\n'
};

struct UsageDescription {
  Arguments arg;
  const char *desc;
  uint32_t default_value;
};

const UsageDescription kDescription [] = {
    {Arguments::kCameraId, "camera_id ",
        kDefaultCameraId},
    {Arguments::kSnapshotWidth, "snapshot_width ",
        kDefaultSnapshotWidth},
    {Arguments::kSnapshotHeight, "snapshot_height ",
        kDefaultSnapshotHeight},
    {Arguments::kPreviewWidth, "preview_width ",
        kDefaultPreviewWidth},
    {Arguments::kPreviewHeight, "preview_height ",
        kDefaultPreviewHeight},
    {Arguments::kLapsePeriod, "period [ms.] ",
        kDefaultLapsePeriod},
    {Arguments::kLapseCount, "count ",
        kDefaultLapseCount},
};

void print_usage() {
  String8 usage_str("Usage: ");
  String8 defaults_str("\nDefaults:\n");
  size_t arg_count = sizeof(kDescription) / sizeof(kDescription[0]);
  for (size_t i = 0; i < arg_count; i++) {
    defaults_str.appendFormat("%s - default value: {%u}\n",
                              kDescription[i].desc,
                              kDescription[i].default_value);
    usage_str.appendFormat("-%c %s ", kDescription[i].arg,
                           kDescription[i].desc);
  }
  usage_str.append(defaults_str);

  printf("%s\n", usage_str.string());
}

void print_params(const TimeLapseParams &params) {
  printf("CameraId: %u\n", params.camera_id);
  printf("Snapshot Width: %u\n", params.snapshot_width);
  printf("Snapshot Height: %u\n", params.snapshot_height);
  printf("Preview Width: %u\n", params.preview_width);
  printf("Preview Height: %u\n", params.preview_height);
  printf("Time lapse period: %u [ms.]\n", params.period);
  printf("Time lapse count: %u\n", params.count);
}

int main(int argc, char *argv[]) {
  if (1 >= argc) {
    print_usage();
    exit(EXIT_SUCCESS);
  }

  int val, opt;
  TimeLapseParams params = {kDefaultCameraId, kDefaultPreviewWidth,
                            kDefaultPreviewHeight, kDefaultSnapshotWidth,
                            kDefaultSnapshotHeight, kDefaultLapsePeriod,
                            kDefaultLapseCount};
  while ((opt = getopt(argc, argv, kArgs)) != -1) {
    switch (opt) {
      case Arguments::kCameraId:
        val = atoi(optarg);
        if (0 > val) {
          printf("%s: Invalid camera id: %d\n", __func__, val);
          exit(EXIT_FAILURE);
        }
        params.camera_id = static_cast<decltype(params.camera_id)> (val);
        break;
      case Arguments::kSnapshotWidth:
        val = atoi(optarg);
        if (0 >= val) {
          printf("%s: Invalid snapshot width: %d\n", __func__, val);
          exit(EXIT_FAILURE);
        }
        params.snapshot_width =
            static_cast<decltype(params.snapshot_width)> (val);
        break;
      case Arguments::kSnapshotHeight:
        val = atoi(optarg);
        if (0 >= val) {
          printf("%s: Invalid snapshot height: %d\n", __func__, val);
          exit(EXIT_FAILURE);
        }
        params.snapshot_height =
            static_cast<decltype(params.snapshot_height)> (val);
        break;
      case Arguments::kPreviewWidth:
        val = atoi(optarg);
        if (0 >= val) {
          printf("%s: Invalid preview width: %d\n", __func__, val);
          exit(EXIT_FAILURE);
        }
        params.preview_width =
            static_cast<decltype(params.preview_width)> (val);
        break;
      case Arguments::kPreviewHeight:
        val = atoi(optarg);
        if (0 >= val) {
          printf("%s: Invalid preview height: %d\n", __func__, val);
          exit(EXIT_FAILURE);
        }
        params.preview_height =
            static_cast<decltype(params.preview_height)> (val);
        break;
      case Arguments::kLapsePeriod:
        val = atoi(optarg);
        if (0 >= val) {
          printf("%s: Invalid lapse period: %d\n", __func__, val);
          exit(EXIT_FAILURE);
        }
        params.period = static_cast<decltype(params.period)> (val);
        break;
      case Arguments::kLapseCount:
        val = atoi(optarg);
        if (0 >= val) {
          printf("%s: Invalid lapse count: %d\n", __func__, val);
          exit(EXIT_FAILURE);
        }
        params.count = static_cast<decltype(params.count)> (val);
        break;
      default:
        print_usage();
        exit(EXIT_FAILURE);
    }
  }
  print_params(params);

  TimeLapse lapse(params);
  return lapse.Run();
}
