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
#include <cmath>
#include <chrono>

#include "qmmf-plugin/qmmf_alg_utils.h"

namespace qmmf {

namespace qmmf_alg_plugin {

/** BufferHandler:
 *
 *  This class handles qmmf algo buffer
 **/
class BufferHandler : AlgBuffer {
 public:
  BufferHandler(uint8_t *vaddr, int32_t fd, uint32_t size, bool cached,
                PixelFormat pix_fmt, int64_t timestamp, uint32_t frame_number,
                std::vector<BufferPlane> plane)
      : AlgBuffer(vaddr, fd, size, cached, pix_fmt, timestamp,
                  frame_number, plane) {}

  ~BufferHandler() { free(vaddr_); }

  /** New
    *    @requirements: buffer requirements
    *    @pix_fmt: pixel format
    *    @width: width
    *    @height: height
    *
    * creates new instance of qmmf qlgo buffer handler
    *
    * return: list of shared pointers of buffer handlers
    **/
  static std::list<std::shared_ptr<BufferHandler> > New(
      BufferRequirements requirements, PixelFormat pix_fmt,
      uint32_t width, uint32_t height, uint32_t stride) {
    std::list<std::shared_ptr<BufferHandler> > allocated_bffers;

    uint32_t num_planes = 0;

    switch (pix_fmt) {
      case kRawBggrMipi8:
      case kRawGbrgMipi8:
      case kRawGrbgMipi8:
      case kRawRggbMipi8:
        num_planes = 1;
        break;
      case kRawBggrMipi10:
      case kRawGbrgMipi10:
      case kRawGrbgMipi10:
      case kRawRggbMipi10:
        num_planes = 1;
        break;
      case kRawBggrMipi12:
      case kRawGbrgMipi12:
      case kRawGrbgMipi12:
      case kRawRggbMipi12:
        num_planes = 1;
        break;
      case kRawBggr10:
      case kRawGbrg10:
      case kRawGrbg10:
      case kRawRggb10:
      case kRawBggr12:
      case kRawGbrg12:
      case kRawGrbg12:
      case kRawRggb12:
      case kRawBggr16:
      case kRawGbrg16:
      case kRawGrbg16:
      case kRawRggb16:
        num_planes = 1;
        break;
      case kNv12:
      case kNv12UBWC:
      case kNv21:
      case kNv21UBWC:
        num_planes = 2;
        break;
      case kJpeg:
        num_planes = 1;
        break;
      default:
        Utils::ThrowException(__func__, "Not supported pixel format");
    }

    std::vector<BufferPlane> planes;

    uint32_t size = 0;
    uint32_t plane_alignment = Utils::LCM(requirements.plane_alignment_,
                                              requirements.stride_alignment_);

    for (uint32_t i = 0; i < num_planes; i++) {
      uint32_t plane_height = height / (i + 1);
      uint32_t plane_size =
          Utils::MakeDivisibleBy(stride * plane_height, plane_alignment);
      size += plane_size;

      planes.push_back(
          BufferPlane(width, plane_height, stride, 0, plane_size));
    }

    for (uint32_t j = 0; j < requirements.count_; j++) {
      uint8_t *vaddr = static_cast<uint8_t *>(malloc(size));
      if (nullptr == vaddr) {
        Utils::ThrowException(__func__, "cannot allocate memory");
      }

      size_t addr = (size_t)vaddr;
      for (uint32_t k = 0; k < planes.size(); k++) {
        addr = Utils::MakeDivisibleBy(addr, plane_alignment);
        planes[k].offset_ = addr - (size_t)vaddr;
        addr += planes[k].length_;
      }

      int64_t timestamp =
          std::chrono::duration_cast< std::chrono::milliseconds >
            (std::chrono::system_clock::now().time_since_epoch()).count();

      static uint32_t frame_number = 0;
      frame_number++;

      std::shared_ptr<BufferHandler> new_handler(
          new BufferHandler(vaddr, addr, size, true, pix_fmt, timestamp,
              frame_number, planes));
      allocated_bffers.push_back(new_handler);
    };

    return allocated_bffers;
  }

  /** GetAlgoBuff
   *
   * returns qmmf algo buffer data
   *
   * return: qmmf algo buffer
   **/
  AlgBuffer GetAlgoBuff() const {
    return AlgBuffer(vaddr_, fd_, size_, cached_, pix_fmt_, timestamp_,
        frame_number_, plane_);
  }
};

}; // namespace qmmf_alg_plugin

}; // namespace qmmf
