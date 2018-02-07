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

#include "qmmf_simple_test_algo.h"

namespace qmmf {

namespace qmmf_alg_plugin {

/** QmmfResizerTestAlgo
 *
 * Test resizer algorithm
 *
 **/
class QmmfResizerTestAlgo : public QmmfSimpleTestAlgo {
 public:
  QmmfResizerTestAlgo() {
    caps_ = Capabilities(
        "ResizerTest",
        BufferRequirements(160, 120, 3840, 2160,
                           true, 1, 0, 0,
                           {kNv21, kNv12}),
        BufferRequirements(160, 120, 3840, 2160,
                           true, 1, 0, 0,
                           {}),
        false, 0, false, false, true, 1.0);
  }

  /** Process
   *    @input_buffers: vector of input buffers
   *    @output_buffers: vector of output buffers
   *
   * main qmmf algo function to process image data
   *
   * return: void
   **/
  void Process(const std::vector<AlgBuffer> &input_buffers,
               const std::vector<AlgBuffer> &output_buffers) {
    Validate(input_buffers, output_buffers);

    for (AlgBuffer b : output_buffers) {
      if (b.plane_.size() != input_buffers[0].plane_.size()) {
        Utils::ThrowException(
            __func__, "output planes are different than input planes");
      }

      for (size_t i = 0; i < b.plane_.size(); i++) {
        uint32_t width =
            std::min(b.plane_[i].width_, input_buffers[0].plane_[i].width_);
        uint32_t height =
            std::min(b.plane_[i].height_, input_buffers[0].plane_[i].height_);
        uint8_t *out_ptr = b.vaddr_ + b.plane_[i].offset_;
        uint8_t *in_ptr =
            input_buffers[0].vaddr_ + input_buffers[0].plane_[i].offset_;
        for (uint32_t h = 0; h < height; h++) {
          memcpy(out_ptr, in_ptr, width);
          out_ptr += b.plane_[i].stride_;
          in_ptr += input_buffers[0].plane_[i].stride_;
        }
      }
    }

    for (AlgBuffer b : input_buffers) {
      listener_->OnFrameProcessed(b);
    }
    for (AlgBuffer b : output_buffers) {
      listener_->OnFrameReady(b);
    }
  }
};

/** QmmfAlgoNew
 *    @calibration_data: calibration data
 *
 * Creates new algorithm instance
 *
 * return: shared pointer to new algorithm instance
 **/
extern "C" IAlgPlugin *QmmfAlgoNew(
    __attribute__((unused)) const std::vector<uint8_t> &calibration_data) {
  return new QmmfResizerTestAlgo();
}

}; // namespace qmmf_alg_plugin

}; // namespace qmmf
