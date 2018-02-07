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

#define LOG_TAG "QmmfAlgoInterfaceGtest"

#include <gtest/gtest.h>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include "qmmf-plugin/qmmf_alg_plugin.h"
#include "qmmf-plugin/qmmf_alg_utils.h"

#include "qmmf_algo_buffer_allocator.h"

namespace qmmf {

namespace qmmf_alg_plugin {

/** QmmfAlgoEventListener
 *    @lock_: mutex protection for thread safe
 *    @signal_: notification for end of processing
 *    @pending_input_buffers_: list of pending input buffers
 *    @pending_output_buffers_: list of pending input buffers
 *
 * Qmmf algorithm event listener
 *
 **/
class QmmfAlgoEventListener : public IEventListener {
 public:
  QmmfAlgoEventListener(
      std::list<std::shared_ptr<BufferHandler> > pending_input_buffers,
      std::list<std::shared_ptr<BufferHandler> > pending_output_buffers)
      : pending_input_buffers_(pending_input_buffers),
        pending_output_buffers_(pending_output_buffers),
        outout_file_index_(0) {}

  ~QmmfAlgoEventListener() {
    std::unique_lock<std::mutex> l(lock_);
    auto rc = signal_.wait_for(l, std::chrono::seconds(2), [&] {
      return (0 ==
              pending_input_buffers_.size() + pending_output_buffers_.size());
    });
    if (!rc) {
      Utils::ThrowException(__func__, "algorithm process timed out !!!");
    }
  }

  /** OnFrameProcessed
   *    @input_buffer: input buffer
   *
   * Indicates that input buffer is processed
   *
   * return: void
   **/
  void OnFrameProcessed(const AlgBuffer &input_buffer) override {
    std::unique_lock<std::mutex> l(lock_);

    pending_input_buffers_.remove_if(
        [&input_buffer](std::shared_ptr<BufferHandler> b) {
          return input_buffer.fd_ == b->GetAlgoBuff().fd_;
        });

    signal_.notify_one();
  }

  /** OnFrameReady
   *    @output_buffer: output buffer
   *
   * Indicates that output buffer is processed
   *
   * return: void
   **/
  void OnFrameReady(const AlgBuffer &output_buffer) override {
    std::unique_lock<std::mutex> l(lock_);

    pending_output_buffers_.remove_if(
        [&output_buffer](std::shared_ptr<BufferHandler> b) {
          return output_buffer.fd_ == b->GetAlgoBuff().fd_;
        });

    std::string complete_file_name = Utils::GetDataFolder() + "out_file_" +
                                     std::to_string(outout_file_index_++);
    std::ofstream output_file(complete_file_name, std::ios::binary);
    if (!output_file.is_open()) {
      Utils::ThrowException(__func__,
                                "cannot open file " + complete_file_name);
    }
    output_file.write(reinterpret_cast<char *>(output_buffer.vaddr_),
                      output_buffer.size_);

    signal_.notify_one();
  }

  /** OnError
   *    @err: error id
   *
   * Indicates runtime error
   *
   * return: void
   **/
  void OnError(RuntimeError err) override {
    ALOGE("%s: Error %d", __func__, err);
  }

 private:
  std::mutex lock_;
  std::condition_variable signal_;
  std::list<std::shared_ptr<BufferHandler> > pending_input_buffers_;
  std::list<std::shared_ptr<BufferHandler> > pending_output_buffers_;
  uint32_t outout_file_index_;
};

/** QmmfAlgoInterfaceGtest
 *    @test_info_: gtest info
 *    @kDefaultIterationCount: default iteration count if not set from persist
 *    @kPersistIterationCount: persist for selecting iteration count
 *    @kDefaultTestedLib: default tested lib name if not set from persist
 *    @kPersistTestedLib: persist for selecting tested lib name
 *    @kDefaultJsonConfiguration: default json configuration if not set from
 *       persist
 *    @kPersistJsonConfiguration: persist for selecting json configuration
 *    @kDefaultInPixFmt: default input pixel format if not set from persist
 *    @kPersistInPixFmt: persist for selecting input pixel format
 *    @kDefaultInWidth: default input width if not set from persist
 *    @kPersistInWidth: persist for selecting input width
 *    @kDefaultInHeight: default input height if not set from persist
 *    @kPersistInHeight: persist for selecting input height
 *    @kDefaultInStride: default input stride if not set from persist
 *    @kPersistInStride: persist for selecting input stride
 *    @kDefaultOutPixFmt: default output pixel format if not set from persist
 *    @kPersistOutPixFmt: persist for selecting output pixel format
 *    @kDefaultOutWidth: default output width if not set from persist
 *    @kPersistOutWidth: persist for selecting output width
 *    @kDefaultOutHeight: default output height if not set from persist
 *    @kPersistOutHeight: persist for selecting output height
 *    @kDefaultOutStride: default output stride if not set from persist
 *    @kPersistOutStride: persist for selecting output stride
 *    @kDefaultInputFileName: default input file name if not set from persist
 *    @kPersistInputFileName: persist for selecting input file name
 *    @persist_iteration_count_: iteration count
 *    @persist_tested_lib_: tested lib name
 *    @persist_config_json_data_: json configuration
 *    @persist_in_pix_fmt_: input pixel format
 *    @persist_in_width_: input width
 *    @persist_in_height_: input height
 *    @persist_out_pix_fmt_: output pixel format
 *    @persist_out_width_: output width
 *    @persist_out_height_: output height
 *    @persist_input_file_name_ : input file name
 *    @input_file_directory_: input file directory
 *    @lib_handle_: lib handle
 *    @algo_: algorithm instance
 *
 * Qmmf algorithm test
 *
 **/
class QmmfAlgoInterfaceGtest : public ::testing::Test {
 public:
  QmmfAlgoInterfaceGtest()
      : test_info_(nullptr),
        persist_iteration_count_(kDefaultIterationCount),
        persist_tested_lib_(kDefaultTestedLib),
        persist_config_json_data_(kDefaultJsonConfiguration),
        persist_in_pix_fmt_(kDefaultInPixFmt),
        persist_in_width_(kDefaultInWidth),
        persist_in_height_(kDefaultInHeight),
        persist_out_pix_fmt_(kDefaultOutPixFmt),
        persist_out_width_(kDefaultOutWidth),
        persist_out_height_(kDefaultOutHeight),
        persist_input_file_name_(kDefaultInputFileName),
        lib_handle_(nullptr),
        algo_(nullptr) {
    input_file_directory_ = Utils::GetDataFolder();
  };

  ~QmmfAlgoInterfaceGtest() { Deinit(); };

 protected:
  const ::testing::TestInfo *test_info_;

  static const uint32_t      kDefaultIterationCount;
  static const std::string   kPersistIterationCount;
  static const std::string   kDefaultTestedLib;
  static const std::string   kPersistTestedLib;
  static const std::string   kDefaultJsonConfiguration;
  static const std::string   kPersistJsonConfiguration;
  static const PixelFormat   kDefaultInPixFmt;
  static const std::string   kPersistInPixFmt;
  static const uint32_t      kDefaultInWidth;
  static const std::string   kPersistInWidth;
  static const uint32_t      kDefaultInHeight;
  static const std::string   kPersistInHeight;
  static const uint32_t      kDefaultInStride;
  static const std::string   kPersistInStride;
  static const PixelFormat   kDefaultOutPixFmt;
  static const std::string   kPersistOutPixFmt;
  static const uint32_t      kDefaultOutWidth;
  static const std::string   kPersistOutWidth;
  static const uint32_t      kDefaultOutHeight;
  static const std::string   kPersistOutHeight;
  static const uint32_t      kDefaultOutStride;
  static const std::string   kPersistOutStride;
  static const std::string   kDefaultInputFileName;
  static const std::string   kPersistInputFileName;

  uint32_t                   persist_iteration_count_;
  std::string                persist_tested_lib_;
  std::string                persist_config_json_data_;
  PixelFormat                persist_in_pix_fmt_;
  uint32_t                   persist_in_width_;
  uint32_t                   persist_in_height_;
  uint32_t                   persist_in_stride_;
  PixelFormat                persist_out_pix_fmt_;
  uint32_t                   persist_out_width_;
  uint32_t                   persist_out_height_;
  uint32_t                   persist_out_stride_;
  std::string                persist_input_file_name_;

  std::string                input_file_directory_;

  void *lib_handle_;
  IAlgPlugin *algo_;

  /** SetUp
  *
  * Gtest setup
  *
  * return: void
  **/
  void SetUp() override {
    test_info_ = ::testing::UnitTest::GetInstance()->current_test_info();

    persist_iteration_count_ =
        Utils::GetProperty(kPersistIterationCount, kDefaultIterationCount);

    persist_tested_lib_ =
        Utils::GetProperty(kPersistTestedLib, kDefaultTestedLib);
    fprintf(stderr, "\n++++++++++++ Tested Library %s ++++++++++++\n",
            persist_tested_lib_.c_str());

    persist_config_json_data_ = Utils::GetProperty(
        kPersistJsonConfiguration, kDefaultJsonConfiguration);
    fprintf(stderr, "Json Configuration %s\n",
            persist_config_json_data_.c_str());

    persist_in_width_ =
        Utils::GetProperty(kPersistInWidth, kDefaultInWidth);
    persist_in_height_ =
        Utils::GetProperty(kPersistInHeight, kDefaultInHeight);
    persist_in_stride_ =
        Utils::GetProperty(kPersistInStride, kDefaultInStride);
    fprintf(stderr, "Input frame resolution %dx%d stride %d format %d\n",
            persist_in_width_, persist_in_height_,
            persist_in_stride_, persist_in_pix_fmt_);

    persist_out_width_ =
        Utils::GetProperty(kPersistInWidth, kDefaultOutWidth);
    persist_out_height_ =
        Utils::GetProperty(kPersistOutHeight, kDefaultOutHeight);
    persist_out_stride_ =
        Utils::GetProperty(kPersistOutStride, kDefaultOutStride);
    fprintf(stderr, "Output frame resolution %dx%d stride %d format %d\n",
            persist_out_width_, persist_out_height_,
            persist_out_stride_, persist_in_pix_fmt_);

    persist_input_file_name_ =
        Utils::GetProperty(kPersistInputFileName, kDefaultInputFileName);
  };

  /** SetUp
  *
  * Gtest tear down
  *
  * return: void
  **/
  void TearDown() override{};

  /** Init
  *    @calibration_data: calibration data
  *
  * Initialize current test
  *
  * return: standard errors
  **/
  int32_t Init() {
    try {
      Utils::LoadLib(persist_tested_lib_, lib_handle_);

      QmmfAlgLoadPlugin LoadPluginFunc;
      Utils::LoadLibHandler(lib_handle_, "QmmfAlgoNew", LoadPluginFunc);

      std::vector<uint8_t> dummy;
      algo_ = LoadPluginFunc(dummy);
    } catch (const std::exception &e) {
      Deinit();
      throw e;
    }

    return 0;
  }

  /** Deinit
  *
  * Deinitialize current test
  *
  * return: standard errors
  **/
  int32_t Deinit() {
    if (nullptr != algo_) {
      delete algo_;
      algo_ = nullptr;
    }

    Utils::UnloadLib(lib_handle_);

    return 0;
  }
};

const uint32_t QmmfAlgoInterfaceGtest::kDefaultIterationCount = 10;
const std::string QmmfAlgoInterfaceGtest::kPersistIterationCount =
    "persist.qmmf.algo.gtest.iter";
const std::string QmmfAlgoInterfaceGtest::kDefaultTestedLib =
    "libqmmf_test_algo_outplace.so";
const std::string QmmfAlgoInterfaceGtest::kPersistTestedLib =
    "persist.qmmf.algo.gtest.lib";
const std::string QmmfAlgoInterfaceGtest::kDefaultJsonConfiguration =
    "{TestJson:\" now \"}";
const std::string QmmfAlgoInterfaceGtest::kPersistJsonConfiguration =
    "persist.qmmf.algo.gtest.conf";

const PixelFormat QmmfAlgoInterfaceGtest::kDefaultInPixFmt = kNv12;
const std::string QmmfAlgoInterfaceGtest::kPersistInPixFmt =
    "persist.qmmf.algo.gtest.inf";
const uint32_t QmmfAlgoInterfaceGtest::kDefaultInWidth = 3840;
const std::string QmmfAlgoInterfaceGtest::kPersistInWidth =
    "persist.qmmf.algo.gtest.inw";
const uint32_t QmmfAlgoInterfaceGtest::kDefaultInHeight = 2160;
const std::string QmmfAlgoInterfaceGtest::kPersistInHeight =
    "persist.qmmf.algo.gtest.inh";
const uint32_t QmmfAlgoInterfaceGtest::kDefaultInStride = 3840;
const std::string QmmfAlgoInterfaceGtest::kPersistInStride =
    "persist.qmmf.algo.gtest.ins";

const PixelFormat QmmfAlgoInterfaceGtest::kDefaultOutPixFmt = kNv12;
const std::string QmmfAlgoInterfaceGtest::kPersistOutPixFmt =
    "persist.qmmf.algo.gtest.outf";
const uint32_t QmmfAlgoInterfaceGtest::kDefaultOutWidth = 3840;
const std::string QmmfAlgoInterfaceGtest::kPersistOutWidth =
    "persist.qmmf.algo.gtest.outw";
const uint32_t QmmfAlgoInterfaceGtest::kDefaultOutHeight = 2160;
const std::string QmmfAlgoInterfaceGtest::kPersistOutHeight =
    "persist.qmmf.algo.gtest.outh";
const uint32_t QmmfAlgoInterfaceGtest::kDefaultOutStride = 3840;
const std::string QmmfAlgoInterfaceGtest::kPersistOutStride =
    "persist.qmmf.algo.gtest.outs";


const std::string QmmfAlgoInterfaceGtest::kDefaultInputFileName = "in_file_";
const std::string QmmfAlgoInterfaceGtest::kPersistInputFileName =
    "persist.qmmf.algo.gtest.file";

/*
* NewInstance: This test case will test new instance API.
* Api test sequence:
*  - new instance
*  - destroy instance
*/
TEST_F(QmmfAlgoInterfaceGtest, NewInstance) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= persist_iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, persist_iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto ret = Init();
    assert(ret == 0);

    ret = Deinit();
    assert(ret == 0);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* GetCaps: This test case will test GetCaps API.
* Api test sequence:
*  - new instance
*  - GetCaps
*  - destroy instance
*/
TEST_F(QmmfAlgoInterfaceGtest, GetCaps) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= persist_iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, persist_iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto ret = Init();
    assert(ret == 0);

    const auto caps = algo_->GetCaps();
    ALOGD("%s: Algorithm Capabilities =\n%s\n", __func__,
          caps.ToString().c_str());

    ret = Deinit();
    assert(ret == 0);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Configure: This test case will test Configure API
* Api test sequence:
*  - new instance
*  - Configure
*  - destroy instance
*/
TEST_F(QmmfAlgoInterfaceGtest, Configure) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= persist_iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, persist_iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto ret = Init();
    assert(ret == 0);

    algo_->Configure(persist_config_json_data_);

    ret = Deinit();
    assert(ret == 0);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* RegisterUnregisterInputBuffers: This test case will test
*   RegisterUnregisterInputBuffers API.
* Api test sequence:
*  - new instance
*  - RegisterInputBuffers
*  - UnregisterInputBuffers
*  - destroy instance
*/
TEST_F(QmmfAlgoInterfaceGtest, RegisterUnregisterInputBuffers) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= persist_iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, persist_iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto ret = Init();
    assert(ret == 0);

    const auto caps = algo_->GetCaps();

    for (auto pix_fmt : caps.in_buffer_requirements_.pixel_formats_) {
      std::vector<AlgBuffer> buffers;

      auto buffer_handlers = BufferHandler::New(caps.in_buffer_requirements_,
          pix_fmt, persist_in_width_, persist_in_height_, persist_in_stride_);
      for (auto buffer_handler : buffer_handlers) {
        AlgBuffer b = buffer_handler->GetAlgoBuff();
        buffers.push_back(b);
      }

      algo_->RegisterInputBuffers(buffers);
      algo_->UnregisterInputBuffers(buffers);
    }

    ret = Deinit();
    assert(ret == 0);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* RegisterUnregisterOutputBuffers: This test case will test
*   RegisterUnregisterOutputBuffers API.
* Api test sequence:
*  - new instance
*  - RegisterOutputBuffers
*  - UnregisterOutputBuffers
*  - destroy instance
*/
TEST_F(QmmfAlgoInterfaceGtest, RegisterUnregisterOutputBuffers) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= persist_iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, persist_iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto ret = Init();
    assert(ret == 0);

    const auto caps = algo_->GetCaps();

    for (auto pix_fmt : caps.out_buffer_requirements_.pixel_formats_) {
      std::vector<AlgBuffer> buffers;

      auto buffer_handlers = BufferHandler::New(caps.out_buffer_requirements_,
          pix_fmt, persist_in_width_, persist_in_height_, persist_in_stride_);
      for (auto buffer_handler : buffer_handlers) {
        AlgBuffer b = buffer_handler->GetAlgoBuff();
        buffers.push_back(b);
      }

      algo_->RegisterOutputBuffers(buffers);
      algo_->UnregisterOutputBuffers(buffers);
    }

    ret = Deinit();
    assert(ret == 0);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Process: This test case will test Process API.
* Api test sequence:
*  - new instance
*  - RegisterInputBuffers
*  - RegisterOutputBuffers
*  - Process
*  - UnregisterInputBuffers
*  - UnregisterOutputBuffers
*  - destroy instance
*/
TEST_F(QmmfAlgoInterfaceGtest, Process) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= persist_iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, persist_iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto ret = Init();
    assert(ret == 0);

    const auto caps = algo_->GetCaps();

    algo_->Configure(persist_config_json_data_);

    std::vector<AlgBuffer> input_buffers;
    auto input_buffer_handlers = BufferHandler::New(
        caps.in_buffer_requirements_, persist_in_pix_fmt_, persist_in_width_,
        persist_in_height_, persist_in_stride_);
    for (auto buffer_handler : input_buffer_handlers) {
      AlgBuffer b = buffer_handler->GetAlgoBuff();
      input_buffers.push_back(b);
    }

    std::vector<AlgBuffer> output_buffers;
    auto output_buffer_handlers = BufferHandler::New(
        caps.out_buffer_requirements_, persist_out_pix_fmt_,
        persist_out_width_, persist_out_height_, persist_out_stride_);
    for (auto buffer_handler : output_buffer_handlers) {
      AlgBuffer b = buffer_handler->GetAlgoBuff();
      output_buffers.push_back(b);
    }

    uint32_t j = 0;
    for (auto b : input_buffer_handlers) {
      std::string complete_file_name = input_file_directory_ +
                                       persist_input_file_name_ +
                                       std::to_string(j++);
      std::ifstream input_file(complete_file_name, std::ios::binary);
      if (!input_file.is_open()) {
        Utils::ThrowException(__func__,
                                  "cannot open file " + complete_file_name);
      }
      input_file.read(reinterpret_cast<char *>(b->GetAlgoBuff().vaddr_),
                      b->GetAlgoBuff().size_);
    }

    QmmfAlgoEventListener l(input_buffer_handlers, output_buffer_handlers);
    algo_->SetCallbacks(&l);

    algo_->RegisterInputBuffers(input_buffers);
    algo_->RegisterOutputBuffers(output_buffers);

    algo_->Process(input_buffers, output_buffers);

    algo_->UnregisterInputBuffers(input_buffers);
    algo_->UnregisterOutputBuffers(output_buffers);

    ret = Deinit();
    assert(ret == 0);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Abort: This test case will test Abort API.
* Api test sequence:
*  - new instance
*  - RegisterInputBuffers
*  - RegisterOutputBuffers
*  - Process
*  - Abort
*  - UnregisterInputBuffers
*  - UnregisterOutputBuffers
*  - destroy instance
*/
TEST_F(QmmfAlgoInterfaceGtest, Abort) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= persist_iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, persist_iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto ret = Init();
    assert(ret == 0);

    const auto caps = algo_->GetCaps();

    algo_->Configure(persist_config_json_data_);

    std::vector<AlgBuffer> input_buffers;
    auto input_buffer_handlers = BufferHandler::New(
        caps.in_buffer_requirements_, persist_in_pix_fmt_, persist_in_width_,
        persist_in_height_, persist_in_stride_);
    for (auto buffer_handler : input_buffer_handlers) {
      AlgBuffer b = buffer_handler->GetAlgoBuff();
      input_buffers.push_back(b);
    }

    std::vector<AlgBuffer> output_buffers;
    auto output_buffer_handlers = BufferHandler::New(
        caps.out_buffer_requirements_, persist_out_pix_fmt_,
        persist_out_width_, persist_out_height_, persist_out_stride_);
    for (auto buffer_handler : output_buffer_handlers) {
      AlgBuffer b = buffer_handler->GetAlgoBuff();
      output_buffers.push_back(b);
    }

    uint32_t j = 0;
    for (auto b : input_buffer_handlers) {
      std::string complete_file_name = input_file_directory_ +
                                       persist_input_file_name_ +
                                       std::to_string(j++);
      std::ifstream input_file(complete_file_name, std::ios::binary);
      if (!input_file.is_open()) {
        Utils::ThrowException(__func__,
                                  "cannot open file " + complete_file_name);
      }
      input_file.read(reinterpret_cast<char *>(b->GetAlgoBuff().vaddr_),
                      b->GetAlgoBuff().size_);
    }

    QmmfAlgoEventListener l(input_buffer_handlers, output_buffer_handlers);
    algo_->SetCallbacks(&l);

    algo_->RegisterInputBuffers(input_buffers);
    algo_->RegisterOutputBuffers(output_buffers);

    algo_->Abort();
    algo_->Process(input_buffers, output_buffers);

    algo_->UnregisterInputBuffers(input_buffers);
    algo_->UnregisterOutputBuffers(output_buffers);

    ret = Deinit();
    assert(ret == 0);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

}; // namespace qmmf_alg_plugin

}; // namespace qmmf
