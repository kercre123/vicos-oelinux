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

#define LOG_TAG "QmmfUtilsGtest"

#include <gtest/gtest.h>

#include "qmmf-plugin/qmmf_alg_plugin.h"
#include "qmmf-plugin/qmmf_alg_utils.h"

namespace qmmf {

namespace qmmf_alg_plugin {

/** QmmfAlgoUtilsGtest
 *    @test_info_: gtest info
 *    @kDefaultIterationCount:  default iteration count if not set from persist
 *    @iteration_count_: iteration count
 *
 * Qmmf algorithm test
 *
 **/
class QmmfAlgoUtilsGtest : public ::testing::Test {
 public:
  QmmfAlgoUtilsGtest() : test_info_(nullptr), iteration_count_(0){};

  ~QmmfAlgoUtilsGtest() { Deinit(); };

 protected:
  const ::testing::TestInfo* test_info_;
  static const uint32_t kDefaultIterationCount = 10;
  uint32_t iteration_count_;

  /** SetUp
  *
  * Gtest setup
  *
  * return: void
  **/
  void SetUp() override {
    test_info_ = ::testing::UnitTest::GetInstance()->current_test_info();

    iteration_count_ = Utils::GetProperty("persist.qmmf.algo.gtest.iter",
                                              kDefaultIterationCount);
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
  int32_t Init() { return 0; }

  /** Deinit
  *
  * Deinitialize current test
  *
  * return: standard errors
  **/
  int32_t Deinit() { return 0; }
};

/*
* ThrowException: This test case will test ThrowException API.
* Api test sequence:
*  - ThrowException
*/
TEST_F(QmmfAlgoUtilsGtest, ThrowException) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    try {
      Utils::ThrowException("title", "message");
    } catch (const std::exception& e) {
      std::stringstream expected_result;
      expected_result <<  "Error: title message";
      assert(0 == expected_result.str().compare(e.what()));
    }
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* MakeDivisibleBy: This test case will test MakeDivisibleBy API.
* Api test sequence:
*  - MakeDivisibleBy
*/
TEST_F(QmmfAlgoUtilsGtest, MakeDivisibleBy) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto res = Utils::MakeDivisibleBy(15, 2);
    assert(16 == res);
    res = Utils::MakeDivisibleBy(16, 2);
    assert(16 == res);
    res = Utils::MakeDivisibleBy(16, 0);
    assert(16 == res);
    res = Utils::MakeDivisibleBy(1226, 25);
    assert(1250 == res);
    res = Utils::MakeDivisibleBy(1225, 25);
    assert(1225 == res);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* GetWidthInBytes: This test case will test GetWidthInBytes API.
* Api test sequence:
*  - GetWidthInBytes
*/
TEST_F(QmmfAlgoUtilsGtest, GetWidthInBytes) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto res = Utils::GetWidthInBytes(16, 2.0);
    assert(32 == res);
    res = Utils::GetWidthInBytes(16, 1.5);
    assert(24 == res);
    res = Utils::GetWidthInBytes(16, 1.25);
    assert(20 == res);
    res = Utils::GetWidthInBytes(16, 1.1);
    assert(18 == res);
    res = Utils::GetWidthInBytes(16, 1.0);
    assert(16 == res);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* GCD: This test case will test GCD API.
* Api test sequence:
*  - GCD
*/
TEST_F(QmmfAlgoUtilsGtest, GCD) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto res = Utils::GCD(16, 2);
    assert(2 == res);
    res = Utils::GCD(16, 15);
    assert(1 == res);
    res = Utils::GCD(16, 8);
    assert(8 == res);
    res = Utils::GCD(16, 4096);
    assert(16 == res);
    res = Utils::GCD(16, 0);
    assert(16 == res);
    res = Utils::GCD(0, 16);
    assert(16 == res);
    res = Utils::GCD(1920 * 1080 * 3 / 2, 4096);
    assert(512 == res);
    res = Utils::GCD(3840 * 2160 * 3 / 2, 4096);
    assert(2048 == res);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* LCM: This test case will test LCM API.
* Api test sequence:
*  - LCM
*/
TEST_F(QmmfAlgoUtilsGtest, LCM) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    auto res = Utils::LCM(16, 2);
    assert(16 == res);
    res = Utils::LCM(16, 15);
    assert(240 == res);
    res = Utils::LCM(16, 8);
    assert(16 == res);
    res = Utils::LCM(16, 4096);
    assert(4096 == res);
    res = Utils::LCM(16, 0);
    assert(16 == res);
    res = Utils::LCM(0, 16);
    assert(16 == res);
    res = Utils::LCM(1920 * 1080 * 3 / 2, 4096);
    ALOGD("res=%d\n", res);
    assert(24883200 == res);
    res = Utils::LCM(3840 * 2160 * 3 / 2, 4096);
    ALOGD("res=%d\n", res);
    assert(24883200 == res);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Property: This test case will test Get Property API.
* Api test sequence:
*  - GetProperty
*  - SetProperty
*  - GetProperty
*/
TEST_F(QmmfAlgoUtilsGtest, Property) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    int test_int_default = 66;
    auto res_int = Utils::GetProperty("random_property", test_int_default);
    assert(res_int == test_int_default);

    int test_float_default = 666.0;
    auto res_float =
        Utils::GetProperty("random_property", test_float_default);
    assert(res_float == test_float_default);

    std::string test_string_default = "6";
    auto res_string =
        Utils::GetProperty("random_property", test_string_default);
    assert(res_string == test_string_default);

    int test_int_property = 22;
    Utils::SetProperty("test_property", test_int_property);
    res_int = Utils::GetProperty("test_property", test_int_default);
    assert(res_int == test_int_property);

    float test_float_property = 222.0;
    Utils::SetProperty("test_property", test_float_property);
    res_float = Utils::GetProperty("test_property", test_float_default);
    assert(res_float == test_float_property);

    std::string test_string_property = "2";
    Utils::SetProperty("test_property", test_string_property);
    res_string = Utils::GetProperty("test_property", test_string_default);
    assert(res_string == test_string_property);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* DllManipulation: This test case will test Get Property API.
* Api test sequence:
*  - LoadLib
*  - LoadLibHandler
*  - UnloadLib
*/
TEST_F(QmmfAlgoUtilsGtest, DllManipulation) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    ALOGD("%s: Running Test(%s) iteration = %d\n", __func__, test_info_->name(),
          i);

    void* lib_handle;
    try {
      Utils::LoadLib("libqmmf_test_algo_outplace.so", lib_handle);

      QmmfAlgLoadPlugin LoadPluginFunc;
      Utils::LoadLibHandler(lib_handle, "QmmfAlgoNew", LoadPluginFunc);

    } catch (const std::exception& e) {
      Utils::UnloadLib(lib_handle);
      fprintf(stderr, "%s\n", e.what());
      assert(0);
    }

    Utils::UnloadLib(lib_handle);
  }

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

}; // namespace qmmf_alg_plugin

}; // namespace qmmf
