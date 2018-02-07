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

#define TAG "AudioTest"

#include "common/audio/test/samples/qmmf_audio_test.h"

#include <cassert>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "common/audio/test/samples/qmmf_audio_test_ion.h"
#include "common/qmmf_log.h"

namespace qmmf_test {
namespace common {
namespace audio {

using ::qmmf::AudioFormat;
using ::qmmf::AudioDeviceId;
using ::qmmf::DeviceId;
using ::qmmf::common::audio::AudioBuffer;
using ::qmmf::common::audio::AudioEndPoint;
using ::qmmf::common::audio::AudioEndPointType;
using ::qmmf::common::audio::AudioEventHandler;
using ::qmmf::common::audio::AudioMetadata;
using ::qmmf::common::audio::AudioEventType;
using ::qmmf::common::audio::AudioEventData;
using ::qmmf::common::audio::BufferFlags;
using ::std::cin;
using ::std::condition_variable;
using ::std::cout;
using ::std::endl;
using ::std::mutex;
using ::std::queue;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

static const char* kDefaultFilePrefix = "/data/misc/qmmf/audio_test";
static const int kDefaultNumberOfBuffers = 4;

AudioTest::AudioTest()
    : buffer_size_(0),
      buffer_number_(kDefaultNumberOfBuffers),
      filename_prefix_(kDefaultFilePrefix),
      thread_(nullptr) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_INFO("%s: %s() test instantiated", TAG, __func__);
}

AudioTest::~AudioTest() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_INFO("%s: %s() test destroyed", TAG, __func__);
}

void AudioTest::Connect() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  AudioEventHandler audio_handler =
    [this] (const AudioEventType event_type,
            const AudioEventData& event_data) -> void {
      switch (event_type) {
        case AudioEventType::kError:
          ErrorHandler(event_data.error);
          break;
        case AudioEventType::kBuffer:
          BufferHandler(event_data.buffer);
          break;
      }
    };

  int result = end_point_.Connect(audio_handler);
  assert(result == 0);
}

void AudioTest::Disconnect() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  int32_t result = end_point_.Disconnect();
  assert(result == 0);

  result = ion_.Deallocate();
  assert(result == 0);
}

void AudioTest::ConfigureSource() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  int32_t result;

  type_ = AudioEndPointType::kSource;

  vector<DeviceId> devices;
  devices.push_back(static_cast<int32_t>(AudioDeviceId::kBuiltIn));

  AudioMetadata metadata;
  memset(&metadata, 0x0, sizeof metadata);
  metadata.format = AudioFormat::kPCM;
  metadata.num_channels = 1;
  metadata.sample_rate = 48000;
  metadata.sample_size = 16;

  result = wav_.Configure(filename_prefix_, type_, &metadata);
  assert(result == 0);

  result = end_point_.Configure(type_, devices, metadata);
  assert(result == 0);

  int32_t latency;
  result = end_point_.GetLatency(&latency);
  assert(result == 0);
  QMMF_INFO("%s: %s() latency is %d", TAG, __func__, latency);

  result = end_point_.GetBufferSize(&buffer_size_);
  assert(result == 0);
  QMMF_INFO("%s: %s() buffer_size is %d", TAG, __func__, buffer_size_);

  result = ion_.Allocate(buffer_number_, buffer_size_);
  if (result < 0) {
    result = ion_.Deallocate();
    assert(false);
  }
}

void AudioTest::ConfigureSink() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  int32_t result;

  type_ = AudioEndPointType::kSink;

  vector<DeviceId> devices;
  devices.push_back(0);

  AudioMetadata metadata;
  memset(&metadata, 0x0, sizeof metadata);
  result = wav_.Configure(filename_prefix_, type_, &metadata);
  assert(result == 0);

  result = end_point_.Configure(type_, devices, metadata);
  assert(result == 0);

  int32_t latency;
  result = end_point_.GetLatency(&latency);
  assert(result == 0);
  QMMF_INFO("%s: %s() latency is %d", TAG, __func__, latency);

  result = end_point_.GetBufferSize(&buffer_size_);
  assert(result == 0);
  QMMF_INFO("%s: %s() buffer_size is %d", TAG, __func__, buffer_size_);

  result = ion_.Allocate(buffer_number_, buffer_size_);
  if (result < 0) {
    result = ion_.Deallocate();
    assert(false);
  }
}

void AudioTest::Start() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  assert(thread_ == nullptr);

  int result = end_point_.Start();
  assert(result == 0);

  while (!messages_.empty())
    messages_.pop();

  thread_ = new thread(AudioTest::StaticThreadEntry, this);
  assert(thread_ != nullptr);
}

void AudioTest::Stop() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  AudioMessage message;
  message.type = AudioMessageType::kMessageStop;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  int32_t result = end_point_.Stop(false);
  assert(result == 0);

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  while (!messages_.empty())
    messages_.pop();
}

void AudioTest::Pause() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  AudioMessage message;
  message.type = AudioMessageType::kMessagePause;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  int32_t result = end_point_.Pause();
  assert(result == 0);
}

void AudioTest::Resume() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  int32_t result = end_point_.Resume();
  assert(result == 0);

  AudioMessage message;
  message.type = AudioMessageType::kMessageResume;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();
}

void AudioTest::ErrorHandler(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  assert(false);
}

void AudioTest::BufferHandler(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());

  AudioMessage message;
  message.type = AudioMessageType::kMessageBuffer;
  message.buffer = buffer;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();
}

void AudioTest::StaticThreadEntry(AudioTest* test) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  test->ThreadEntry();
}

void AudioTest::ThreadEntry() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  wav_.Open();

  switch (type_) {
    case AudioEndPointType::kSource: SourceThread(); break;
    case AudioEndPointType::kSink: SinkThread(); break;
  }

  wav_.Close();
}

void AudioTest::SourceThread() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  vector<AudioBuffer> buffers;
  bool paused = false;

  // send the initial list of buffers
  ion_.GetList(&buffers);
  int32_t result = end_point_.SendBuffers(buffers);
  assert(result == 0);
  buffers.clear();

  bool keep_running = true;
  bool stop_received = false;
  while (keep_running) {
    // wait until there is something to do
    if (buffers.empty() && messages_.empty()) {
      unique_lock<mutex> lk(message_lock_);
      signal_.wait(lk);
    }

    // process the next pending message
    message_lock_.lock();
    if (!messages_.empty()) {
      AudioMessage message = messages_.front();

      switch (message.type) {
        case AudioMessageType::kMessagePause:
          QMMF_DEBUG("%s: %s-MessagePause() TRACE", TAG, __func__);
          paused = true;
          break;

        case AudioMessageType::kMessageResume:
          QMMF_DEBUG("%s: %s-MessageResume() TRACE", TAG, __func__);
          paused = false;
          break;

        case AudioMessageType::kMessageStop:
          QMMF_DEBUG("%s: %s-MessageStop() TRACE", TAG, __func__);
          paused = false;
          stop_received = true;
          break;

        case AudioMessageType::kMessageBuffer:
          QMMF_DEBUG("%s: %s-MessageBuffer() TRACE", TAG, __func__);
          QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s] to queue[%u]",
                       TAG, __func__, message.buffer.ToString().c_str(),
                       buffers.size());
          buffers.push_back(message.buffer);
          QMMF_VERBOSE("%s: %s() buffers queue is now %u deep",
                       TAG, __func__, buffers.size());
          break;
      }
      messages_.pop();
    }
    message_lock_.unlock();

    if (!buffers.empty() && !paused && keep_running) {
      // write the data to file and reset the buffers
      for (AudioBuffer& buffer : buffers) {
        QMMF_VERBOSE("%s: %s() processing next buffer[%s] for queue[%u]",
                     TAG, __func__, buffer.ToString().c_str(), buffers.size());

        ion_.Associate(&buffer);
        wav_.Write(buffer);

        if (stop_received &&
            buffer.flags & static_cast<uint32_t>(BufferFlags::kFlagEOS))
          keep_running = false;

        memset(buffer.data, 0x00, buffer.capacity);
        buffer.size = 0;
        buffer.timestamp = 0;
      }

      // send the buffers
      int32_t result = end_point_.SendBuffers(buffers);
      assert(result == 0);
      buffers.clear();
    }
  }
}

void AudioTest::SinkThread() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  vector<AudioBuffer> buffers;
  bool paused = false;
  bool keep_running = true;

  ion_.GetList(&buffers);
  for (AudioBuffer& buffer : buffers) {
    int32_t result = wav_.Read(&buffer);
    if (result == AudioTestWav::kEOF) {
      keep_running = false;
      break;
    }
  }

  // send initial list of buffers
  int32_t result = end_point_.SendBuffers(buffers);
  assert(result == 0);
  buffers.clear();

  while (keep_running) {
    // wait until there is something to do
    if (buffers.empty() && messages_.empty()) {
      unique_lock<mutex> lk(message_lock_);
      signal_.wait(lk);
    }

    // process the next pending message
    message_lock_.lock();
    if (!messages_.empty()) {
      AudioMessage message = messages_.front();

      switch (message.type) {
        case AudioMessageType::kMessagePause:
          QMMF_DEBUG("%s: %s-MessagePause() TRACE", TAG, __func__);
          paused = true;
          break;

        case AudioMessageType::kMessageResume:
          QMMF_DEBUG("%s: %s-MessageResume() TRACE", TAG, __func__);
          paused = false;
          break;

        case AudioMessageType::kMessageStop:
          QMMF_DEBUG("%s: %s-MessageStop() TRACE", TAG, __func__);
          paused = false;
          keep_running = false;
          break;

        case AudioMessageType::kMessageBuffer:
          QMMF_DEBUG("%s: %s-MessageBuffer() TRACE", TAG, __func__);
          QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
                       message.buffer.ToString().c_str());
          buffers.push_back(message.buffer);
          break;
      }
      messages_.pop();
    }
    message_lock_.unlock();

    if (!buffers.empty() && !paused && keep_running) {
      // reset the buffers and read data from file
      for (AudioBuffer& buffer : buffers) {
        QMMF_VERBOSE("%s: %s() processing next buffer[%s]", TAG, __func__,
                     buffer.ToString().c_str());

        ion_.Associate(&buffer);
        memset(buffer.data, 0x00, buffer.capacity);
        buffer.size = 0;
        buffer.timestamp = 0;

        int32_t result = wav_.Read(&buffer);
        if (result == AudioTestWav::kEOF) {
          keep_running = false;
          break;
        }
      }

      // send the buffers
      int32_t result = end_point_.SendBuffers(buffers);
      assert(result == 0);
      buffers.clear();
    }
  }
}

void CommandMenu::PrintMenu() {
    cout << endl << endl;
    cout << "====== QMMF-SDK AUDIO TEST MENU ======" << endl;

    cout << static_cast<char>(Command::kConnect) << ". Connect" << endl;
    cout << static_cast<char>(Command::kDisconnect) << ". Disconnect" << endl;
    cout << static_cast<char>(Command::kConfigureSource) << ". Configure Source"
         << endl;
    cout << static_cast<char>(Command::kConfigureSink) << ". Configure Sink"
         << endl;
    cout << static_cast<char>(Command::kStart) << ". Start" << endl;
    cout << static_cast<char>(Command::kStop) << ". Stop" << endl;
    cout << static_cast<char>(Command::kPause) << ". Pause" << endl;
    cout << static_cast<char>(Command::kResume) << ". Resume" << endl;
    cout << static_cast<char>(Command::kExit) << ". Exit" << endl;

    cout << endl;
    cout << "Selection: ";
}

CommandMenu::Command CommandMenu::GetCommand() {
    PrintMenu();
    char selection;
    cin >> selection;
    return Command(static_cast<Command>(selection));
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf_test

using ::qmmf_test::common::audio::AudioTest;
using ::qmmf_test::common::audio::CommandMenu;
using ::std::cout;
using ::std::endl;

int main(const int argc, const char * const argv[]) {
  QMMF_INFO("%s: %s() TRACE", TAG, __func__);
  AudioTest test;
  CommandMenu menu;

  bool keep_running = true;
  while (keep_running) {
    CommandMenu::Command command = menu.GetCommand();

    switch(command) {
      case CommandMenu::Command::kConnect:
        test.Connect();
        break;
      case CommandMenu::Command::kDisconnect:
        test.Disconnect();
        break;
      case CommandMenu::Command::kConfigureSource:
        test.ConfigureSource();
        break;
      case CommandMenu::Command::kConfigureSink:
        test.ConfigureSink();
        break;
      case CommandMenu::Command::kStart:
        test.Start();
        break;
      case CommandMenu::Command::kStop:
        test.Stop();
        break;
      case CommandMenu::Command::kPause:
        test.Pause();
        break;
      case CommandMenu::Command::kResume:
        test.Resume();
        break;
      case CommandMenu::Command::kExit:
        test.Stop();
        test.Disconnect();
        keep_running = false;
        break;
      default:
        cout << endl;
        cout << "Invalid selection" << endl;
        break;
    }
  }
  return 0;
}
