/*
 * Copyright (c) 2016 The Linux Foundation. All rights reserved.
 * Not a Contribution.
 */

/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "recorder/src/service/qmmf_recorder_common.h"
#include "qmmf_camera3_device_client.h"
#include "qmmf_camera3_utils.h"
#include "qmmf_camera3_monitor.h"

using namespace android;

namespace qmmf {

namespace cameraadaptor {

Camera3Monitor::Camera3Monitor()
    : monitor_updated_(false),
      idle_notify_(nullptr),
      next_monitor_(0),
      composite_state_(IDLE) {
  pthread_mutex_init(&input_lock_, NULL);
  pthread_mutex_init(&lock_, NULL);
  pthread_cond_init(&input_signal_, NULL);
}

Camera3Monitor::~Camera3Monitor() {
  RequestExitAndWait();

  pthread_mutex_destroy(&input_lock_);
  pthread_mutex_destroy(&lock_);
  pthread_cond_destroy(&input_signal_);
}

int32_t Camera3Monitor::AcquireMonitor() {
  int32_t res, id;

  pthread_mutex_lock(&lock_);
  id = next_monitor_++;
  res = monitor_states_.add(id, IDLE);
  pthread_mutex_unlock(&lock_);
  if (0 > res) {
    QMMF_ERROR("%s: Cannot add new monitor %d: %s (%zd)\n", __func__, id,
               strerror(-res), res);
    return res;
  }

  res = id;

  if (0 <= res) {
    pthread_mutex_lock(&input_lock_);
    monitor_updated_ = true;
    pthread_cond_signal(&input_signal_);
    pthread_mutex_unlock(&input_lock_);
  }

  return res;
}

void Camera3Monitor::ReleaseMonitor(int32_t id) {
  int32_t idx;
  pthread_mutex_lock(&lock_);
  idx = monitor_states_.removeItem(id);
  pthread_mutex_unlock(&lock_);

  if (0 <= idx) {
    pthread_mutex_lock(&input_lock_);
    monitor_updated_ = true;
    pthread_cond_signal(&input_signal_);
    pthread_mutex_unlock(&input_lock_);
  }

  return;
}

void Camera3Monitor::ChangeStateToIdle(int32_t id) { ChangeState(id, IDLE); }

void Camera3Monitor::ChangeStateToActive(int32_t id) {
  ChangeState(id, ACTIVE);
}

void Camera3Monitor::ChangeState(int id, MonitorState state) {
  pthread_mutex_lock(&input_lock_);

  StateTransition newState;
  memset(&newState, 0, sizeof(newState));
  newState.id = id;
  newState.state = state;

  input_queue_.add(newState);
  pthread_cond_signal(&input_signal_);
  pthread_mutex_unlock(&input_lock_);
}

void Camera3Monitor::RequestExit() {
  pthread_cond_signal(&input_signal_);
  Camera3Thread::RequestExit();
}

void Camera3Monitor::RequestExitAndWait() {
  pthread_cond_signal(&input_signal_);
  Camera3Thread::RequestExitAndWait();
}

Camera3Monitor::MonitorState Camera3Monitor::BuildCompositeState() {
  for (uint32_t i = 0; i < monitor_states_.size(); i++) {
    if (monitor_states_.valueAt(i) == ACTIVE) {
      return ACTIVE;
    }
  }
  return IDLE;
}

bool Camera3Monitor::ThreadLoop() {
  int32_t res = 0;

  pthread_mutex_lock(&input_lock_);
  while (input_queue_.size() == 0 && !monitor_updated_) {
    res = cond_wait_relative(&input_signal_, &input_lock_, WAIT_TIMEOUT);
    if (ExitPending()) {
      pthread_mutex_unlock(&input_lock_);
      return false;
    }
    if (0 != res) {
      if (-ETIMEDOUT != res) {
        QMMF_ERROR("%s: Error during state change wait: %s (%d)\n", __func__,
                   strerror(res), res);
      }
      // timeouts can occur
      pthread_mutex_unlock(&input_lock_);
      break;
    }
  }
  pthread_mutex_unlock(&input_lock_);

  pthread_mutex_lock(&input_lock_);
  pthread_mutex_lock(&lock_);

  MonitorState oldState = BuildCompositeState();
  if (oldState != composite_state_) {
    updated_states_.add(oldState);
  }

  for (uint32_t i = 0; i < input_queue_.size(); i++) {
    const StateTransition &newState = input_queue_[i];
    int32_t idx = monitor_states_.indexOfKey(newState.id);
    if (idx >= 0) {
      monitor_states_.replaceValueAt(idx, newState.state);
      MonitorState newState = BuildCompositeState();
      if (newState != oldState) {
        updated_states_.add(newState);
      }
      oldState = newState;
    }
  }
  input_queue_.clear();
  monitor_updated_ = false;

  composite_state_ = oldState;
  pthread_mutex_unlock(&lock_);
  pthread_mutex_unlock(&input_lock_);

  if (!updated_states_.isEmpty()) {
    for (size_t i = 0; i < updated_states_.size(); i++) {
      bool idle = (updated_states_[i] == IDLE);
      if (nullptr != idle_notify_) {
        idle_notify_(idle);
      }
    }
  }
  updated_states_.clear();

  return true;
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
