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

#include "qmmf_condition.h"

uint32_t qmmf_log_level;

namespace qmmf {

#if (defined(_GLIBCXX_HAS_GTHREADS) && defined(_GLIBCXX_USE_C99_STDINT_TR1)) || (defined(_LIBCPP_THREADING_SUPPORT))

#if defined(__GTHREAD_COND_INIT) || defined(_LIBCPP_CONDVAR_INITIALIZER)
  QCondition::QCondition() noexcept = default;
#else
  QCondition::QCondition() noexcept {
    __GTHREAD_COND_INIT_FUNCTION(&cond_);
  }
#endif

  QCondition::~QCondition() noexcept {
    qthread_cond_destroy(&cond_);
  }

  void QCondition::Signal() {
    auto status = qthread_cond_signal(&cond_);
    assert(status == 0);
  }

  void QCondition::SignalAll() {
    auto status = qthread_cond_broadcast(&cond_);
    assert(status == 0);
  }

  void QCondition::Wait(std::unique_lock<std::mutex>& lock) {
    auto status = qthread_cond_wait(&cond_, lock.mutex()->native_handle());
    assert(status == 0);
  }

#endif // (_GLIBCXX_HAS_GTHREADS && _GLIBCXX_USE_C99_STDINT_TR1) || (_LIBCPP_THREADING_SUPPORT)

};  //namespace qmmf
