/*******************************************************************************
Copyright (c) 2016, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*
* MODULE : Reference clock. Another module notifies this modules of ticks
* from some source, and this modules notifies other registered modules of the
* ticks.
*/

#include <stdlib.h>
#include "openavb_platform.h"
#include "openavb_trace.h"

// DEBUG Uncomment to turn on logging for just this module.
//#define AVB_LOG_ON	1

#define	AVB_LOG_COMPONENT	"RefClock"
#include "openavb_log.h"

#include "openavb_debug.h"

#include "openavb_reference_clock_pub.h"

#define MAX_OBSERVERS 10

#define DEBUG_MCR 1

// The list of observers is going to be access by multiple threads so
// a mutex is needed to protect the data
static MUTEX_HANDLE(reference_clock_mutex);
#define REF_CLK_LOCK() { \
	MUTEX_CREATE_ERR(); \
	MUTEX_LOCK(reference_clock_mutex); \
	MUTEX_LOG_ERR("Mutex Lock failure"); \
}

#define REF_CLK_UNLOCK() { \
	MUTEX_CREATE_ERR(); \
	MUTEX_UNLOCK(reference_clock_mutex); \
	MUTEX_LOG_ERR("Mutex Unlock failure"); \
}

typedef struct
{
	openavb_ref_clk_observer_cb_t callback; // Observer callback
	void *context; // Pointer provided when the callback was registered
} refClkObserver;

// The list of observers to the reference clock
static refClkObserver observers[MAX_OBSERVERS] = {{0}};
static U16 observer_count = 0;

// Register a new clock observer
bool refClkRegisterObserver(openavb_ref_clk_observer_cb_t callback,
	void *context)
{
	REF_CLK_LOCK();
	if (observer_count >= MAX_OBSERVERS) {
		REF_CLK_UNLOCK();
		return FALSE;
	}

	observers[observer_count].callback = callback;
	observers[observer_count].context = context;
	observer_count++;

	REF_CLK_UNLOCK();
	return TRUE;
}

// Unregister an existing clock observer
bool refClkUnregisterObserver(void *context)
{
	REF_CLK_LOCK();

	// Find the correct observer
	U16 index;
	for (index = 0; index < observer_count; ++index) {
		if (observers[index].context == context) {
			break;
		}
	}

	// If the observer is not found, return false
	if (index == observer_count) {
		REF_CLK_UNLOCK();
		return FALSE;
	}

	// The observer is found. Remove it and move all the
	// observers in the list after it forward one spot.
	++index;
	for (; index < observer_count; ++index) {
		observers[index - 1] = observers[index];
	}

	REF_CLK_UNLOCK();
	return TRUE;
}

// Notify all of the observers that a tick as occurred
void refClkSignalTick(U64 timestamp, U16 ticks, U32 interval,
	bool restart_clock)
{
	REF_CLK_LOCK();

#if DEBUG_MCR
	static U32 counter = 0;
	counter++;
	if (counter % 300 == 0) {
		AVB_LOGF_INFO("clocktick: %llu", timestamp);
	}
#endif

	U16 index;
	for (index = 0; index < observer_count; ++index) {
		observers[index].callback(observers[index].context,
			timestamp, ticks, interval, restart_clock);
	}

	REF_CLK_UNLOCK();
}
