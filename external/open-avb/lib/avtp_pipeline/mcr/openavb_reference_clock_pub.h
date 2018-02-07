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
* MODULE : Public interface for the reference clock
*/

#ifndef OPENAVB_REFERENCE_CLOCK_PUB_H
#define OPENAVB_REFERENCE_CLOCK_PUB_H

#include "openavb_platform_pub.h"
#include "openavb_types_base_pub.h"

/**
 * Callback to the reference clock
 *
 * This callback function is called whenever the reference clock "ticks".
 * The function must be registered by calling refClkRegisterObserver.
 *
 * \param context pointer that was passed into refClkRegisterObserver
 * \param timestamp time at which the tick occurred. Typically, this will be the
 *        AVTP time (number of nanoseconds since some arbitrary start time).
 * \param ticks the number of ticks that have elapsed since the last time
 *        time this function was called. This will almost always be 1. However,
 *        if the ticks were delayed for some reason, multiple ticks can be
 *        signalled with a single call by setting this to more than 1.
 * \param interval the number of events (e.g. audio samples) that the observer
 *        should process between each tick.
 * \param restart_clock true if the clock has been restarted and the observer
 *        should restart any counters that depend on the clock.
 */
typedef void (*openavb_ref_clk_observer_cb_t)(void *context, U64 timestamp,
	U16 ticks, U32 interval, bool restart_clock);

/**
 * Register an observer to the reference clock
 *
 * A module can register an observer to the reference clock to be notified of
 * each tick from the reference clock.
 *
 * \param callback the callback function that is called for each tick
 * \param context pointer that is passed into the callback function when the
 *        when the callback function is called. This pointer is also used to
 *        identify the observer in refClkUnregisterObserver so this should not
 *        be NULL
 * \return true if the callback function was successfully registered, false
 *        otherwise
 */
bool refClkRegisterObserver(openavb_ref_clk_observer_cb_t callback,
	void *context);

/**
 * Unregister an oberserver to the reference clock
 *
 * A module calls this function to unregister itself from the reference
 * clock.
 *
 * \param context pointer that was passed in as the context when registering
 *        the function. This is used to identify the callback
 * \return true if the function successfully unregisters the callback function,
 *        false otherwise
 */
bool refClkUnregisterObserver(void *context);

/**
 * Signal a tick to the observers
 *
 * This function is called by the clock source to notify the observers that
 * a tick has occurred.
 *
 * \param timestamp time at which the tick occurred. Typically, this will be the
 *        AVTP time (number of nanoseconds since some arbitrary start time)
 *        associated with the current time.
 * \param ticks the number of ticks that have elapsed since the the last time
 *        time this function was called. This will almost always be 1. However,
 *        if the ticks were delayed for some reason, multiple ticks can be
 *        signaled with a single call by setting this to more than 1.
 * \param interval the number of events (e.g. audio samples) that the observer
 *        should process between each tick.
 * \param restart_clock true if the clock has been restarted and the observer
 *        should restart any counters that depend on the clock.
 */
void refClkSignalTick(U64 timestamp, U16 ticks, U32 interval,
	bool restart_clock);

#endif // OPENAVB_REFERENCE_CLOCK_PUB_H
