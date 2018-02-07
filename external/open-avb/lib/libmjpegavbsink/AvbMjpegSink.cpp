/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#define LOG_NDEBUG 0
#define LOG_TAG "AvbMjpegSink"

#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>

#include <cutils/properties.h>

#include <utils/misc.h>
#include <utils/Atomic.h>
#include <utils/Errors.h>
#include <utils/Log.h>

#include "AvbMjpegSink.h"
#include "GlSink.h"
#include "MjpegFrame.h"

#define BILLION 1000000000L
#define NUM_BUFS 10

using namespace android;

AvbMjpegSink::AvbMjpegSink()
    : mMutex(PTHREAD_MUTEX_INITIALIZER),
      mFrames(nullptr),
      mWriteIdx(0),
      mReadIdx(0),
      mFrameCount(0),
      mNumWaitingBufs(0),
      mNumReadyBufs(0),
      mDropData(true), // start to true to wait for next SOI
      mGlSink(new GlSink(this)) {
    pthread_mutex_init(&mMutex, nullptr);
    InitJpegInputBufs();
}

AvbMjpegSink::~AvbMjpegSink() {
    for(int i = 0; i < NUM_BUFS; i++) {
        mFrames[i]->requestExit();
        mFrames[i] = nullptr;
    }
    delete[] mFrames;

    mGlSink->requestExit();
    mGlSink->FrameReady();
    mGlSink = nullptr;
}

/**
 *
 */
inline bool AvbMjpegSink::InitJpegInputBufs() {
    mFrames = new sp<MjpegFrame>[NUM_BUFS];
    for(int i = 0; i < NUM_BUFS; i++) {
        mFrames[i] = new MjpegFrame(this);
    }

    printf("AvbMjpegSink: buffer initialization completed\n");
    return true;
}

MjpegFrame* AvbMjpegSink::GetNextReadyFrame() {
    return mFrames[mReadIdx].get();
}

int AvbMjpegSink::GetNumReadyFrames() {
    return mNumReadyBufs;
}

void AvbMjpegSink::DecodeComplete() {
    if( 0 != pthread_mutex_lock(&mMutex)) {
        return;
    }

    mNumReadyBufs++;
    mNumWaitingBufs--;

    pthread_mutex_unlock(&mMutex);
    mGlSink->FrameReady();
}

void AvbMjpegSink::FramePosted() {
    if( 0 != pthread_mutex_lock(&mMutex)) {
        return;
    }

    mNumReadyBufs--;
    mReadIdx = (mReadIdx + 1) % NUM_BUFS;

    pthread_mutex_unlock(&mMutex);
}

/**
 *
 */
int AvbMjpegSink::DataSink(char *pBuf, int size, int endOfSegment) {
    if( size <= 0 ) {
        printf("AvbMjpegSink: ERROR. JPEG buffer len is invalid: %d bytes\n", size);
        return 0;
    }

    MjpegFrame* currFrame = mFrames[mWriteIdx].get();
    bool ready = (currFrame->GetState() == MjpegFrame::WAITING_FOR_DATA);

    // Check if the current buffer is ready to receive data
    if (!mDropData && !ready) {
        // Start dropping data until we have a free buffer
        mDropData = true;
        printf("AvbMjpegSink: No empty buffers!, mNumWaitingBufs=%d, mNumReadyBufs=%d\n",
                mNumWaitingBufs, mNumReadyBufs);
        return 0;
    }

    // Continue to drop data until next start of frame and free buffer
    if (mDropData) {
        if (ready && ((pBuf[0] == 0xff) && (pBuf[1] == 0xd8))) {
            // Got the beginning of the next frame
            // Reset current buffer and stop dropping data.
            currFrame->Reset();
            mDropData = false;
        } else {
            // else keep dropping
            return 0;
        }
    }

    // try to copy data into buffer
    if (!currFrame->AppendJpegData(pBuf, size)) {
        // Failed to copy data, reset and wait for next frame.
        currFrame->Reset();    // set pointer to starting of the buffer
        mDropData = true;
        return 0;
    }

    // If this is the last segment for this frame, push it to the decoder
    if (endOfSegment) {
        if( 0 != pthread_mutex_lock(&mMutex)) {
            printf("AvbMjpegSink::DataSink - failed to aquire mutex\n");
            return -1;
        }
        mWriteIdx = (mWriteIdx + 1) % NUM_BUFS;
        mNumWaitingBufs++;
        currFrame->StartDecode(mFrameCount++);
        pthread_mutex_unlock(&mMutex);

#ifdef ENABLE_TIME_LOGGING
        struct timespec currTime;
        clock_gettime(CLOCK_MONOTONIC, &currTime);
        if (mFrameCount != 1) {
            double delta = BILLION * (currTime.tv_sec - mLastSegment.tv_sec)
                + (currTime.tv_nsec - mLastSegment.tv_nsec);
            printf("AvbMjpegSink: Frame:%d, size=%d, tranfer time = %.3lf msec, rate=%.3lf Bpms\n",
                currFrame->GetFrameNum(),
                currFrame->GetDataLen(),
                (delta/1000000),
                currFrame->GetDataLen() / (delta/1000000));
        }
        mLastSegment = currTime;

        printf("AvbMjpegSink::DataSink: Frame:%d, Buffs[pend:%d,rdy:%d,free=%d],\n",
            currFrame->GetFrameNum(), mNumWaitingBufs, mNumReadyBufs,
            (NUM_BUFS - (mNumWaitingBufs + mNumReadyBufs)));
#endif
    }

    return size;
}
