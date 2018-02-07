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
#define LOG_TAG "AvbJpegDecoder"

#include <stdint.h>
#include <sys/types.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <cutils/properties.h>
#include <binder/IPCThreadState.h>
#include <utils/misc.h>
#include <utils/Atomic.h>
#include <utils/Errors.h>
#include <utils/Log.h>

#include <SkBitmap.h>
#include <SkStream.h>
#ifdef USE_SK_CODEC
#include <SkCodec.h>
#else
#include <SkImageDecoder.h>
#endif

#include "AvbMjpegSink.h"
#include "GlSink.h"
#include "MjpegFrame.h"

#define BILLION 1000000000L
#define JPEG_BUF_SIZE 600000 //600KB - should match frame size in interface

using namespace android;

MjpegFrame::MjpegFrame(AvbMjpegSink* parent)
    : Thread(false),
      mParent(parent),
      mState(WAITING_FOR_DATA),
      mJpegData(new unsigned char[JPEG_BUF_SIZE]),
      mJpegDataLen(0),
      mFrameNum(0) {
}

MjpegFrame::~MjpegFrame() {
    delete[] mJpegData;
}

void MjpegFrame::Reset() {
    mState = WAITING_FOR_DATA;
    mJpegDataLen = 0;
}

void MjpegFrame::StartDecode(int frameNum) {
    mFrameNum = frameNum;
    mState = PENDING_DECODE;
    run("MjpegFrameDecode", PRIORITY_DISPLAY);
}

bool MjpegFrame::AppendJpegData(char *pBuf, int size) {
    // Check if we got more data than would fit in our buffer
    if (size > (int)(JPEG_BUF_SIZE - mJpegDataLen)) {
        printf( "MjpegFrame::AppendJpegData: ERROR: Buffer too small for frame data\n");
        return false;
    }

    // copy data into buffer
    memcpy(mJpegData + mJpegDataLen, pBuf, size);
    mJpegDataLen += size;
    return true;
}

/**
 *
 */
bool MjpegFrame::threadLoop() {
#ifdef ENABLE_TIME_LOGGING
    double decodeTime;
    struct timespec start_ts, end_ts;
    clock_gettime(CLOCK_MONOTONIC, &start_ts);
#endif


#ifdef USE_SK_CODEC
    // For newer versions of skia library use SkCodec to decode jpeg data
    sk_sp<SkData> skData = SkData::MakeWithoutCopy(mJpegData, mJpegDataLen);
    std::unique_ptr<SkCodec> skCodec(SkCodec::NewFromData(skData));

    SkImageInfo info = skCodec->getInfo().makeColorType(kN32_SkColorType);
    if (!mBitmap.tryAllocPixels(info)) {
        printf("MjpegFrame::Frame %d failed to alloc skBitmap pixels \n",
                mFrameNum);
        return false;
    }
    int status = skCodec->getPixels(info, mBitmap.getPixels(), mBitmap.rowBytes());

    if (status != 0) {
        printf("MjpegFrame::Frame %d decoding failed, status = %d \n",
                mFrameNum, status);
        mState = DECODE_ERROR;
    } else {
        mState = READY_TO_POST;
    }
#else
    // For older versions of skia library use SkImageDecoder to decode jpeg data
    int status = SkImageDecoder::DecodeMemory(
            mJpegData,
            mJpegDataLen,
            &mBitmap,
            kUnknown_SkColorType,
            SkImageDecoder::kDecodePixels_Mode);

    if (!status) {
        printf("MjpegFrame::Frame %d decoding failed, status = %d \n",
                mFrameNum, status);
        mState = DECODE_ERROR;
    } else {
        mState = READY_TO_POST;
    }
#endif

#ifdef ENABLE_TIME_LOGGING
    // record end of decoding
    clock_gettime(CLOCK_MONOTONIC, &end_ts);
    decodeTime = BILLION * (end_ts.tv_sec - start_ts.tv_sec)
            + (end_ts.tv_nsec - start_ts.tv_nsec);
    printf("MjpegFrame-DecodeComplete: Frame:%d, WxH=%ix%i, color-type %i, JpegSize=%d, Dec time = %.3lf msec (%.3lf Bpms)\n",
            mFrameNum,
            mBitmap.width(),
            mBitmap.height(),
            mBitmap.colorType(),
            mJpegDataLen,
            (decodeTime/1000000),
            mJpegDataLen / (decodeTime/1000000));
#endif

    sp<AvbMjpegSink> parent = mParent.promote();
    if (parent != nullptr && !exitPending()) {
        parent->DecodeComplete();
    }

    return false;
}


