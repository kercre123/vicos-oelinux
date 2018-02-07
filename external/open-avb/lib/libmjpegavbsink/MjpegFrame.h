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

#ifndef MJPEG_FRAME_H
#define MJPEG_FRAME_H

#include <stdint.h>
#include <sys/types.h>

#include <utils/Thread.h>

//#define ENABLE_TIME_LOGGING

class SkBitmap;

namespace android {

class MjpegFrame;
class AvbMjpegSink;

/**
 *
 */
class MjpegFrame : public Thread {
public:
    enum FrameState {
        WAITING_FOR_DATA,
        PENDING_DECODE,
        DECODE_ERROR,
        READY_TO_POST
    };

    MjpegFrame(AvbMjpegSink* parent);
    ~MjpegFrame();
    virtual bool threadLoop();

    void Reset();
    bool AppendJpegData(char *pBuf, int size);
    void StartDecode(int frameNum);

    MjpegFrame::FrameState GetState() { return mState; };
    void SetState(MjpegFrame::FrameState state) { mState = state; };

    int GetFrameNum() { return mFrameNum; };
    void SetFrameNum(int num) { mFrameNum = num; };

    int GetDataLen() { return mJpegDataLen; };

    SkBitmap& GetDecodedBitmap() { return mBitmap; };

private:
    wp<AvbMjpegSink> mParent;
    FrameState mState;
    unsigned char *mJpegData;
    unsigned int mJpegDataLen;
    unsigned int mFrameNum;
    SkBitmap mBitmap;
};

}; // namespace android

#endif // MJPEG_FRAME_H
