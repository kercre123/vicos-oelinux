/* Copyrights (c) 2016-2017, The Linux Foundation. All rights reserved.
 * "Not a Contribution."
 */

/*
 * Copyright (C) 2012 The Android Open Source Project
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

#define LOG_TAG "AVBh264Stream"
#include <utils/Log.h>

#include <inttypes.h>
#include <sys/types.h>
#include <pthread.h>
#include <semaphore.h>
#include <binder/IServiceManager.h>
#include <binder/ProcessState.h>
#include <media/ICrypto.h>
#include <media/stagefright/foundation/ABuffer.h>
#include <media/stagefright/foundation/ALooper.h>
#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/DataSource.h>
#include <media/stagefright/MediaCodec.h>
#include <media/stagefright/MediaCodecList.h>
#include <media/stagefright/MediaDefs.h>
#ifdef USE_MEDIA_CODEC_BUFFER
#include <media/MediaCodecBuffer.h>
#endif
#include <gui/ISurfaceComposer.h>
#include <gui/SurfaceComposerClient.h>
#include <gui/Surface.h>
#include <ui/DisplayInfo.h>

#include "Avbh264Stream.h"

using namespace android;

/******************************************************************************
 *                               Defines
 *****************************************************************************/
#define OUTPUT_DQ_TIMEOUT          20000ll // Can wait for output buff
#define INPUT_DQ_TIMEOUT             500ll // Shouldn't wait on input buff
#define INPUT_DQ_TIMEOUT_INCREMENT   100ll // Incremental wait time on input buff dq

#define INITIAL_BUFFER_SIZE   100000
#define BUFFER_SIZE_INCREMENT 100000
#define MAX_BUFFER_SIZE      7077888 // Sized to match MediaCodec input buffer

class AvbH264Sink;

/******************************************************************************
 *                        global static variables
 *****************************************************************************/
sem_t gThreadInitSem;
sp<AvbH264Sink> gSink;

/******************************************************************************
 *                            CodecOutputInfo
 *
 * This class is used to keep statistic on the output from the media codec.
 *****************************************************************************/
class CodecOutputInfo : public RefBase {
public:
    CodecOutputInfo()
        : mNumBuffersDecoded(0),
          mNumBytesDecoded(0),
          mStartTimeUs(0),
          mPrintEveryNBufs(-1) { } ;
    void FrameDecoded(size_t size);
    void PrintInfo();
    void PrintInfoEveryNBuffer(int val) { mPrintEveryNBufs = val; };

private:
    int64_t mNumBuffersDecoded;
    int64_t mNumBytesDecoded;
    int64_t mStartTimeUs;
    int mPrintEveryNBufs;
};

/**
 * This function should be called every time the codec produces a frame.
 */
void CodecOutputInfo::FrameDecoded(size_t frameSize) {
    if (mNumBuffersDecoded == 0) {
        // For more accurate fps count, start timer on first frame received
        mStartTimeUs = ALooper::GetNowUs();
    }

    mNumBuffersDecoded++;
    mNumBytesDecoded += frameSize;

    if (mPrintEveryNBufs != -1) {
        if ((mNumBuffersDecoded % mPrintEveryNBufs) == 0) {
            PrintInfo();
        }
    }
}

/**
 * This function is used to print out the current stats.
 */
void CodecOutputInfo::PrintInfo() {
    int64_t elapsedTimeUs = ALooper::GetNowUs() - mStartTimeUs;

    ALOGD("track 0: %lld frames decoded, %.2f fps. %lld"
            " bytes received. %.2f KB/sec\n",
           (long long)mNumBuffersDecoded,
           mNumBuffersDecoded * 1E6 / elapsedTimeUs,
           (long long)mNumBytesDecoded,
           mNumBytesDecoded * 1E6 / 1024 / elapsedTimeUs);
}

/******************************************************************************
 *                               VideoStats
 *
 * Simple container to keep track of video's width, height, and framerate.
 *****************************************************************************/
class VideoStats {
public:
    VideoStats(int w, int h, int fr)
        : width(w),
          height(h),
          frameRate(fr) { };
    int width;
    int height;
    int frameRate;
};

/******************************************************************************
 *                              PacketBuffer
 *
 * This class is used to buffer input packets in order to feed data to the
 * media codec in larger chunks.
 *****************************************************************************/
class PacketBuffer : public RefBase {
public:
    PacketBuffer()
        : mBuffer(nullptr),
          mBufferSize(0),
          mPos(0),
          mCount(0) {
        GrowBuffer(INITIAL_BUFFER_SIZE);
    };
    ~PacketBuffer();
    bool GrowBuffer(size_t size);
    bool CopyBuf(uint8_t *pBuf, int size);
    void Reset();
    size_t GetSize() { return mPos; };
    uint8_t* GetBuf() { return mBuffer; };

private:
    uint8_t* mBuffer;
    size_t mBufferSize;
    size_t mPos;
    int mCount;
};

PacketBuffer::~PacketBuffer() {
    if (mBuffer) {
        free(mBuffer);
        mBuffer = nullptr;
    }
}

/**
 * Call this function to grow the packet buffer by some amount.
 */
bool PacketBuffer::GrowBuffer(size_t size) {
    size_t newSize = mBufferSize + size;

    if (newSize >= MAX_BUFFER_SIZE) {
        ALOGE("Fatal error: packet buffer growing too large (%zu). "\
              "Ensure talker and listener use comparable data rates",
              newSize);
        // Drop all pending data and start over to try recovering
        Reset();
        return false;
    }

    mBuffer = (uint8_t *) realloc(mBuffer, newSize);

    if (!mBuffer) {
        ALOGE("Fatal error: failed to realloc packet buffer (%p), "\
              "currSize=%zu, newSize=%zu", mBuffer, mBufferSize, newSize);
        return false;
    }

    mBufferSize = newSize;
    return true;
}

/**
 * Call this function to append a packet's data to the buffer.
 */
bool PacketBuffer::CopyBuf(uint8_t *pBuf, int size) {
    // Check buffer is large enough to contain new data
    if ((mPos + size) > mBufferSize) {
        // Try to grow buffer to fit new data. Growth increment should be much
        // greater than new buffer size to prevent constant reallocation.
        if (!GrowBuffer(BUFFER_SIZE_INCREMENT)) {
            return false;
        }
    }

    // Copy data
    memcpy((mBuffer + mPos), pBuf, size);
    mPos += size;
    mCount++;
    return true;
}

/**
 * Resets the buffer.
 * This function should be called after data has been pushed into the media
 * codec input buffer.
 */
void PacketBuffer::Reset() {
    mCount = 0;
    mPos = 0;
}

/******************************************************************************
 *                              Display
 *
 * The Display class is used to connect to a display and acquire a surface
 * to render our decoded frames.
 *****************************************************************************/
class Display : public RefBase {
public:
    Display();
    virtual ~Display();
    sp<Surface> & GetSurface() { return mSurface; };
private:
    sp<SurfaceComposerClient> mComposerClient;
    sp<SurfaceControl> mControl;
    sp<Surface> mSurface;
};


/**
 * Initialize display
 */
Display::Display() {
    mComposerClient = new SurfaceComposerClient;
    CHECK_EQ(mComposerClient->initCheck(), (status_t)OK);

    sp<IBinder> display(SurfaceComposerClient::getBuiltInDisplay(
                         ISurfaceComposer::eDisplayIdMain));
    DisplayInfo info;
    SurfaceComposerClient::getDisplayInfo(display, &info);
    ssize_t displayWidth = info.w;
    ssize_t displayHeight = info.h;

    ALOGI("display is %zd x %zd", displayWidth, displayHeight);

    mControl = mComposerClient->createSurface(
                String8("AVB H264 sink"),
                displayWidth,
                displayHeight,
                PIXEL_FORMAT_RGB_565,
                0);

    CHECK(mControl != NULL);
    CHECK(mControl->isValid());

    SurfaceComposerClient::openGlobalTransaction();
    CHECK_EQ(mControl->setLayer(INT_MAX), (status_t)OK);
    CHECK_EQ(mControl->show(), (status_t)OK);
    SurfaceComposerClient::closeGlobalTransaction();

    mSurface = mControl->getSurface();
    CHECK(mSurface != NULL);
}

Display::~Display() {
    mComposerClient->dispose();
}


/******************************************************************************
 *                                AvbH264Sink
 *
 * This class is used to decode h264 data and render frames to a surface.
 *****************************************************************************/
class AvbH264Sink : public RefBase {
public:
    AvbH264Sink(VideoStats* vs)
        : mCodec(nullptr),
          mPacketBuffer(),
          mVideoStats(vs->width, vs->height, vs->frameRate),
          mInputTimeout(INPUT_DQ_TIMEOUT),
          mStopStream(false),
          mSawOutputEOS(false),
          mNextDqUsec(0),
          mFrameDurationUsec(0) {
       if (vs->frameRate > 0) {
           mFrameDurationUsec = (s2ns(1) / vs->frameRate) / 1000;
       }
    };
    virtual ~AvbH264Sink() { };
    int DataSink(uint8_t *pBuf, int size);
    int Run();
    int Stop();
    int InitMediaCodec(const sp<ALooper> &looper, const sp<Surface> &surface);

private:
    sp<MediaCodec> mCodec;
    PacketBuffer mPacketBuffer;
    VideoStats mVideoStats;
    int64_t mInputTimeout;

    bool mStopStream;
    bool mSawOutputEOS;
    int64_t mNextDqUsec;
    int64_t mFrameDurationUsec;
};

/**
 * Initializes the media codec and attaches to the given surface.
 */
int AvbH264Sink::InitMediaCodec(const sp<ALooper> &looper,
        const sp<Surface> &surface) {
    int err = 0;
    sp<AMessage> format = new AMessage;
    format->setString("mime",MEDIA_MIMETYPE_VIDEO_AVC);
    format->setInt32("height",mVideoStats.height);
    format->setInt32("width",mVideoStats.width );
    format->setInt32("arbitrary_bytes", 1);

    mCodec = MediaCodec::CreateByType(looper,
            MEDIA_MIMETYPE_VIDEO_AVC, false);

    CHECK(mCodec != NULL);

    err = mCodec->configure(format, surface, NULL, 0);
    CHECK_EQ(err, (status_t)OK);

    err =  mCodec->start();
    CHECK_EQ(err, (status_t)OK);

    return err;
}

/**
 * This is the run loop for the Avbh264SinkThread.
 * This function pull decoded frames from the media codec at a set interval
 * based on the video's frame rate.
 */
int AvbH264Sink::Run() {
    CodecOutputInfo outputInfo;
    size_t index;
    size_t offset;
    size_t size;
    int64_t presentationTimeUs;
    uint32_t flags;

    outputInfo.PrintInfoEveryNBuffer(60);

    while (!(mSawOutputEOS || mStopStream)) {
        int64_t frameStartUsec = ALooper::GetNowUs();

        status_t err = mCodec->dequeueOutputBuffer(
                &index, &offset, &size, &presentationTimeUs, &flags,
                OUTPUT_DQ_TIMEOUT);

        if (err == OK) {
            outputInfo.FrameDecoded(size);

            err = mCodec->renderOutputBufferAndRelease(index);
            CHECK_EQ(err, (status_t)OK);

            if (flags & MediaCodec::BUFFER_FLAG_EOS) {
                ALOGD("reached EOS on output.");
                mSawOutputEOS = true;
            }

            // space out the frames based on the frame rate
            int64_t displayTimeUsec = ALooper::GetNowUs() - frameStartUsec;
            if (displayTimeUsec < mFrameDurationUsec) {
                usleep(mFrameDurationUsec - displayTimeUsec);
            }
        } else if (err == INFO_OUTPUT_BUFFERS_CHANGED) {
            // ignore
        } else if (err == INFO_FORMAT_CHANGED) {
            sp<AMessage> format;
            CHECK_EQ((status_t)OK, mCodec->getOutputFormat(&format));
            ALOGI("INFO_FORMAT_CHANGED: %s", format->debugString().c_str());
        } else if (err == -EAGAIN) {
            // try pulling again, not output buffer was available within timeout
        } else {
            ALOGE("Got error %d\n", err);
        }
    }

    ALOGD("Reached EOS or stream stopped - stopping codec and exiting\n");
    outputInfo.PrintInfo();
    mCodec->stop();
    mCodec->release();

    return 0;
}

/**
 * Calling this function will halt video playback and exit the thread.
 */
int AvbH264Sink::Stop() {
    mStopStream = true;
    return 0;
}

/**
 * Data sink for h264 stream.
 * This function will be called every time the listener receives a packet.
 */
int AvbH264Sink::DataSink(uint8_t *pBuf, int size) {
    size_t bufferIndex = 0;
#ifdef USE_MEDIA_CODEC_BUFFER
    sp<MediaCodecBuffer> codecBuffer = nullptr;
#else
    sp<ABuffer> codecBuffer = nullptr;
#endif

    uint32_t bufferFlags = 0;
    int err = 0;
    int64_t currentTimeUsec = ALooper::GetNowUs();

    if (mStopStream) {
        return -1;
    }

    if (!mPacketBuffer.CopyBuf(pBuf, size)) {
        return -1;
    }

    // Check if we have buffered enough data
    if (mNextDqUsec == 0) {
        mNextDqUsec = currentTimeUsec + mFrameDurationUsec;
        return size;
    } else if (mNextDqUsec > currentTimeUsec) {
        return size;
    }

    // Try to dequeue an input buffer from the codec
    err = mCodec->dequeueInputBuffer(&bufferIndex, mInputTimeout);
    if (err != OK) {
        // Couldn't get an input buffer in time, buffer data for a bit longer
        mNextDqUsec = currentTimeUsec + mFrameDurationUsec;
        mInputTimeout += INPUT_DQ_TIMEOUT_INCREMENT;
        return size;
    }

    // Grab the input buffer
    err = mCodec->getInputBuffer(bufferIndex, &codecBuffer);
    if (err != OK) {
        return -1;
    }

    // Push buffered data into the input buffer
    //ALOGE("Pushing %zu bytes into codec input buffer %zu (maxsize = %zu)\n",
    //        mPacketBuffer.GetSize(), bufferIndex,  codecBuffer->capacity());
    memcpy(codecBuffer->data(), mPacketBuffer.GetBuf(), mPacketBuffer.GetSize());

    // Re-queue the filled input buffer
    err = mCodec->queueInputBuffer(
            bufferIndex,
            0,
            mPacketBuffer.GetSize(),
            0,
            bufferFlags);

    // reset packet buffer and timers
    mPacketBuffer.Reset();
    mInputTimeout = INPUT_DQ_TIMEOUT;
    mNextDqUsec = 0;

    CHECK_EQ(err, (status_t)OK);
    return size;
}

/**
 * This function is used to create and initialize everything necessary to
 * decode and render the video stream.
 */
void* Avbh264SinkThread(void *arg) {
    ProcessState::self()->startThreadPool();

    // Create sink and keep local ref to keep sink alive until thread is done
    sp<AvbH264Sink> sink = gSink = new AvbH264Sink((VideoStats*) arg);
    sp<Display> display = new Display();
    sp<ALooper> looper = new ALooper();

    looper->start();

    // Init
    sink->InitMediaCodec(looper, display->GetSurface());

    // Unblock Avbh264StreamInitialize call
    if (sem_post(&gThreadInitSem)!=0){
       ALOGE("failed to post the semaphore");
       return NULL;
    }

    // This function returns once EOS is reached or stream is halted
    sink->Run();

    looper->stop();
    return NULL;
}

/******************************************************************************
 *                              Exported Symbols
 *****************************************************************************/

#if defined( __cplusplus )
extern "C"
{
#endif /* end of macro __cplusplus */

int Avbh264StreamInitialize(int width, int height, int frameRate) {
    pthread_t tid;
    int rc;

    if (sem_init(&gThreadInitSem,0,0) < 0){
        ALOGE("semaphore initialization failed\n");
        return -1;
    }

    VideoStats vs(width, height, frameRate);

    rc = pthread_create(&tid, NULL, Avbh264SinkThread, &vs);
    if (rc!=0){
        ALOGE(" Avbh264sinkthread creation failed...\n ");
        return -1;
    }

    if (sem_wait(&gThreadInitSem)!= 0){
        ALOGE("waiting on semaphore failed\n");
        return -1;
    }

    return 0;
}

int Avbh264DataSink(char *pBuf, int size) {
    if (gSink.get()) {
        return gSink->DataSink((uint8_t *)pBuf, size);
    }
    return -1;
}

int Avbh264StreamClose() {
    if (gSink.get()) {
        gSink->Stop();
    }
    gSink = nullptr;
    return 0;
}

#ifdef __cplusplus
}
#endif /* __cplusplus*/