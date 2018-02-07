/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Not a Contribution.
 *
 * Copyright (C) 2007 The Android Open Source Project
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

#define LOG_NDEBUG 0
#define LOG_TAG "AvbMjpegSink"

#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <cutils/properties.h>
#include <utils/misc.h>
#include <sys/types.h>

#include <binder/IPCThreadState.h>
#include <utils/Atomic.h>
#include <utils/Errors.h>
#include <utils/Log.h>

#include <ui/PixelFormat.h>
#include <ui/Rect.h>
#include <ui/Region.h>
#include <ui/DisplayInfo.h>

#include <gui/ISurfaceComposer.h>
#include <gui/Surface.h>
#include <gui/SurfaceComposerClient.h>

#include <GLES/gl.h>
#include <GLES/glext.h>
#include <EGL/eglext.h>

#include "AvbMjpegSink.h"
#include "GlSink.h"
#include "MjpegFrame.h"

#define BILLION 1000000000L
#define FPS_LIMIT 24

// ---------------------------------------------------------------------------
//                         AvbMjpegSink impl
// ---------------------------------------------------------------------------
using namespace android;

GlSink::GlSink(AvbMjpegSink* parent)
    : Thread(false),
      mSession(new SurfaceComposerClient()),
      mParent(parent),
      mMutex(PTHREAD_MUTEX_INITIALIZER),
      mCv(PTHREAD_COND_INITIALIZER) {
    pthread_mutex_init(&mMutex, nullptr);
    pthread_cond_init(&mCv, nullptr);
}

GlSink::~GlSink() {
}

void GlSink::onFirstRef() {
    status_t err = mSession->linkToComposerDeath(this);
    ALOGE_IF(err, "linkToComposerDeath failed (%s) ", strerror(-err));
    if (err == NO_ERROR) {
        run("AvbMjpeg-GlSink", PRIORITY_DISPLAY);
    }
}

sp<SurfaceComposerClient> GlSink::session() const {
    return mSession;
}

/**
 *
 */
void GlSink::binderDied(const wp<IBinder>&)
{
    ALOGD("SurfaceFlinger died, exiting...");

    // calling requestExit() is not enough here because the Surface code
    // might be blocked on a condition variable that will never be updated.
    kill( getpid(), SIGKILL );
    requestExit();
}

/**
 *
 */
void GlSink::FrameReady() {
    if( 0 != pthread_mutex_lock(&mMutex)) {
        return;
    }

    pthread_cond_broadcast(&mCv);

    pthread_mutex_unlock(&mMutex);
}

/**
 *
 */
status_t GlSink::readyToRun() {
    sp<IBinder> dtoken(SurfaceComposerClient::getBuiltInDisplay(
            ISurfaceComposer::eDisplayIdMain));
    DisplayInfo dinfo;
    status_t status = SurfaceComposerClient::getDisplayInfo(dtoken, &dinfo);
    if (status) {
        printf("AvbMjpeg-GlSink::readyToRun - failed to get SCC\n");
        return -1;
    }

    char value[PROPERTY_VALUE_MAX];
    property_get("persist.panel.orientation", value, "0");
    int orient = atoi(value)/90;
    if (orient == eOrientation90 || orient == eOrientation270) {
        int temp = dinfo.h;
        dinfo.h = dinfo.w;
        dinfo.w = temp;
    }
    Rect destRect(dinfo.w, dinfo.h);
    mSession->setDisplayProjection(dtoken, orient, destRect, destRect);
    // create the native surface
    sp<SurfaceControl> control = session()->createSurface(String8("AvbMjpegSink"),
            dinfo.w, dinfo.h, PIXEL_FORMAT_RGB_565);

    SurfaceComposerClient::openGlobalTransaction();
    control->setLayer(0x40000000);
    SurfaceComposerClient::closeGlobalTransaction();

    sp<Surface> s = control->getSurface();

    // initialize opengl and egl
    const EGLint attribs[] = {
            EGL_RED_SIZE,   8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE,  8,
            EGL_DEPTH_SIZE, 0,
            EGL_NONE
    };
    EGLint w, h;
    EGLint numConfigs;
    EGLConfig config;
    EGLSurface surface;
    EGLContext context;

    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

    eglInitialize(display, 0, 0);
    eglChooseConfig(display, attribs, &config, 1, &numConfigs);
    surface = eglCreateWindowSurface(display, config, s.get(), NULL);
    context = eglCreateContext(display, config, NULL, NULL);
    eglQuerySurface(display, surface, EGL_WIDTH, &w);
    eglQuerySurface(display, surface, EGL_HEIGHT, &h);

    if (eglMakeCurrent(display, surface, surface, context) == EGL_FALSE) {
        printf("GlSink::readyToRun - failed to eglMakeCurrent\n");
        return NO_INIT;
    }

    mDisplay = display;
    mContext = context;
    mSurface = surface;
    mWidth = w;
    mHeight = h;
    mFlingerSurfaceControl = control;
    mFlingerSurface = s;

    return NO_ERROR;
}

/**
 *
 */
bool GlSink::threadLoop() {
    glGenTextures(1, &mTexture.name);
    int framesNeeded = 1;
    nsecs_t frameDuration = s2ns(1) / FPS_LIMIT; // todo get fps from ini file
    nsecs_t displayTime = 0;

    // This will run until video is terminated
    do {
        sp<AvbMjpegSink> parent = mParent.promote();

        pthread_mutex_lock(&mMutex);
        while ((parent.get() != nullptr) &&
                (parent->GetNumReadyFrames() < framesNeeded)) {
            if (framesNeeded > 1) {
                printf("AvbMjpeg-GlSink:: waiting on %d frames\n", framesNeeded);
            }
            parent = nullptr; // release strong ref while waiting
            pthread_cond_wait(&mCv, &mMutex);
            parent = mParent.promote(); // re-aquire strong ref
        }
        pthread_mutex_unlock(&mMutex);

        // If we're shutting down, break out before trying to do any work
        if (exitPending() || (parent == nullptr)) {
            parent = nullptr;
            break;
        }

        MjpegFrame* frame = parent->GetNextReadyFrame();

        // Skip this frame if it failed to decode properly
        if (frame->GetState() == MjpegFrame::DECODE_ERROR) {
            printf("AvbMjpeg-GlSink:: skipping frame %d due to decode error\n",
                    frame->GetFrameNum());
            framesNeeded = 1;
            frame->Reset();
            parent->FramePosted();
            continue;
        }

        // Ensure frames are coming in the right order
        if (frame->GetState() != MjpegFrame::READY_TO_POST) {
            printf( "AvbMjpeg-GlSink:: notice - frame %d not yet ready. State = %d\n",
                    frame->GetFrameNum(), frame->GetState());
            framesNeeded++;
            continue;
        } else {
            // Once all needed frames have arrived, this can be safely reset to 1
            framesNeeded = 1;
        }

        // Post the frame
        displayTime = PostToDisplay(frame);

        parent->FramePosted();
        parent = nullptr;
        if (displayTime < frameDuration) {
            usleep((frameDuration - displayTime)/1000);
        }
     } while (!exitPending());

    // Cleanup
    eglMakeCurrent(mDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(mDisplay, mContext);
    eglDestroySurface(mDisplay, mSurface);
    glDeleteTextures(1, &mTexture.name);
    mFlingerSurface.clear();
    mFlingerSurfaceControl.clear();
    eglTerminate(mDisplay);
    return false;
}


/**
 *
 */
nsecs_t GlSink::PostToDisplay(MjpegFrame* frame) {
    double displayTime;
    struct timespec start_ts, end_ts;
    clock_gettime(CLOCK_MONOTONIC, &start_ts);

    const SkBitmap& bitmap = frame->GetDecodedBitmap();

    // prepare for blending
    bitmap.lockPixels();
    const int w = mTexture.w = bitmap.width();
    const int h = mTexture.h = bitmap.height();
    void* p = bitmap.getPixels();
    GLint crop[4] = { 0, h, w, -h };

    glBindTexture(GL_TEXTURE_2D, mTexture.name);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, p);
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_CROP_RECT_OES, crop);
    glTexParameterx(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glEnable(GL_TEXTURE_2D);

    // Blend
    const GLint xc = (mWidth  - mTexture.w) / 2;
    const GLint yc = (mHeight - mTexture.h) / 2;
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glTexEnvx(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture(GL_TEXTURE_2D, mTexture.name);
    glDrawTexiOES(xc, yc, 0, mTexture.w, mTexture.h);
    EGLBoolean res = eglSwapBuffers(mDisplay, mSurface);
    if (res == EGL_FALSE) {
        printf( "AvbMjpeg-GlSink: eglSwapBuffers failed..!\n");
        return 0;
    }

    // Done with this frame, mark as free
    frame->Reset();


    // record end of decoding
    clock_gettime(CLOCK_MONOTONIC, &end_ts);
    displayTime = BILLION * (end_ts.tv_sec - start_ts.tv_sec)
            + (end_ts.tv_nsec - start_ts.tv_nsec);
#ifdef ENABLE_TIME_LOGGING
    printf("AvbMjpeg-GlSink: Frame:%d, disp time = %.3lf msec\n",
            frame->GetFrameNum(),
            (displayTime/1000000));
#endif

    return displayTime;
}

