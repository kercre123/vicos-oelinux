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

#ifndef GL_SINK_H
#define GL_SINK_H

#include <stdint.h>
#include <binder/IBinder.h>
#include <sys/types.h>
#include <utils/Thread.h>

#include <EGL/egl.h>
#include <GLES/gl.h>
#include <SkBitmap.h>

class SkBitmap;

namespace android {

class AudioPlayer;
class Surface;
class SurfaceComposerClient;
class SurfaceControl;
class MjpegFrame;
class AvbMjpegSink;

/**
 *
 */
class GlSink : public Thread, public IBinder::DeathRecipient {
public:
    GlSink(AvbMjpegSink* parent);
    virtual ~GlSink();

    void FrameReady();

    sp<SurfaceComposerClient> session() const;

private:
    virtual bool        threadLoop();
    virtual status_t    readyToRun();
    virtual void        onFirstRef();
    virtual void        binderDied(const wp<IBinder>& who);

    enum {
        eOrientationDefault     = 0,
        eOrientation90          = 1,
        eOrientation180         = 2,
        eOrientation270         = 3,
    };
    struct Texture {
        GLint   w;
        GLint   h;
        GLuint  name;
    };

    nsecs_t PostToDisplay(MjpegFrame* data);

    sp<SurfaceComposerClient>       mSession;
    wp<AvbMjpegSink> mParent;
    pthread_mutex_t mMutex;
    pthread_cond_t mCv;
    Texture     mTexture;
    int         mWidth;
    int         mHeight;
    EGLDisplay  mDisplay;
    EGLDisplay  mContext;
    EGLDisplay  mSurface;
    sp<SurfaceControl> mFlingerSurfaceControl;
    sp<Surface> mFlingerSurface;
};

}; // namespace android

#endif // GL_SINK_H
