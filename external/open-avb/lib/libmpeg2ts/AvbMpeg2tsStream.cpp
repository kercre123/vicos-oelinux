/* Copyrights (c) 2016, The Linux Foundation. All rights reserved.

   "Not a Contribution."
 */

/*
 * Copyright (C) 2010 The Android Open Source Project
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

//#define LOG_NDEBUG 0
#define LOG_TAG "AvbMpeg2tsStream"
#include "utils/Log.h"

#include <binder/ProcessState.h>
#include <cutils/properties.h> // for property_get

#include <media/IMediaHTTPService.h>
#include <media/IStreamSource.h>
#include <media/mediaplayer.h>
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/DataSource.h>
#include <media/stagefright/MPEG2TSWriter.h>
#include <media/stagefright/MediaExtractor.h>
#include <media/stagefright/MediaSource.h>
#include <media/stagefright/MetaData.h>

#include <binder/IServiceManager.h>
#include <media/IMediaPlayerService.h>
#include <gui/ISurfaceComposer.h>
#include <gui/SurfaceComposerClient.h>
#include <gui/Surface.h>

#include <fcntl.h>
#include <ui/DisplayInfo.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <pthread.h>
#include <ctype.h>
#include <semaphore.h>
#include <sys/mman.h>
#include "AvbMpeg2tsStream.h"

sem_t theadInitSem ;
using namespace android;

struct MyStreamSource : public BnStreamSource {
    // Object assumes ownership of fd.
    MyStreamSource(int fd);
    virtual void setListener(const sp<IStreamListener> &listener);
    virtual void setBuffers(const Vector<sp<IMemory> > &buffers);

    virtual void onBufferAvailable(size_t index);
protected:
    virtual ~MyStreamSource();

private:
    int mFd;
    off64_t mFileSize;
    uint64_t mNumPacketsSent;
    sp<IStreamListener> mListener;
    Vector<sp<IMemory> > mBuffers;
    DISALLOW_EVIL_CONSTRUCTORS(MyStreamSource);
};

MyStreamSource::MyStreamSource(int fd)
    : mFd(fd),
      mFileSize(0),
      mNumPacketsSent(0) {
    CHECK_GE(fd, 0);

    mFileSize = lseek64(fd, 0, SEEK_END);
    lseek64(fd, 0, SEEK_SET);
}

MyStreamSource::~MyStreamSource() {
    close(mFd);
    mFd = -1;
}
void MyStreamSource::setListener(const sp<IStreamListener> &listener) {
    mListener = listener;
}

void MyStreamSource::setBuffers(const Vector<sp<IMemory> > &buffers) {
    mBuffers = buffers;
}

void MyStreamSource::onBufferAvailable(size_t index) {
    CHECK_LT(index, mBuffers.size());
    sp<IMemory> mem = mBuffers.itemAt(index);
    ssize_t n = read(mFd, mem->pointer(), mem->size());
    if (n <= 0) {
        mListener->issueCommand(IStreamListener::EOS, false /* synchronous */);
    } else {
        mListener->queueBuffer(index, n);

        mNumPacketsSent += n / 188;
    }
}

struct MyClient : public BnMediaPlayerClient {
    MyClient()
        : mEOS(false) {
    }

    virtual void notify(int msg, int ext1 __unused, int ext2 __unused, const Parcel *obj __unused) {
        Mutex::Autolock autoLock(mLock);

        if (msg == MEDIA_ERROR || msg == MEDIA_PLAYBACK_COMPLETE) {
            mEOS = true;
            mCondition.signal();
        }
    }

    void waitForEOS() {
        Mutex::Autolock autoLock(mLock);
        while (!mEOS) {
            mCondition.wait(mLock);
        }
    }

protected:
    virtual ~MyClient() {
    }

private:
    Mutex mLock;
    Condition mCondition;

    bool mEOS;

    DISALLOW_EVIL_CONSTRUCTORS(MyClient);
};


void* AvbMpegStreamthread(void *arg)
{
    if (arg == NULL) {};
    android::ProcessState::self()->startThreadPool();

    struct sockaddr_un addr;

    int sockfd = 0, connfd = 0;
    if ((sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
       ALOGE("socket error");
       return NULL;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path,MPEG2TS_SOCKET_PATH, sizeof(addr.sun_path)-2);
    unlink(MPEG2TS_SOCKET_PATH);

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
       ALOGE("bind error");
       return NULL;
    }

    if (listen(sockfd, 5) == -1) {
       ALOGE("listen error");
       return NULL;
    }

    if (sem_post(&theadInitSem)!=0){
       ALOGE("failed to post the semaphore");
       return NULL;
    }

    connfd = accept(sockfd, (struct sockaddr*)NULL, NULL);
    if (connfd < 0){
       ALOGE("failed to accept the connection");
       return NULL;
    }

    sp<SurfaceComposerClient> composerClient = new SurfaceComposerClient;
    CHECK_EQ(composerClient->initCheck(), (status_t)OK);

    sp<IBinder> display(SurfaceComposerClient::getBuiltInDisplay(
            ISurfaceComposer::eDisplayIdMain));
    DisplayInfo info;
    SurfaceComposerClient::getDisplayInfo(display, &info);
    ssize_t displayWidth = info.w;
    ssize_t displayHeight = info.h;

    ALOGV("display is %zd x %zd\n", displayWidth, displayHeight);

    sp<SurfaceControl> control =
        composerClient->createSurface(
                String8("A Surface"),
                displayWidth,
                displayHeight,
                PIXEL_FORMAT_RGB_565,
                0);

    CHECK(control != NULL);
    CHECK(control->isValid());

    SurfaceComposerClient::openGlobalTransaction();
    CHECK_EQ(control->setLayer(INT_MAX), (status_t)OK);
    CHECK_EQ(control->show(), (status_t)OK);
    SurfaceComposerClient::closeGlobalTransaction();

    sp<Surface> surface = control->getSurface();
    CHECK(surface != NULL);

    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder = sm->getService(String16("media.player"));
    sp<IMediaPlayerService> service = interface_cast<IMediaPlayerService>(binder);

    CHECK(service.get() != NULL);

    sp<MyClient> client = new MyClient;

    sp<IStreamSource> source;

    source = new MyStreamSource(connfd);
    sp<IMediaPlayer> player =
        service->create(client, AUDIO_SESSION_ALLOCATE);

    if (player != NULL && player->setDataSource(source) == NO_ERROR) {
        ALOGV("initialising the player with the connfd.");
        player->setVideoSurfaceTexture(surface->getIGraphicBufferProducer());
        player->start();

        client->waitForEOS();

        player->stop();
    } else {
        ALOGE("failed to instantiate player.\n");
    }

    composerClient->dispose();

    pthread_exit (NULL);
}
extern "C" {
    int AvbMpegStreamInitialize(void)
    {
        pthread_t tid;
        int rc;

        if (sem_init(&theadInitSem,0,0) < 0){
            ALOGE("semaphore initialization failed");
            return -1;
        }

        rc = pthread_create(&tid, NULL, AvbMpegStreamthread, NULL);
        if (rc!=0){
            ALOGE(" AvbMpegStreamthread creation failed...\n ");
            return -1;
        }

        if (sem_wait(&theadInitSem)!= 0){
            ALOGE("waiting on semaphore failed");
            return -1;
        }

        return 0;
    }
// This function will be called to connect with  the streaming application
    int AvbMpegConnectToStreamSource(void)
    {
        struct sockaddr_un addr;
        int sockfd;
        if ((sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
                ALOGE("socket error");
                return -1;
        }

        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, MPEG2TS_SOCKET_PATH, sizeof(addr.sun_path)-1);
        if (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
                ALOGE("connect error");
                return -1;
        }

       return sockfd;
   }

}
