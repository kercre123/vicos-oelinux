/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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


#define LOG_TAG "OverlayTestApp"

#include <stdlib.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/msm_ion.h>
#include <utils/Log.h>
#include "qmmf_overlay_test.h"

using namespace android;

OverlayTest::OverlayTest()
{
    ALOGD("%s: Enter ",__func__);
    mIonDevice = -1;
    mSize = 0;
    mIonFd = 0;
    mVaddr = NULL;
    memset(&mHandleData, 0x0, sizeof mHandleData);
    mInputWidth = -1;
    mInputHeight = -1;
    mStaticImage = false;
    mStaticImagePath.setTo("");
    mImageLocation = OverlayLocationType::kTopLeft;
    mStaticImageId = -1;
    mDateAndTime = false;
    mDateLocation = OverlayLocationType::kTopRight;
    mDateFormat = OverlayDateFormatType::kYYYYMMDD;
    mTimeFormat = OverlayTimeFormatType::kHHMM_AMPM;
    mDateAndTimeId = -1;
    mUserText = false;
    mTextLocation = OverlayLocationType::kBottomRight;
    mText.setTo("");
    ALOGD("%s: Exit ",__func__);
}

OverlayTest::~OverlayTest()
{
    ALOGD("%s: Enter ",__func__);
    if(mIonFd) {
        ioctl(mIonDevice, ION_IOC_FREE, &mHandleData);
        close(mIonFd);
        mIonFd = -1;
    }

    if(mIonDevice)
        close(mIonDevice);

    ALOGD("%s: Exit ",__func__);
}

int32_t OverlayTest::parseConfig(char *fileName) {

    ALOGD("%s: Enter ",__func__);
    ALOGD("%s: fileName=%s ",__func__, fileName);
    FILE *fp;
    const int MAX_LINE = 128;
    char line[MAX_LINE];
    char value[50];
    char key[25];

    if(!(fp = fopen(fileName,"r"))) {
        ALOGE("failed to open config file: %s", fileName);
        return -1;
    }

    while(fgets(line,MAX_LINE-1,fp)) {
        if((line[0] == '\n') || (line[0] == '/') || line[0] == ' ')
            continue;
        memset(value, 0x0, sizeof(value));
        memset(key, 0x0, sizeof(key));
        int len = strlen(line);
        int i,j = 0;

        int pos = strcspn(line,":");
        for(i = 0; i< pos; i++){
            if(line[i] != ' ') {
                key[j] = line[i];
                j++;
            }
        }

        key[j] = '\0';
        j = 0;
        for(i = pos+1; i< len; i++) {
            if(line[i] != ' ') {
                value[j] = line[i];
                 j++;
            }
        }
        value[j] = '\0';

        if(!strncmp("InputYUVFileName", key, sizeof "InputYUVFileName")) {
            mYUVInputFile.setTo(value);
            ALOGD("%s: mYUVInputFile=%s", __func__, mYUVInputFile.string());
        } else if(!strncmp("InputWidth", key, sizeof "InputWidth")) {
            mInputWidth = atoi(value);
            ALOGD("%s: mInputWidth=%d", __func__, mInputWidth);
        } else if(!strncmp("InputHeight", key, sizeof "InputHeight")) {
            mInputHeight = atoi(value);
            ALOGD("%s: mInputHeight=%d", __func__, mInputHeight);
        } else if(!strncmp("StaticImage", key, sizeof "StaticImage")) {
            mStaticImage = atoi(value) ? true:false;
        } else if(!strncmp("StaticImageFilePath", key, sizeof "StaticImageFilePath")) {
                mStaticImagePath.setTo(value);
                ALOGD("%s: StaticImagePath=%s", __func__, mStaticImagePath.string());
        } else if(!strncmp("StaticImageLocation", key, sizeof "StaticImageLocation")) {
            mImageLocation = static_cast<OverlayLocationType>(atoi(value));
            ALOGD("%s: mImageLocation=%d", __func__, mImageLocation);
        } else if(!strncmp("DateAndTime", key, sizeof "DateAndTime")) {
            mDateAndTime = atoi(value) ? true:false;
            ALOGD("%s: mDateAndTime=%d", __func__, mDateAndTime);
        } else if(!strncmp("DateAndTimeLocation", key, sizeof "DateAndTimeLocation")) {
            mDateLocation = static_cast<OverlayLocationType>(atoi(value));
            ALOGD("%s: mDateLocation=%d", __func__, mDateLocation);
        } else if(!strncmp("DateFormat", key, sizeof "DateFormat")) {
            mDateFormat = static_cast<OverlayDateFormatType>(atoi(value));
            ALOGD("%s: mDateFormat=%d", __func__, mDateFormat);
        } else if(!strncmp("TimeFormat", key, sizeof "TimeFormat")) {
            mTimeFormat = static_cast<OverlayTimeFormatType>(atoi(value));
            ALOGD("%s: mTimeFormat=%d", __func__, mTimeFormat);
        } else if(!strncmp("UserText", key, sizeof "UserText")) {
            mUserText = atoi(value) ? true:false;
            ALOGD("%s: mUserText=%d", __func__, mUserText);
        } else if(!strncmp("Text", key, sizeof "Text")) {
            mText.setTo(value);
            ALOGD("%s: mText=%s", __func__, mText.string());
        } else if(!strncmp("UserTextLocation", key, sizeof "UserTextLocation")) {
            mTextLocation = static_cast<OverlayLocationType>(atoi(value));
            ALOGD("%s: mTextLocation=%d", __func__, mTextLocation);
        } else {
            ALOGE("Unknown Key %s found in %s", key, fileName);
        }
    }
    fclose(fp);
    return 0;
}

int32_t OverlayTest::applyOverlay()
{
    ALOGD("%s: Enter",__func__);
    int32_t ret = 0;

    mIonDevice = open("/dev/ion", O_RDONLY);
    if (mIonDevice < 0) {
        ALOGE("%s: Ion dev open failed %s\n", __func__,strerror(errno));
        return -1;
    }
    ret = allocateMemory();
    if(ret != 0) {
        ALOGE("%s:Ion allocation failed!", __func__);
        return ret;
    }
    uint32_t *pixels = NULL;
    pixels = (uint32_t*)mVaddr;
    //Load YUV input file.
    FILE *inputFile = NULL ,*outputFile = NULL;
    size_t bytes;
    inputFile = fopen(mYUVInputFile.string(), "rb");
    if(inputFile) {
        bytes = fread(pixels, 1, mSize, inputFile);
        ALOGD("%s: Total btyes = %d",__func__,bytes);
        fclose(inputFile);
    } else {
        ALOGE("%s: (%s)File open Failed!!",__func__, mYUVInputFile.string());
        goto ERROR;
    }

    ret = mOverlayHandle.Init(TargetBufferFormat::kYUVNV12);
    if(ret != 0) {
        ALOGE("%s: Overlay:Init failed!", __func__);
        return ret;
    }

    if(mDateAndTime) {
        OverlayParam param;
        memset(&param, 0x0, sizeof param);
        param.type     = OverlayType::kDateType;
        param.location = mDateLocation;
        param.color    = 0xFFFF0000; //SK_ColorRED
        param.date_time.date_format   = mDateFormat;
        param.date_time.time_format   = mTimeFormat;

        ret = mOverlayHandle.CreateOverlayItem(param, &mDateAndTimeId);
        if(ret != 0) {
            ALOGE("%s: createOverlayItem failed!", __func__);
        }
        mOverlayHandle.EnableOverlayItem(mDateAndTimeId);
    }

    OverlayTargetBuffer buf;
    memset(&buf, 0x0, sizeof buf);

    buf.ion_fd    = mIonFd;
    buf.frame_len = mSize;
    buf.width     = mInputWidth;
    buf.height    = mInputHeight;
    buf.format    = TargetBufferFormat::kYUVNV12; //TODO: take from config file.

    ret = mOverlayHandle.ApplyOverlay(buf);
    if(ret != 0) {
        ALOGE("%s: applyOverlay failed!", __func__);
    }

    //Dump output.
    //TODO: take out file from config file.
    outputFile = fopen("/data/misc/qmmf/output.yuv", "wb");
    if(outputFile) {
        bytes = fwrite(pixels, 1, mSize, outputFile);
        fclose(outputFile);
    } else {
        ALOGE("%s: Output file Open Failed!",__func__);
    }

    //TODO: Add other overlay Items.

    ALOGD("%s: Exit",__func__);
    return 0;
ERROR:
    ioctl(mIonDevice, ION_IOC_FREE, &mHandleData);
    close(mIonFd);
    mIonFd = -1;
    return -1;
}

int32_t OverlayTest::allocateMemory()
{
    ALOGD("%s:Enter",__func__);

    struct ion_allocation_data alloc;
    struct ion_fd_data ionFdData;
    void *data = NULL;
    int ionType = 0x1 << ION_IOMMU_HEAP_ID;
    int32_t ret = 0;

    uint32_t size = mInputWidth * mInputHeight * 1.5;

    memset(&alloc, 0, sizeof(ion_allocation_data));
    alloc.len = size;
    alloc.len = (alloc.len + 4095) & (~4095);
    alloc.align = 4096;
    alloc.flags = ION_FLAG_CACHED;
    alloc.heap_id_mask = ionType;
    ret = ioctl(mIonDevice, ION_IOC_ALLOC, &alloc);
    if (ret < 0) {
        ALOGE("%s:ION allocation failed\n",__func__);
        goto ION_ALLOC_FAILED;
    }

    memset(&ionFdData, 0, sizeof(ion_fd_data));
    ionFdData.handle = alloc.handle;
    ret = ioctl(mIonDevice, ION_IOC_SHARE, &ionFdData);
    if (ret < 0) {
        ALOGE("%s:ION map failed %s\n",__func__,strerror(errno));
        goto ION_MAP_FAILED;
    }

    data = mmap(NULL, alloc.len,
                PROT_READ  | PROT_WRITE,
                MAP_SHARED,ionFdData.fd, 0);

    if (data == MAP_FAILED) {
        ALOGE("%s:ION mmap failed: %s (%d)\n",__func__, strerror(errno),
            errno);
        goto ION_MAP_FAILED;
    }

    memset(&mHandleData, 0, sizeof(mHandleData));
    mHandleData.handle  = ionFdData.handle;
    mIonFd              = ionFdData.fd;
    mSize               = alloc.len;
    mVaddr              = data;

    ALOGD("%s:Exit ",__func__);
    return ret;

ION_MAP_FAILED:
    memset(&mHandleData, 0, sizeof(mHandleData));
    mHandleData.handle = ionFdData.handle;
    ioctl(mIonDevice, ION_IOC_FREE, &mHandleData);
ION_ALLOC_FAILED:
    close(mIonDevice);
    return -1;
}

int main(int argc,char *argv[])
{
    ALOGD("%s: Enter argc=%d",__func__, argc);

    OverlayTest overlayTest;

    if(argc < 2) {
        printf("Usage: %s overlay_config.txt", argv[0]);
        return -1;
    }
    auto ret = overlayTest.parseConfig(argv[1]);
    if(ret != 0) {
        ALOGE("config Error!");
        return -1;
    }
    ret  = overlayTest.applyOverlay();
    if(ret != 0) {
        ALOGE("applyOverlay failed!");
        return -1;
    }
    return 0;
}