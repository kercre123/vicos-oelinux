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


#ifndef OVERLAY_TEST_H
#define OVERLAY_TEST_H

#include <utils/String8.h>
#include "qmmf-sdk/qmmf_overlay.h"


using namespace qmmf;
using namespace overlay;
using namespace android;

class OverlayTest
{

public:
    OverlayTest();

    ~OverlayTest();

    int32_t parseConfig(char* fileName);

    int32_t applyOverlay();

private:

    int32_t allocateMemory();

    int32_t   mIonDevice;
    uint32_t  mSize;
    int32_t   mIonFd;
    void*     mVaddr;
    struct ion_handle_data mHandleData;


    String8 mYUVInputFile;
    int32_t mInputWidth;
    int32_t mInputHeight;

    bool mStaticImage;
    String8 mStaticImagePath;
    OverlayLocationType mImageLocation;
    uint8_t mStaticImageId;

    bool mDateAndTime;
    OverlayLocationType mDateLocation;
    OverlayDateFormatType mDateFormat;
    OverlayTimeFormatType mTimeFormat;
    uint32_t mDateAndTimeId;

    bool mUserText;
    OverlayLocationType mTextLocation;
    String8 mText;

    Overlay mOverlayHandle;
};

#endif
