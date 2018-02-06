/*
 * Copyright (c) 2016-2017, Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef _RENDERER_H_
#define _RENDERER_H_

#include "VASimDecoder.h"
#include "VASimFrameBuffer.h"
#include "VASimConfig.h"

class VASimRenderer
{
public:
    VASimRenderer();
    int init(const class VASimConfig &config);
    //int init( VideoState *is);
    int destroy();
    int get_master_sync_type(VideoState *is);
    void videoFresh(void *opaque, double *remaining_time);
    int putOverlay(struct vaapi_metadata_frame *overlayFrame);

    int video_open(VideoState *is, int force_set_video_mode);
    int resize(int targetW, int targetH);

    inline void toggleFullScreen() { enableFullScreen = !enableFullScreen; }

private:
    void display(VideoState *vs);
    void drawRect(int x, int y, int w, int h, int color, int update);
    void drawRectRGB(int x, int y, int w, int h, int r, int g, int b, int update);

    VASimConfig Config;

    struct vaapi_metadata_frame overlayFrame;

    int FullScreenWidth;
    int FullScreenHeight;
    bool enableFullScreen;

    int displayWidth;
    int displayHeight;
    int displayX;
    int displayY;

    const int DEFAULT_WIDTH;
    const int DEFAULT_HEIGHT;
};

#endif // #define _RENDERER_H_
