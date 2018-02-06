/*
 * Copyright (c) 2016-2017, Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef _DECODER_H_
#define _DECODER_H_

#include "VASimFrameBuffer.h"
#include <VAM/vaapi.h>

#include <string>

class DecoderClock
{
public:
    DecoderClock();
    double get();
    void set(double pts, int serial);
    void set_speed(double targetSpeed);
    void init(int *queue_serial);
    void sync_to_slave(DecoderClock *slave);

    double pts;           /* clock base */
    double pts_drift;     /* clock base minus time at which we updated the clock */
    double last_updated;
    double speed;
    int serial;           /* clock is based on a packet with this serial */
    int paused;
    int *queue_serial;    /* pointer to the current packet queue serial, used for obsolete clock detection */
};

struct VideoState;
class VASimDecoder
{
public:
    VASimDecoder();
    int init();
    int destroy();

    int open(VideoState *vs, std::string filename, int videoRowIDX, int videoColumnIDX);
    void start(AVCodecContext *_avctx, PacketQueue *_queue,
        int (*fn)(void *), void *arg);
    int decode_frame(AVFrame *frame);
    void close(VideoState *vs);

    AVPacket pkt;
    AVPacket pkt_temp;
    PacketQueue *queue;
    AVCodecContext *avctx;
    int pkt_serial;
    int finished;
    int packet_pending;
    int64_t start_pts;
    AVRational start_pts_tb;
    int64_t next_pts;
    AVRational next_pts_tb;
};

/* Minimum SDL audio buffer size, in samples. */
#define SDL_AUDIO_MIN_BUFFER_SIZE 512

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
/* TODO: We assume that a decoded and resampled frame fits into this buffer */
#define SAMPLE_ARRAY_SIZE (8 * 65536)

typedef struct AudioParams
{
    int freq;
    int channels;
    int64_t channel_layout;
    enum AVSampleFormat fmt;
    int frame_size;
    int bytes_per_sec;
} AudioParams;

struct VideoState
{
    VideoState();

    int videoColumnIDX;
    int videoRowIDX;

    //AVInputFormat *iformat;
    bool isStop;
    bool force_refresh;
    bool paused;
    bool pauseStateNow;
    bool queue_attachments_req;
    int read_pause_return;
    AVFormatContext *ic;

    DecoderClock vidclk;
    FrameBuffer pictq;
    VASimDecoder viddec;

    int viddec_width;
    int viddec_height;

    double frame_timer;
    double frame_last_returned_time;
    double frame_last_filter_delay;
    int videoStreamIDX;
    AVStream *video_st;
    PacketQueue packetQ;
    double max_frame_duration;      // maximum duration of a frame - above this, we consider the jump a timestamp discontinuity
    struct SwsContext *img_convert_ctx;
    bool eof;
    std::string filename;
    //int width, height, xleft, ytop;
    int step;

    int frameDrop;

    bool enableDisplay;
    bool putVideoInited; // hack: used for init VAAPI when first frame callback from renderer
};

int getSnapshot( vaapi_snapshot_info *info);
AVFrame* decoder_open_image(const char* imageFileName);
int decoder_close_image(AVFrame *frame);

#endif // #define _DECODER_H_
