/*
 * Copyright (c) 2016-2017, Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef _FRAMEQUEUE_H_
#define _FRAMEQUEUE_H_

#define __STDC_CONSTANT_MACROS
//#define UINT64_C uint64_t

# if __WORDSIZE == 64
#  define __PRI64_PREFIX    "l"
#  define __PRIPTR_PREFIX   "l"
# else
#  define __PRI64_PREFIX    "ll"
#  define __PRIPTR_PREFIX
# endif
# define PRId64     __PRI64_PREFIX "d"

#ifdef __cplusplus
extern "C"
{
#endif

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
//#include <libswscale/swscale.h>
#include <libavutil/time.h>
#include <libavutil/avstring.h>

#ifdef __cplusplus
}//extern "C"
#endif

typedef struct MyAVPacketList
{
    AVPacket pkt;
    struct MyAVPacketList *next;
    int serial;
} MyAVPacketList;

class PacketQueue
{
public:
    PacketQueue();
    int put_private(AVPacket *pkt);
    int put(AVPacket *pkt);
    int put_nullpacket(int stream_index);
    void flush();
    void abort();
    void start();
    int get(AVPacket *pkt, int block, int *serial);

    bool isStop;
    MyAVPacketList *first_pkt, *last_pkt;
    int nb_packets;
    int size;
    int serial;
};

typedef struct Frame
{
    AVFrame *frame;
    int serial;
    double pts;
    double duration;
    int64_t pos; // byte position in the file
    int allocated;
    int width;
    int height;
    AVRational sar;
} Frame;

const int FrameBufferSize = 3;
class FrameBuffer
{
public:
    FrameBuffer();

    /* jump back to the previous frame if available by resetting rindex_shown */
    inline int prev() { int ret = rindex_shown; rindex_shown = 0; return ret; }
    Frame *peek_last() { return &queue[rindex]; }
    Frame *peek_next() { return &queue[(rindex + rindex_shown + 1) % FrameBufferSize]; }
    Frame *peek() { return &queue[(rindex + rindex_shown) % FrameBufferSize]; }

    int init(PacketQueue *pktq_target);
    int destory();
    void signal();
    Frame *peek_writable();
    void push();
    void next();
    void allocated();

    inline int nb_remaining() { return size - rindex_shown; }

    Frame queue[FrameBufferSize];
    int rindex;
    int windex;
    int size;
    int rindex_shown;
    PacketQueue *pktq;
};

#endif // #define _FRAMEQUEUE_H_
