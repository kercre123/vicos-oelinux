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
#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>
#include <pthread.h>

#define POP_WAIT_DELAY 10000 /* us */

namespace qmmf {

enum QueueType {
  REALTIME = 0,
  COMPLETE,
  UNKNOWN_TYPE
};

struct AVPacket {
  uint8_t* data;
  size_t size;
  int64_t timestamp;
};

struct AVNode {
  struct AVNode* prev;
  struct AVNode* next;
  void* data;
};

struct AVQueue {
  pthread_mutex_t mutex;
  pthread_cond_t condv;
  AVNode* head_node;
  QueueType type;
  int max_size;
  int queue_size;
  bool overflow;
  int delay;
  char* pps;
  int pps_size;
  bool is_pps;
};

#ifdef __cplusplus
extern "C" {
#endif

extern int AVQueueInit(AVQueue** p, QueueType queue_type, int queue_size,
                       int delay_count);
extern int AVQueuePushHead(AVQueue* p, void* data);
extern void* AVQueuePopTail(AVQueue* p);
extern int AVQueueSize(AVQueue* p);
extern void AVQueueAbort(AVQueue* p, void (*func)(void*));
extern void AVQueueWake(AVQueue* p);
extern void AVQueueFree(AVQueue** p, void (*func)(void*));
extern void AVFreePacket(void* databuf);

#ifdef __cplusplus
}

} //namespace qmmf ends

#endif
#endif /* QUEUE_H */
