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
#include <malloc.h>
#include <qmmf-sdk/qmmf_queue.h>
#include <utils/Timers.h>
#include <sys/time.h>
#include <string.h>
#include <errno.h>

namespace qmmf {

/******************************************************************************
  API Function define
 ******************************************************************************/
int AVQueueInit(AVQueue** p, QueueType queue_type,
                int queue_size, int delay_count) {
  if (*p != NULL) {
    return -1;
  }

  if (queue_type >= UNKNOWN_TYPE) {
    return -1;
  }

  if ((REALTIME == queue_type) && (delay_count > queue_size)) {
    return -1;
  }

  *p = (AVQueue*)malloc(sizeof(AVQueue));
  if ((*p) == NULL) {
    return -1;
  }

  (*p)->head_node = (AVNode*)malloc(sizeof(AVNode));
  if (((*p)->head_node) == NULL) {
    free((*p));
    (*p) = NULL;
    return -1;
  }

  (*p)->head_node->prev = (*p)->head_node;
  (*p)->head_node->next = (*p)->head_node;
  (*p)->head_node->data = 0;
  if ((REALTIME == queue_type)) {
    (*p)->max_size = queue_size;
    (*p)->delay = delay_count;
  } else {
    (*p)->max_size = 0;
    (*p)->delay = 0;
  }

  (*p)->overflow = false;
  (*p)->type = queue_type;
  (*p)->queue_size = 0;
  (*p)->pps = NULL;
  (*p)->pps_size = 0;
  (*p)->is_pps = false;
  pthread_mutex_init(&(*p)->mutex, 0);
  pthread_cond_init(&(*p)->condv, 0);
  return 1;
}

void AVQueueWake(AVQueue* p) {
  if (p == NULL) {
    return;
  }
  pthread_mutex_lock(&p->mutex);
  if (p->queue_size == 0) {
    p->queue_size = -1;
  }
  pthread_cond_broadcast(&p->condv);
  pthread_mutex_unlock(&p->mutex);
}

void AVQueueFree(AVQueue** p, void (*func)(void*)) {
  if ((*p) == NULL) {
    return;
  }
  AVQueueAbort((*p), func);
  pthread_mutex_destroy(&(*p)->mutex);
  pthread_cond_destroy(&(*p)->condv);
  if ((NULL != (*p)->head_node)) {
    free((*p)->head_node);
  }
  if ((NULL != (*p)->pps)) {
    free((*p)->pps);
  }
  free((*p));
  (*p) = NULL;
}

int AVQueuePushHead(AVQueue* p, void* databuf) {
  AVNode* node = NULL;
  AVNode* head = NULL;
  void* result = NULL;
  if (p == NULL) {
    return -1;
  }

  node = (AVNode*)malloc(sizeof(AVNode));
  if (node == NULL) {
    return -1;
  }
  node->data = databuf;
  pthread_mutex_lock(&p->mutex);
  if ((REALTIME == p->type)) {
    if ((p->queue_size >= p->max_size)) {
      AVNode* node1;
      p->overflow = true;
      p->queue_size--;
      head = p->head_node;
      node1 = head->prev;
      head->prev = node1->prev;
      node1->prev->next = head;
      result = node1->data;
      free(node1);
      AVFreePacket(result);
    }
  }

  p->queue_size++;
  head = p->head_node;
  node->next = head->next;
  head->next->prev = node;
  node->prev = head;
  head->next = node;
  pthread_cond_signal(&p->condv);
  pthread_mutex_unlock(&p->mutex);
  return 0;
}

int AVQueueSize(AVQueue* p) {
  int size = 0;
  if (p == NULL) {
    return -1;
  }
  pthread_mutex_lock(&p->mutex);
  size = p->queue_size;
  pthread_mutex_unlock(&p->mutex);
  return size;
}

void* AVQueuePopTail(AVQueue* p) {
  AVNode* node = NULL;
  AVNode* head = NULL;
  void* result = NULL;
  if (p == NULL) {
    return NULL;
  }
  pthread_mutex_lock(&p->mutex);
  while (p->queue_size == 0) {
    struct timeval now;
    struct timespec timeout;
    gettimeofday(&now, NULL);
    memset(&timeout, 0, sizeof(timeout));
    now.tv_usec += POP_WAIT_DELAY;
    if (now.tv_usec / 1000000) {
      timeout.tv_sec = now.tv_sec + (now.tv_usec / 1000000);
      timeout.tv_nsec = us2ns(now.tv_usec % 1000000);
    } else {
      timeout.tv_sec = now.tv_sec;
      timeout.tv_nsec = us2ns(now.tv_usec);
    }
    int ret = pthread_cond_timedwait(&p->condv, &p->mutex, &timeout);
    if (ETIMEDOUT == ret) {
      pthread_mutex_unlock(&p->mutex);
      return result;
    }
  }

  if (p->queue_size > 0) {
    if ((true == p->overflow)) {
      while ((p->queue_size > p->delay)) {
        p->queue_size--;
        head = p->head_node;
        node = head->prev;
        head->prev = node->prev;
        node->prev->next = head;
        result = node->data;
        free(node);
        AVFreePacket(result);
      }
      p->overflow = false;
    }
    p->queue_size--;
    head = p->head_node;
    node = head->prev;
    head->prev = node->prev;
    node->prev->next = head;
    result = node->data;
    free(node);
  } else {
    result = NULL;
  }
  pthread_mutex_unlock(&p->mutex);
  return result;
}

void AVQueueAbort(AVQueue* p, void (*func)(void*)) {
  AVNode* node = NULL;
  void* data = NULL;
  if (p == NULL) {
    return;
  }
  pthread_mutex_lock(&p->mutex);
  while (p->queue_size > 0) {
    node = p->head_node->next;
    data = node->data;
    if (data && func) {
      func(data);
    }
    p->head_node->next = node->next;
    free(node);
    p->queue_size--;
  }
  p->head_node->next = p->head_node;
  p->head_node->prev = p->head_node;
  pthread_mutex_unlock(&p->mutex);
}

void AVFreePacket(void* data_buf) {
  if (data_buf != NULL) {
    if ((NULL != ((AVPacket*)data_buf)->data)) {
      free(((AVPacket*)data_buf)->data);
    }
    free(data_buf);
  }
}

} //namespace qmmf ends

