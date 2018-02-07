/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <uv.h>

#ifndef offsetof
#define offsetof(a, b) __builtin_offsetof(a, b)
#endif

#ifndef container_of
#define container_of(ptr, type, member)                    \
    ({                                                     \
        const typeof(((type *)0)->member) *__mptr = (ptr); \
        (type *)((char *)__mptr - offsetof(type, member)); \
    })
#endif

struct subscriber {
  LIST_ENTRY(subscriber) node;
  uv_tcp_t tcp;
};

struct sub_buf {
  int counter;
  uv_buf_t buf;
  unsigned int ip;
};

static LIST_HEAD( subscribers, subscriber) subscribers;

static int gNeed_notification;

static void subscriber_incoming_cb(uv_stream_t *srv, int status) {
  struct subscriber *sub;

  if (status < 0) {
    return;
  }
  sub = malloc(sizeof *sub);
  uv_tcp_init(srv->loop, &sub->tcp);
  LIST_INSERT_HEAD(&subscribers, sub, node);

  uv_accept(srv, (uv_stream_t *) &sub->tcp);

  gNeed_notification++;
}

static void subscriber_close_cb(uv_handle_t *handle) {
  struct subscriber *sub;

  sub = container_of((uv_tcp_t * )handle, struct subscriber, tcp);

  free(sub);
}

static void notify_done(uv_write_t *req, int status) {
  struct sub_buf *sbuf = req->data;

  if (--sbuf->counter == 0) {
    free(sbuf);
  }

  if (status < 0) {
    struct subscriber *sub;

    sub = container_of((uv_tcp_t * )req->handle, struct subscriber, tcp);

    LIST_REMOVE(sub, node);

    uv_close((uv_handle_t *) &sub->tcp, subscriber_close_cb);

    gNeed_notification--;
  }

  free(req);
}

static inline void flush_subscribers(const unsigned int ip) {
  struct subscriber *sub;
  struct sub_buf *sbuf;

  sbuf = malloc(sizeof *sbuf);
  sbuf->buf.base = (char *) &sbuf->ip;
  sbuf->buf.len = sizeof sbuf->ip;
  sbuf->ip = ip;
  sbuf->counter = gNeed_notification;

  LIST_FOREACH(sub, &subscribers, node)
  {
    uv_write_t *req = malloc(sizeof *req);

    req->data = sbuf;

    uv_write(req, (uv_stream_t *) &sub->tcp, &sbuf->buf, 1, notify_done);
  }
}

static void event_cb(uv_poll_t *ev, int status, int events) {
  ssize_t nread;
  char ip[20];
  char buf[1024];

  if (status < 0) {
    return;
  }

  nread = read(0, buf, sizeof buf);
  if (nread <= 0) {
    return;
  }

  buf[nread] = 0;

  if (strncmp(buf, "Deleted", 7)) {
    char *start;

    start = strstr(buf, "inet ");
    if (!start) {
      return;
    }
    sscanf(start + 5, "%s", ip);
    start = strrchr(ip, '/');
    *start = 0;

    if (gNeed_notification) {
      flush_subscribers(inet_addr(ip));
    }
  }
}

int main(int argc, char *argv[]) {
  uv_loop_t *loop;
  uv_tcp_t srv;
  uv_poll_t ev;
  struct sockaddr_in addr = { 0, };

  signal(SIGPIPE, SIG_IGN);

  loop = uv_default_loop();

  uv_tcp_init(loop, &srv);

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  addr.sin_port = htons(3704);

  uv_tcp_bind(&srv, (struct sockaddr *) &addr, 0);

  uv_listen((uv_stream_t *) &srv, 1, subscriber_incoming_cb);

  uv_poll_init(loop, &ev, 0);

  uv_poll_start(&ev, UV_READABLE, event_cb);

  uv_run(loop, UV_RUN_DEFAULT);

  uv_loop_close(loop);

  return 0;
}
