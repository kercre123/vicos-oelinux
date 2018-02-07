/*
Copyright (c) 2017, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define _GNU_SOURCE
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <libwebsockets.h>
#include <uv.h>

#ifndef likely
#define likely(x) \
    (__builtin_constant_p(x) ? !!(x) : __builtin_expect(!!(x), 1))
#endif
#ifndef unlikely
#define unlikely(x) \
    (__builtin_constant_p(x) ? !!(x) : __builtin_expect(!!(x), 0))
#endif

#define INIT (1)
#define WS_RTSP_PENDING (1 << 1)
#define RTSP_CONNECTING (1 << 2)
#define WS_RTP_WAITING (1 << 3)
#define WS_RTP_PENDING (1 << 4)
#define FORWARDING (1 << 5)
#define SHUTTING_DOWN (1 << 6)

static unsigned int state;

static char ws_buf[2048];

static char xmit_buf[2048];
static size_t xmit_buf_len = LWS_PRE;

static uv_tcp_t rtsp;
static uv_timer_t timer;
static uv_connect_t conn_req;
static uv_write_t rtsp_write_req;

static struct wsi_ctx {
  struct lws *rtsp;
  struct lws *rtp;
} wsi_ctx;

struct wsi_discq_entry {
  LIST_ENTRY(wsi_discq_entry) node;
  struct lws *wsi;
};

static LIST_HEAD( wsi_discq_head, wsi_discq_entry)
wsi_discq;

static int read_stopped = 1;
static int rtsp_new_arrival;
static int rtp_new_arrival;

static int refcnt;

static inline pid_t gettid() {
  return syscall(SYS_gettid);
}

static void teardown() {
  if (--refcnt == 0) {
    read_stopped = 1;
    rtsp_new_arrival = 0;
    rtp_new_arrival = 0;
    xmit_buf_len = LWS_PRE;

    state = INIT;
  }
}

static void rtsp_close_cb(uv_handle_t *handle) {
  teardown();
}

static void alloc_cb(uv_handle_t *handle, size_t len, uv_buf_t *buf) {

  if (unlikely(xmit_buf_len >= sizeof xmit_buf)) {
    buf->base = NULL;
    buf->len = 0;
  } else {
    buf->base = xmit_buf + xmit_buf_len;
    buf->len = sizeof(xmit_buf) - xmit_buf_len;
  }
}

static void rtsp_read_cb(uv_stream_t *stream, ssize_t nread,
                         const uv_buf_t *buf) {
  if (unlikely(nread < 0)) {

    uv_read_stop(stream);
    refcnt = 3;
    lws_callback_on_writable(wsi_ctx.rtsp);
    lws_callback_on_writable(wsi_ctx.rtp);
    uv_close((uv_handle_t *) stream, rtsp_close_cb);

    state = SHUTTING_DOWN;
    return;
  }

  if (unlikely(nread == 0)) {
    return;
  }

  xmit_buf_len += nread;

  if (xmit_buf_len >= sizeof xmit_buf) {
    read_stopped = 1;
    uv_read_stop(stream);
  }

  if (likely(xmit_buf[LWS_PRE] == '$')) {
    lws_callback_on_writable(wsi_ctx.rtp);
    rtp_new_arrival = 1;
  } else {
    lws_callback_on_writable(wsi_ctx.rtsp);
    rtsp_new_arrival = 1;
  }
}

static void timeout_cb(uv_timer_t *timer) {

  switch (state) {
  case WS_RTSP_PENDING:
    refcnt = 1;
    lws_callback_on_writable(wsi_ctx.rtsp);
    break;
  case RTSP_CONNECTING:
  case WS_RTP_WAITING:
    refcnt = 2;
    lws_callback_on_writable(wsi_ctx.rtsp);
    uv_close((uv_handle_t *) &rtsp, rtsp_close_cb);
    break;
  case WS_RTP_PENDING:
    refcnt = 3;
    lws_callback_on_writable(wsi_ctx.rtsp);
    lws_callback_on_writable(wsi_ctx.rtp);
    uv_close((uv_handle_t *) &rtsp, rtsp_close_cb);
    break;
  default:
    return;
  }

  state = SHUTTING_DOWN;
}

static void rtsp_conn_cb(uv_connect_t *req, int status) {
  char *out = ws_buf + LWS_PRE;

  uv_timer_stop(&timer);

  if (status < 0) {
    if (state != RTSP_CONNECTING) {
      return;
    }
    refcnt = 2;
    lws_callback_on_writable(wsi_ctx.rtsp);
    uv_close((uv_handle_t *) &rtsp, rtsp_close_cb);
    state = SHUTTING_DOWN;
    return;
  }

  snprintf(out, sizeof(ws_buf) - LWS_PRE, "INIT 0\r\n\r\n");
  lws_write(wsi_ctx.rtsp, (unsigned char *) out, strlen(out), LWS_WRITE_TEXT);

  uv_timer_start(&timer, timeout_cb, 2000, 0);

  state = WS_RTP_WAITING;
}

static void rtsp_write_cb(uv_write_t *req, int status) {
  if (unlikely(status < 0)) {

    if (state == SHUTTING_DOWN) {
      return;
    }

    refcnt = 3;
    lws_callback_on_writable(wsi_ctx.rtsp);
    lws_callback_on_writable(wsi_ctx.rtp);
    uv_read_stop((uv_stream_t *) &rtsp);
    uv_close((uv_handle_t *) &rtsp, rtsp_close_cb);
    state = SHUTTING_DOWN;
  }

}

static inline int rtsp_conn_start() {
  char dest[16];
  int port;
  char *s;
  struct sockaddr_in addr = { 0, };

  s = strstr(ws_buf, "host");
  if (!s) {
    refcnt = 1;
    state = SHUTTING_DOWN;
    return -1;
  }
  sscanf(s, "host %s", dest);
  dest[strlen(dest)] = 0;

  s = strstr(s, "port");
  if (!s) {
    refcnt = 1;
    state = SHUTTING_DOWN;
    return -1;
  }
  sscanf(s, "port %d", &port);

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr(dest);
  addr.sin_port = htons(port);

  uv_tcp_init(uv_default_loop(), &rtsp);
  uv_tcp_connect(&conn_req, &rtsp, (struct sockaddr *) &addr, rtsp_conn_cb);

  // RTSP_CONNECTING timer
  uv_timer_start(&timer, timeout_cb, 2000, 0);

  state = RTSP_CONNECTING;

  return 0;
}

static int ws_rtsp_recv(struct lws *wsi, void *in, size_t len) {
  uv_buf_t buf;

  memcpy(ws_buf, in, len);
  ws_buf[len] = 0;

  if (unlikely(state == WS_RTSP_PENDING)) {
    return rtsp_conn_start();
  }

  buf.base = ws_buf;
  buf.len = len;
  uv_write(&rtsp_write_req, (uv_stream_t *) &rtsp, &buf, 1, rtsp_write_cb);

  return 0;
}

static void wsi_discq_add(struct lws *wsi) {
  struct wsi_discq_entry *entry = malloc(sizeof *entry);
  entry->wsi = wsi;
  LIST_INSERT_HEAD(&wsi_discq, entry, node);
}

static int wsi_discq_try_discard(struct lws *wsi) {
  struct wsi_discq_entry *entry;

  LIST_FOREACH(entry, &wsi_discq, node)
  {
    if (wsi == entry->wsi) {
      LIST_REMOVE(entry, node);
      free(entry);
      return 1;
    }
  }

  return 0;
}

static void ws_rtsp_close(struct lws *wsi) {
  // stop timer anyway
  uv_timer_stop(&timer);

  wsi_ctx.rtsp = NULL;

  switch (state) {
  case WS_RTSP_PENDING:
    state = INIT;
    return;
  case RTSP_CONNECTING:
  case WS_RTP_WAITING:
    refcnt = 1;
    uv_close((uv_handle_t *) &rtsp, rtsp_close_cb);
    break;
  case FORWARDING:
    uv_read_stop((uv_stream_t *) &rtsp);
  case WS_RTP_PENDING:
    refcnt = 2;
    lws_callback_on_writable(wsi_ctx.rtp);
    uv_close((uv_handle_t *) &rtsp, rtsp_close_cb);
    break;
  case SHUTTING_DOWN:
    teardown();
    return;
  default:
    return;
  }

  state = SHUTTING_DOWN;
}

static int ws_rtsp_cb(struct lws *wsi, enum lws_callback_reasons reason,
                      void *user, void *in, size_t len) {
  switch (reason) {
  case LWS_CALLBACK_GET_THREAD_ID:
    return (int) gettid();
  case LWS_CALLBACK_ESTABLISHED:
    if (state != INIT) {
      wsi_discq_add(wsi);
      return -1;
    }
    wsi_ctx.rtsp = wsi;
    uv_timer_start(&timer, timeout_cb, 2000, 0);
    state = WS_RTSP_PENDING;
    break;
  case LWS_CALLBACK_CLOSED:
    if (wsi_discq_try_discard(wsi))
      break;
    ws_rtsp_close(wsi);
    break;
  case LWS_CALLBACK_RECEIVE:
    if (unlikely(!(state & (WS_RTSP_PENDING | FORWARDING)))) {
      break;
    }

    if (state == WS_RTSP_PENDING)
      uv_timer_stop(&timer);

    return ws_rtsp_recv(wsi, in, len);
  case LWS_CALLBACK_SERVER_WRITEABLE:
    if (unlikely(state == SHUTTING_DOWN)) {
      return -1;
    }
    if (unlikely(!rtsp_new_arrival)) {
      break;
    }

    rtsp_new_arrival = 0;
    {
      char *start = xmit_buf + LWS_PRE;
      char *end = memmem(start, xmit_buf_len - LWS_PRE, "\r\n\r\n", 4);

      if (end) {
        char *body;
        size_t content_len = 0;

        *end = 0;
        body = strcasestr(start, "content-length");
        if (body) {
          memcpy(body, "content-length", strlen("content-length"));
          sscanf(body, "content-length: %zu", &content_len);
        }
        *end = '\r';

        lws_write(wsi_ctx.rtsp, (unsigned char *) start,
            end - start + 4 + content_len, LWS_WRITE_TEXT);
        xmit_buf_len -= end - start + 4 + content_len;
        memmove(start, end + 4 + content_len, xmit_buf_len - LWS_PRE);

        if (xmit_buf_len > LWS_PRE) {
          if (likely(xmit_buf[LWS_PRE] == '$')) {
            lws_callback_on_writable(wsi_ctx.rtp);
            rtp_new_arrival = 1;
          } else {
            lws_callback_on_writable(wsi_ctx.rtsp);
            rtsp_new_arrival = 1;
          }
        }

        if (read_stopped) {
          uv_read_start((uv_stream_t *) &rtsp, alloc_cb, rtsp_read_cb);
          read_stopped = 0;
        }
      }
    }
    break;
  default:
    break;
  }

  return 0;
}

static inline void ws_rtp_ack(struct lws *wsi, void *in, size_t len) {
  char *out = ws_buf + LWS_PRE;

  memcpy(ws_buf, in, len);
  ws_buf[len] = 0;

  snprintf(out, sizeof(ws_buf) - LWS_PRE, "INIT 0\r\n\r\n");
  lws_write(wsi, (unsigned char *) out, strlen(out), LWS_WRITE_TEXT);
}

static void ws_rtp_close(struct lws *wsi) {
  uv_timer_stop(&timer);

  wsi_ctx.rtp = NULL;

  switch (state) {
  case FORWARDING:
    uv_read_stop((uv_stream_t *) &rtsp);
  case WS_RTP_PENDING:
    refcnt = 2;
    lws_callback_on_writable(wsi_ctx.rtsp);
    uv_close((uv_handle_t *) &rtsp, rtsp_close_cb);
    break;
  case SHUTTING_DOWN:
    teardown();
    return;
  default:
    return;
  }

  state = SHUTTING_DOWN;
}

static int ws_rtp_cb(struct lws *wsi, enum lws_callback_reasons reason,
                     void *user, void *in, size_t len) {
  switch (reason) {
  case LWS_CALLBACK_GET_THREAD_ID:
    return (int) gettid();
  case LWS_CALLBACK_ESTABLISHED:
    if (state != WS_RTP_WAITING) {
      wsi_discq_add(wsi);
      return -1;
    }
    uv_timer_stop(&timer);
    wsi_ctx.rtp = wsi;
    uv_timer_start(&timer, timeout_cb, 2000, 0);
    state = WS_RTP_PENDING;
    break;
  case LWS_CALLBACK_CLOSED:
    if (wsi_discq_try_discard(wsi)) {
      break;
    }
    ws_rtp_close(wsi);
    break;
  case LWS_CALLBACK_RECEIVE:
    if (unlikely(state != WS_RTP_PENDING)) {
      break;
    }
    uv_timer_stop(&timer);
    ws_rtp_ack(wsi, in, len);
    uv_read_start((uv_stream_t *) &rtsp, alloc_cb, rtsp_read_cb);
    read_stopped = 0;
    state = FORWARDING;
    break;
  case LWS_CALLBACK_SERVER_WRITEABLE:
    if (unlikely(state == SHUTTING_DOWN)) {
      return -1;
    }

    if (unlikely(!rtp_new_arrival)) {
      break;
    }
    rtp_new_arrival = 0;

    {
      char *start = xmit_buf + LWS_PRE;
      struct magic_header {
        char magic;
        unsigned char channel;
        unsigned short len;
      }*h = (struct magic_header *) start;
      unsigned short len = ntohs(h->len);

      if (len + LWS_PRE + sizeof(*h) <= xmit_buf_len) {
        lws_write(wsi_ctx.rtp, (unsigned char *) start, len + sizeof(*h),
            LWS_WRITE_BINARY);
        xmit_buf_len -= len + sizeof(*h);
        memmove(start, start + len + sizeof(*h), xmit_buf_len - LWS_PRE);

        if (xmit_buf_len > LWS_PRE) {
          if (likely(xmit_buf[LWS_PRE] == '$')) {
            lws_callback_on_writable(wsi_ctx.rtp);
            rtp_new_arrival = 1;
          } else {
            lws_callback_on_writable(wsi_ctx.rtsp);
            rtsp_new_arrival = 1;
          }
        }

        if (read_stopped) {
          uv_read_start((uv_stream_t *) &rtsp, alloc_cb, rtsp_read_cb);
          read_stopped = 0;
        }
      }
    }
    break;
  default:
    break;
  }

  return 0;
}

static struct lws_protocols protocols[] = { { "rtsp", ws_rtsp_cb, 0, 4096, }, {
    "rtp", ws_rtp_cb, 0, 4096, }, { NULL, } };

int main(int argc, char *argv[]) {
  uv_loop_t *loop;
  struct lws_context *lws_ctx;
  struct lws_context_creation_info lws_info = { 0, };

  lws_info.port = atoi(argv[1]);
  lws_info.iface = NULL;
  lws_info.protocols = protocols;
  lws_info.uid = lws_info.gid = -1;
  lws_info.options = LWS_SERVER_OPTION_LIBUV | LWS_SERVER_OPTION_VALIDATE_UTF8;

  lws_ctx = lws_create_context(&lws_info);
  if (!lws_ctx) {
    lwsl_err("lws init failed\n");
    return 1;
  }

  lws_uv_sigint_cfg(lws_ctx, 0, NULL);
  // If UIs reject certs of WSS, openssl will raise SIGPIPE
  signal(SIGPIPE, SIG_IGN);

  loop = uv_default_loop();

  uv_timer_init(loop, &timer);

  lws_uv_initloop(lws_ctx, loop, 0);

  state = INIT;

  uv_run(loop, UV_RUN_DEFAULT);

  lws_context_destroy(lws_ctx);

  return 0;
}
