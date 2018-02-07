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
#include <cutils/properties.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <uv.h>

uint32_t camera_mode() {
  uint32_t camera_mode_;
  char prop[PROPERTY_VALUE_MAX];
  property_get("persist.qmmf.ipcweb.camtype", prop, 0);
  camera_mode_ = atoi (prop);
  return camera_mode_;
}

#define BUF_SIZE 64
#define PORT_RECV_CB 3702
#define PORT 3704
#define ADDRESS "239.255.255.250"

enum cam_type {
    CAM_IP,
    CAM_360
};

static uv_tcp_t sub;
static uv_udp_t mcast;

static int first_notification = 1;

static unsigned int ip;
static char _buf[64];

static void mcast_alloc_cb(uv_handle_t *handle, size_t hint, uv_buf_t *buf) {
    buf->base = _buf;
    buf->len = sizeof _buf;
}

static void ucast_xmit_cb(uv_udp_send_t *req, int status) {
    uv_buf_t *buf = req->data;

    free(buf->base);
    free(buf);
    free(req);
}

static void mcast_recv_cb(uv_udp_t *mcast, ssize_t nread, const uv_buf_t *buf,
                          const struct sockaddr *addr, unsigned flags) {
    uv_udp_send_t *req;
    uv_buf_t *_buf;
    char *resp;
    struct in_addr _addr;

    if (nread <= 0) {
        return;
    }

    if (flags == UV_UDP_PARTIAL) {
        return;
    }
    req = malloc(sizeof *req);

    _buf = malloc(sizeof *_buf);
    resp = malloc(BUF_SIZE);
    _buf->base = resp;

    req->data = _buf;

    _addr.s_addr = ip;

    uint32_t camera_mode_ = camera_mode();

    if (camera_mode_ == CAM_IP) {
      _buf->len = snprintf(resp, BUF_SIZE,
        "{\"id\": \"%s\", \"ipv4\": \"%s\"}", "IPCAMERA", inet_ntoa(_addr));
    } else {
      _buf->len = snprintf(resp, BUF_SIZE,
        "{\"id\": \"%s\", \"ipv4\": \"%s\"}", "360CAMERA", inet_ntoa(_addr));
    }

    uv_udp_send(req, mcast, _buf, 1, addr, ucast_xmit_cb);
}

static void sub_close_cb(uv_handle_t *handle) {
    // nothing to do
}

static void sub_alloc_cb(uv_handle_t *handle, size_t hint, uv_buf_t *buf) {
    buf->base = (char *)&ip;
    buf->len = sizeof ip;
}

static void sub_recv_cb(uv_stream_t *sub, ssize_t nread, const uv_buf_t *buf) {
    if (first_notification) {
        struct sockaddr_in addr = {
            0,
        };

        uv_udp_init(sub->loop, &mcast);

        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(PORT_RECV_CB);

        uv_udp_bind(&mcast, (struct sockaddr *)&addr, 0);

        addr.sin_addr.s_addr = ip;

        uv_udp_set_membership(&mcast, ADDRESS,
                              inet_ntoa(addr.sin_addr), UV_JOIN_GROUP);

        uv_udp_recv_start(&mcast, mcast_alloc_cb, mcast_recv_cb);

        first_notification = 0;
    }
}

static void sub_connect_cb(uv_connect_t *req, int status) {
    if (status < 0) {
        uv_close((uv_handle_t *)&sub, sub_close_cb);
        return;
    }

    uv_read_start((uv_stream_t *)&sub, sub_alloc_cb, sub_recv_cb);
}

int main(int argc, char *argv[]) {
    struct sockaddr_in addr = {
        0,
    };
    uv_loop_t *loop;
    uv_connect_t req;

    loop = uv_default_loop();

    uv_tcp_init(loop, &sub);

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(PORT);

    uv_tcp_connect(&req, &sub, (struct sockaddr *)&addr, sub_connect_cb);

    uv_run(loop, UV_RUN_DEFAULT);

    uv_loop_close(loop);

    return 0;
}
