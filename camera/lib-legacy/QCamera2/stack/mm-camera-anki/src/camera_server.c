/**
 * File: camera_server.c
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: Anki Camera Server: Single client server that allows a client process               
 *              to access processed camera frames.
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include "camera_format.h"
#include "camera_server.h"
#include "camera_process.h"
#include "log.h"


#define ANKI_CAMERA_SOCKET_DIR      "/var/run/mm-anki-camera"
#define ANKI_CAMERA_SOCKET_NAME     "camera-server"
#define ANKI_CAMERA_SOCKET_PATH     ANKI_CAMERA_SOCKET_DIR "/" ANKI_CAMERA_SOCKET_NAME

// forward declarations
int disconnect_client(struct server_ctx* ctx);
int shutdown_camera(struct server_ctx* ctx);


int configure_socket(int socket)
{
  int flags = fcntl(socket, F_GETFL, 0);
  if (flags == -1) {
    loge("%s: failed to get socket settings: %s", __func__, strerror(errno));
    return -1;
  }

  flags |= O_NONBLOCK;
  flags = fcntl(socket, F_SETFL, flags);
  if (flags == -1) {
    loge("%s: failed to set socket nonblocking: %s", __func__, strerror(errno));
    return -1;
  }

  const int enable = 1;
  const int status = setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

  return status;
}

int create_socket(const char* socket_path)
{
  assert(socket_path != NULL);
  int fd = socket(PF_LOCAL, SOCK_DGRAM, 0);
  if (fd == -1) {
    loge("%s: failed to create socket: %s", __func__, strerror(errno));
    return -1;
  }

  int rc = -1;

  // ensure path exists
  struct stat st = {0};
  if (stat(ANKI_CAMERA_SOCKET_DIR, &st) == -1) {
    mkdir(ANKI_CAMERA_SOCKET_DIR, 0770);
  }


  rc = configure_socket(fd);
  if (rc != 0) {
    close(fd);
    loge("%s: failed to configure socket", __func__);
    return -1;
  }

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);

  // cleanup existing socket
  (void)unlink(socket_path);

  rc = bind(fd, (struct sockaddr*)&addr, sizeof(addr));
  if (rc == -1) {
    loge("%s: failed to bind socket: %s", __func__, strerror(errno));
    close(fd);
    return rc;
  }

  return fd;
}

//
// Outgoing Data
//

ssize_t write_fd(int fd, void *ptr, size_t nbytes, int sendfd)
{
  struct msghdr   msg;
  struct iovec    iov[1];

  union {
    struct cmsghdr    cm;
    char              control[CMSG_SPACE(sizeof(int))];
  } control_un;
  struct cmsghdr  *cmptr;

  msg.msg_control = control_un.control;
  msg.msg_controllen = sizeof(control_un.control);

  cmptr = CMSG_FIRSTHDR(&msg);
  cmptr->cmsg_len = CMSG_LEN(sizeof(int));
  cmptr->cmsg_level = SOL_SOCKET;
  cmptr->cmsg_type = SCM_RIGHTS;
  *((int *) CMSG_DATA(cmptr)) = sendfd;

  msg.msg_name = NULL;
  msg.msg_namelen = 0;

  iov[0].iov_base = ptr;
  iov[0].iov_len = nbytes;
  msg.msg_iov = iov;
  msg.msg_iovlen = 1;

  return (sendmsg(fd, &msg, 0));
}

int write_outgoing_data(struct server_ctx* ctx)
{
  uint32_t i;
  int rc;
  uint32_t msg_count = ctx->tx_cursor;
  for (i = 0; i < msg_count; ++i) {
    struct anki_camera_msg* msg = &(ctx->tx_packets[i]);
    logv("%s: send msg to:%d msg_id=%d fd=%d", __FUNCTION__, ctx->fd, msg->msg_id, msg->fd);
    if (msg->fd >= 0) {
      rc = write_fd(ctx->fd, msg, sizeof(struct anki_camera_msg), msg->fd);
    }
    else {
      rc = write(ctx->fd, msg, sizeof(struct anki_camera_msg));
    }
    ctx->tx_cursor--;
    if (rc <= 0) {
      loge("%s: error sending msg %d : %d : %s",
           __FUNCTION__, msg->msg_id, rc, strerror(errno));
      break;
    }
    logv("%s: sent msg: %d", __FUNCTION__, msg->msg_id);
  }
  return rc;
}

struct anki_camera_msg* enqueue_message(struct server_ctx* ctx, anki_camera_msg_id_t msg_id)
{
  uint32_t cursor = ctx->tx_cursor;
  if (cursor > ANKI_CAMERA_MAX_PACKETS) {
    loge("%s: tx buffer full: %u", __FUNCTION__, ctx->tx_cursor);
    return NULL;
  }
  struct anki_camera_msg *msg = &ctx->tx_packets[cursor];
  memset(msg, 0, sizeof(struct anki_camera_msg));
  msg->msg_id = msg_id;
  msg->fd = -1;
  ctx->tx_cursor = cursor + 1;
  return msg;
}

//
// Incoming Data
//

int process_one_message(struct server_ctx* ctx, struct anki_camera_msg* msg)
{
  int rc = 0;
  const anki_camera_msg_id_t msg_id = msg->msg_id;
  switch (msg_id) {
  case ANKI_CAMERA_MSG_C2S_HEARTBEAT: {
    (void)enqueue_message(ctx, ANKI_CAMERA_MSG_S2C_HEARTBEAT);
  }
  break;
  case ANKI_CAMERA_MSG_C2S_CLIENT_REGISTER: {
    // allocate mem
    if (!is_camera_capture_initialized(&ctx->camera)) {
      logv("%s: initialize capture buffer", __FUNCTION__);
      rc = camera_capture_init(&ctx->camera, ctx->capture_params.pixel_format);
      if (rc != 0) {
        loge("%s: error initializing camera capture", __FUNCTION__);
        break;
      }
    }

    // send FD to client
    logv("%s: sending buffer.fd=%d", __FUNCTION__, ctx->camera.buffer.fd);
    struct anki_camera_msg* buf_msg = enqueue_message(ctx, ANKI_CAMERA_MSG_S2C_BUFFER);
    buf_msg->fd = ctx->camera.buffer.fd;
    // send buffer size in payload
    memcpy(buf_msg->payload, &ctx->camera.buffer.size, sizeof(ctx->camera.buffer.size));

    // ack with status
    struct anki_camera_msg* status_msg = enqueue_message(ctx, ANKI_CAMERA_MSG_S2C_STATUS);
    status_msg->payload[0] = ANKI_CAMERA_MSG_C2S_CLIENT_REGISTER;
  }
  break;
  case ANKI_CAMERA_MSG_C2S_CLIENT_UNREGISTER: {
    // graceful teardown
    (void) shutdown_camera(ctx);
    struct anki_camera_msg* status_msg = enqueue_message(ctx, ANKI_CAMERA_MSG_S2C_STATUS);
    status_msg->payload[0] = ANKI_CAMERA_MSG_C2S_CLIENT_UNREGISTER;
  }
  break;
  case ANKI_CAMERA_MSG_C2S_START: {
    rc = camera_capture_start(&ctx->camera, &ctx->capture_params);

    // ack with status
    struct anki_camera_msg* rsp = enqueue_message(ctx, ANKI_CAMERA_MSG_S2C_STATUS);
    rsp->payload[0] = ANKI_CAMERA_MSG_C2S_START;
  }
  break;
  case ANKI_CAMERA_MSG_C2S_STOP: {
    rc = camera_capture_stop(&ctx->camera);
    struct anki_camera_msg* rsp = enqueue_message(ctx, ANKI_CAMERA_MSG_S2C_STATUS);
    rsp->payload[0] = ANKI_CAMERA_MSG_C2S_STOP;
  }
  break;
  case ANKI_CAMERA_MSG_C2S_PARAMS: {
    anki_camera_msg_params_payload_t* payload = (anki_camera_msg_params_payload_t*)msg->payload;
    switch(payload->id) {
      case ANKI_CAMERA_MSG_C2S_PARAMS_ID_EXP: {
        anki_camera_exposure_t exposure;
        memcpy(&exposure, payload->data, sizeof(exposure));
        camera_capture_set_exposure(exposure);
      }
      break;
      case ANKI_CAMERA_MSG_C2S_PARAMS_ID_AWB: {
        anki_camera_awb_t awb;
        memcpy(&awb, payload->data, sizeof(awb));
        camera_capture_set_awb(awb);
      }
      break;
      case ANKI_CAMERA_MSG_C2S_PARAMS_ID_FORMAT: {
        anki_camera_pixel_format_t format;
        memcpy(&format, payload->data, sizeof(format));
        rc = camera_capture_set_format(&ctx->camera, format);

        logv("%s: sending buffer.fd=%d", __FUNCTION__, ctx->camera.buffer.fd);
        struct anki_camera_msg* buf_msg = enqueue_message(ctx, ANKI_CAMERA_MSG_S2C_BUFFER);
        buf_msg->fd = ctx->camera.buffer.fd;
        // send buffer size in payload
        memcpy(buf_msg->payload, &ctx->camera.buffer.size, sizeof(ctx->camera.buffer.size));
      }
      break;
      case ANKI_CAMERA_MSG_C2S_PARAMS_ID_SNAPSHOT: {
        if(payload->data[0])
        {
          rc = camera_capture_start_snapshot();
        }
        else
        {
          rc = camera_capture_stop_snapshot();
        }
      }
      break;
    }
  }
  break;
  default:
    loge("%s: received unknown message: %d", __FUNCTION__, msg_id);
    rc = -1;
    break;
  }
  return rc;
}

int process_incoming_messages(struct server_ctx* ctx)
{
  uint32_t i;
  int rc;
  uint32_t msg_count = ctx->rx_cursor;
  for (i = 0; i < msg_count; ++i) {
    struct anki_camera_msg* msg = &(ctx->rx_packets[i]);
    rc = process_one_message(ctx, msg);
    ctx->rx_cursor--;
    if (rc != 0) {
      loge("%s: error processing message: %d", __func__, msg->msg_id);
      break;
    }
  }
  logv("%s: processed %u incoming messages", __func__, i);
  return rc;
}

int read_incoming_data(struct server_ctx* ctx)
{
  uint8_t buffer[sizeof(struct anki_camera_msg)];
  struct sockaddr_un saddr;
  socklen_t saddrlen = sizeof(saddr);

  int rc = -1;
  do {
    memset(buffer, 0, sizeof(buffer));
    rc = recvfrom(ctx->fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&saddr, &saddrlen);
    if (rc < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        rc = 0;
      }
      else {
        loge("%s: recv() failed: %s", __func__, strerror(errno));
        rc = -1; // indicate read failed
      }
    }
    else if (rc > 0) {
      // connect if necessary
      if (!ctx->has_client) {
        logv("%s: connect %d", __func__, ctx->fd);
        int c = connect(ctx->fd, (struct sockaddr *) &saddr, saddrlen);
        if (c != 0) {
          loge("%s: client connect failed: %s", __func__, strerror(errno));
          rc = -1;
          break;
        }
        ctx->has_client = 1;
      }
      // copy bytes into rx buffer
      int bytes_read = rc;

      if (ctx->rx_cursor == ANKI_CAMERA_MAX_PACKETS) {
        loge("%s: No more space, dropping packet", __func__);
        rc = -1;
        break;
      }

      struct anki_camera_msg* msg = &(ctx->rx_packets[ctx->rx_cursor]);
      ctx->rx_cursor++;
      memcpy(msg, buffer, sizeof(*msg));
      logv("%s: received msg: %d", __func__, msg->msg_id);
    }
  }
  while (rc > 0);

  if (rc == 0) {
    rc = process_incoming_messages(ctx);
  }

  return rc;
}

int disconnect_client(struct server_ctx* ctx)
{
  int rc = 0;
  if (ctx->fd >= 0) {
    struct sockaddr saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sa_family = AF_UNSPEC;
    int rc = connect(ctx->fd, &saddr, sizeof(saddr));
    if (rc != 0) {
      // log error, but continue to clear has_client flag
      loge("%s: failed to disconnect socket: %s", __func__, strerror(errno));
    }
  }

  // regardless of outcome, reset flag to allow new connection
  ctx->has_client = 0;

  return rc;
}

int shutdown_camera(struct server_ctx* ctx)
{
  int rc = 0;
  if (is_camera_capture_initialized(&ctx->camera)) {
    if (is_camera_capture_running(&ctx->camera)) {
      rc = camera_capture_stop(&ctx->camera);
      if (rc != 0) {
        loge("%s: error stopping camera", __func__);
      }
    }
    if (is_camera_capture_ready(&ctx->camera)) {
      rc = camera_capture_release(&ctx->camera);
      if (rc != 0) {
        loge("%s: error releasing camera", __func__);
      }
    }
  }

  return rc;
}

//
// Event Loop
//

int event_loop(struct server_ctx* ctx)
{
  struct timeval timeout;
  struct timeval* timeout_p = NULL;
  fd_set read_fds;
  fd_set write_fds;

  int max_fd = ctx->fd;

  ctx->is_running = 1;
  int rc = -1;
  do {
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_SET(ctx->fd, &read_fds);

    if (ctx->tx_cursor > 0) {
      FD_SET(ctx->fd, &write_fds);
    }

    if (ctx->has_client) {
      // If a client is connected, we expect to receive a heartbeat every 500 millisec
      timeout_p = &timeout;
      timeout_p->tv_sec  = 0;
      timeout_p->tv_usec = 500000;
    }
    else {
      // Otherwise, wait forever for a connection
      timeout_p = NULL;
    }
    rc = select(max_fd + 1, &read_fds, &write_fds, NULL, timeout_p);

    if (rc < 0) {
      loge("%s: select failed %d : %s", __FUNCTION__,
           errno, strerror(errno));
      break;
    }

    if (rc == 0) {
      // select timeout: we stop receiving messages from our connected client.
      if (ctx->params.exit_on_disconnect) {
        // If we our parent process is expecting us to exit, break out of event loop
        break;
      }
      // otherwise, reset state and keep running
      (void)disconnect_client(ctx);
      (void)shutdown_camera(ctx);
    }

    if (rc > 0) {
      if (FD_ISSET(ctx->fd, &write_fds)) {
        rc = write_outgoing_data(ctx);
        if (rc == -1) {
          break;
        }
      }
      if (FD_ISSET(ctx->fd, &read_fds)) {
        rc = read_incoming_data(ctx);
        if (rc == -1) {
          break;
        }
      }
    }
  }
  while (ctx->is_running);

  // Do not explicitly handle error cases.
  // When this loop exits, we want the program to cleanup & terminate.
  // It will be restarted by the OS, allowing the client to establish a new connection.
  if (rc < 0) {
    loge("%s: event_loop exited with error: %d", __func__, rc);
  }

  // cleanup
  (void)disconnect_client(ctx);
  (void)shutdown_camera(ctx);

  return rc;
}

//
// Public API
//

int create_server(struct server_ctx* ctx, int socket_fd)
{
  memset(ctx, 0, sizeof(struct server_ctx));
  ctx->fd = -1;

  if (socket_fd < 0) {
    // create socket
    socket_fd = create_socket(ANKI_CAMERA_SOCKET_PATH);
    if (socket_fd < 0) {
      return socket_fd;
    }
    ctx->owns_socket = 1;
  }
  else {
    int rc = configure_socket(socket_fd);
    if (rc != 0) {
      return rc;
    }
  }

  ctx->fd = socket_fd;

  return 0;
}


int destroy_server(struct server_ctx* ctx)
{
  int rc = 0;
  if (ctx->owns_socket && (ctx->fd >= 0)) {
    rc = close(ctx->fd);
  }

  memset(ctx, 0, sizeof(struct server_ctx));

  if (unlink(ANKI_CAMERA_SOCKET_PATH) != 0) {
    loge("%s: unlink socket failed", __func__);
  }

  return rc;
}

int start_server(struct server_ctx* ctx)
{

  if (ctx->params.autostart_camera) {
    if (!is_camera_capture_initialized(&ctx->camera)) {
      logd("%s: initialize capture buffer", __func__);
      int rc = camera_capture_init(&ctx->camera, ctx->capture_params.pixel_format);
      if (rc != 0) {
        loge("%s: error initializing camera capture", __func__);
      }
    }
    camera_capture_start(&ctx->camera, &ctx->capture_params);
  }

  int rc = event_loop(ctx);
  return rc;
}

int stop_server(struct server_ctx* ctx)
{
  ctx->request_close = 1;
  ctx->is_running = 0;
  return 0;
}
