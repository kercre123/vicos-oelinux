/**
 * File: camera_server.h
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

#ifndef __mm_anki_camera_server_h__
#define __mm_anki_camera_server_h__

#include <stdint.h>
#include "camera_process.h"

#define ANKI_CAMERA_MAX_PACKETS     12
#define ANKI_CAMERA_MSG_PAYLOAD_LEN 128

typedef enum {
  ANKI_CAMERA_MSG_C2S_HEARTBEAT,
  ANKI_CAMERA_MSG_C2S_CLIENT_REGISTER,
  ANKI_CAMERA_MSG_C2S_CLIENT_UNREGISTER,
  ANKI_CAMERA_MSG_C2S_START,
  ANKI_CAMERA_MSG_C2S_STOP,
  ANKI_CAMERA_MSG_C2S_PARAMS,
  ANKI_CAMERA_MSG_S2C_STATUS,
  ANKI_CAMERA_MSG_S2C_BUFFER,
  ANKI_CAMERA_MSG_S2C_HEARTBEAT,
} anki_camera_msg_id_t;

struct anki_camera_msg {
  anki_camera_msg_id_t msg_id;
  uint32_t version;
  uint32_t client_id;
  int fd;
  uint8_t payload[ANKI_CAMERA_MSG_PAYLOAD_LEN];
};

typedef enum {
  ANKI_CAMERA_MSG_C2S_PARAMS_ID_EXP,
  ANKI_CAMERA_MSG_C2S_PARAMS_ID_AWB,
  ANKI_CAMERA_MSG_C2S_PARAMS_ID_FORMAT,
  ANKI_CAMERA_MSG_C2S_PARAMS_ID_SNAPSHOT,
} anki_camera_params_id_t;

typedef struct {
  anki_camera_params_id_t id;
  uint8_t data[sizeof(((struct anki_camera_msg*)0)->payload) - sizeof(anki_camera_params_id_t)];
} anki_camera_msg_params_payload_t;

struct server_params {
  char output_path[256];
  uint8_t exit_on_disconnect;
  uint8_t autostart_camera;
  uint8_t debug_dump_images;
};

struct server_ctx {
  int fd;
  int owns_socket;
  int is_running;
  int camera_running;
  int request_close;
  int has_client;

  struct server_params params;
  struct anki_camera_capture_params capture_params;
  struct anki_camera_capture camera;

  uint32_t rx_cursor;
  struct anki_camera_msg rx_packets[ANKI_CAMERA_MAX_PACKETS];
  uint32_t tx_cursor;
  struct anki_camera_msg tx_packets[ANKI_CAMERA_MAX_PACKETS];
};

int create_server(struct server_ctx* ctx, int socket_fd);
int destroy_server(struct server_ctx* ctx);
int start_server(struct server_ctx* ctx);
int stop_server(struct server_ctx* ctx);

#endif // __mm_anki_camera_server_h__
