/*
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include <power_state.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <pthread.h>

int sock;
pthread_t pid;
callback notify_client;

void send_acknowledgement(client_ack_t ack)
{
  write(sock, &ack, sizeof(client_ack_t));
}

static void *wait_for_event(void *arg)
{
  int ret;
  power_state_t pwr_state;
  while (read(sock, &pwr_state, sizeof(power_state_t)) > 0) {
    ret = (*notify_client)(pwr_state);
  }
  return NULL;
}

int pwr_state_notification_register(callback cbfunc)
{
  int len, err = -1;
  struct sockaddr_un saun;

  if ((sock = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
    perror("client: socket");
    return err;
  }

  saun.sun_family = AF_UNIX;
  strcpy(saun.sun_path, ADDRESS);
  len = sizeof(saun.sun_family) + strlen(saun.sun_path);

  if (connect(sock, (struct sockaddr*) &saun, len) < 0) {
    perror("client: connect");
    return err;
  }

  if (cbfunc) {
    notify_client = cbfunc;
    pthread_create(&pid, NULL, wait_for_event, NULL);
  }
  return 0;
}
