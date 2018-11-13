/*
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * Copyright (c) 2011,2013 The Linux Foundation. All rights reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <linux/input.h>
#include <errno.h>
#include <sys/un.h>
#include <pthread.h>
#include <power_state.h>
#include <CwBase.h>
#include <CanWrapper.h>
#include <CwFrame.h>

//#define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
#define DEBUG_LOG(...) printf(__VA_ARGS__);
#else
#define DEBUG_LOG(...)
#endif

#define ERR(...) fprintf(stderr, "I:" __VA_ARGS__)

#define MAXEVENTS                   1
#define MAX_CLIENTS_COUNT           10
#define WAIT_TIME_MS                350
#define PWR_KEY_DEV                 "/dev/input/event0"
#define SHUTDOWN_TIME_US            1000000
#define CFG_FILE_PATH               "/etc/power_state.conf"

typedef struct can_frame_id {
  int sys_suspend;
  int sys_resume;
  int sys_shutdown;
} can_frame_id_t;

int *client_fd;
int client_count = 0;
int ack = 0;
pthread_mutex_t ack_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ack_wait = PTHREAD_COND_INITIALIZER;
CanWrapper * pTheCh = 0;
int suspended = 0;
char gpio_num[3];
int gpio;
can_frame_id_t frame_id;

void send_notification(power_state_t pwr_state) {
  int i;
  struct timeval curr_time;
  struct timespec wait_time;

  /* Acquire wakelock so that the system doesnt enter suspend/resume
   * until the client has stored the NV information and other
   * calibration data and get an acknowledgement from the client
   */
  system("echo power_manager_daemon > /sys/power/wake_lock");
  ack = 0;
  pthread_mutex_lock(&ack_mutex);
  for (i = 0; i < client_count; i++)
  {
    if (client_fd[i] != -1) {
      write(client_fd[i], &pwr_state, sizeof(power_state_t));
      ack |= (1 << i);
    }
  }

  gettimeofday(&curr_time,NULL);

  /* The daemon will wait for an acknowlegement from the client till
   * 350ms after which timeout will happen and the daemon will release
   * the wakelock
   */
  wait_time.tv_sec = curr_time.tv_sec;
  wait_time.tv_nsec = (curr_time.tv_usec + 1000 * WAIT_TIME_MS) * 1000;

  pthread_cond_timedwait(&ack_wait, &ack_mutex, &wait_time);
  if (ack != 0) {
    DEBUG_LOG("Ack timeout\n");
  }
  pthread_mutex_unlock(&ack_mutex);
  system("echo power_manager_daemon > /sys/power/wake_unlock");
}

void send_sys_suspend_event()
{
  power_state_t pwr_state;
  pwr_state.sys_state = SYS_SUSPEND;
  if (client_count > 0) {
    send_notification(pwr_state);
  }
  system("echo 1 > /sys/kernel/debug/eth/suspend_ipa_offload");
}

void send_sys_resume_event()
{
  int i;
  if (client_count > 0) {
    for (i = 0; i < client_count; i++)
    {
      if (client_fd[i] != -1) {
        power_state_t pwr_state;
        pwr_state.sys_state = SYS_RESUME;
        write(client_fd[i], &pwr_state, sizeof(power_state_t));
      }
    }
  }
}

void send_sys_shutdown_event()
{
  power_state_t pwr_state;
  pwr_state.sys_state = SYS_SHUTDOWN;
  if (client_count > 0) {
    send_notification(pwr_state);
  }
  system("shutdown -hP now");   /* Triggers system shutdown */
}

static void *wait_for_pwr_key_event(void *arg)
{
  int rc, fd, n, time, pwr_key_press = 0;
  struct input_event ev;
  struct timeval press, release;

  fd = open(PWR_KEY_DEV, O_RDONLY);
  if (fd < 0) {
    ERR("powerkey device open() failed : %s\n",strerror(errno));
    return NULL;
  }

  memset(&press, 0, sizeof(struct timeval));
  memset(&release, 0, sizeof(struct timeval));

  while (1) {
    n = read(fd, &ev, sizeof(struct input_event));
    if (n < sizeof(struct input_event)) {
      ERR("Read powerkey event failed : %s\n",strerror(errno));
      break;
    }

    if (ev.type == EV_KEY && ev.code == KEY_POWER) {
      if (ev.value == 1){
        memcpy(&press, &ev.time, sizeof(struct timeval));
        pwr_key_press = 1;
      } else {
        memcpy(&release, &ev.time, sizeof(struct timeval));
        if ((press.tv_sec >= 0) && (release.tv_sec >= 0) && (pwr_key_press == 1)) {
          pwr_key_press = 0;
          if (release.tv_usec > press.tv_usec) {
            release.tv_usec += 1000000;
            release.tv_sec--;
          }
          time = (int) (release.tv_sec - press.tv_sec) * 1000000 + release.tv_usec - press.tv_usec;

          if (time < SHUTDOWN_TIME_US) {
            if (suspended == 0) {
              DEBUG_LOG("Power key event detected : send suspend notification\n");
              send_sys_suspend_event();
              suspended = 1;
            }
            else {
              DEBUG_LOG("Power key event detected : send resume notification\n");
              send_sys_resume_event();
              suspended = 0;
            }
          } else {
            DEBUG_LOG("Power key event detected : send shutdown notification\n");
            send_sys_shutdown_event();
            break;
          }
        }
      }
    }
  }
  return NULL;
}

static void *wait_for_gpio_event(void *arg)
{
  int fd = -1;
  unsigned char value;
  char data_path[64];
  unsigned char prev_value = '0';

  snprintf(data_path,sizeof(data_path),"/sys/class/gpio/gpio%d/value", gpio);
  fd = open(data_path,O_RDONLY);

  while (read(fd, &value, sizeof(value)) > 0) {
    if (value != prev_value) {
      if (value == '0') {
        send_sys_suspend_event();
      }
      else if (value == '1') {
        send_sys_resume_event();
      }
      prev_value = value;
    }
    lseek(fd, 0, SEEK_SET);
  }
  return NULL;
}

void sys_suspend(CwFrame * pf, void* /*userData*/, int ifNo) {
  send_sys_suspend_event();
}

void sys_resume(CwFrame * pf, void* /*userData*/, int ifNo) {
  send_sys_resume_event();
}

void sys_shutdown(CwFrame * pf, void* /*userData*/, int ifNo) {
  send_sys_shutdown_event();
}

void register_for_can_frame_ids()
{
  FILE *fp;
  int fd;

  /* power_state.conf file has the different CAN frame IDs for suspend,
   * resume and shutdown. Register for those frame IDs to listen for
   * power state changes. The frame IDs can be modified by editing the
   * power_state.conf file.
   */
  fp = fopen(CFG_FILE_PATH,"r");
  if (fp == NULL)
  {
    ERR("config file open failed : %s", strerror(errno));
    return;
  }

  char inp[128];
  char *delim = ":";
  char *frame;
  char *id;

  while (fgets(inp,sizeof inp,fp) != NULL)
  {
    frame = strtok(inp, delim);
    id = strtok(NULL, delim);

    if (frame !=NULL && id != NULL)
    {
      /* SUSPEND frame */
      char *fr_id;
      if (strlen(id) > 2)
        fr_id = id+2;                                /* To ignore 0x in the frame ID read from config file */
      if (strcmp(frame, "SYS_SUSPEND_FRAME")== 0) {
        sscanf(fr_id, "%x", &frame_id.sys_suspend);  /* CAN frame IDs are interpreted as hex value */
        if (frame_id.sys_suspend) {
          pTheCh->registerListener(frame_id.sys_suspend, CwBase::MASK11, sys_suspend, 0, CwBase::IFACE_ANY);
          DEBUG_LOG("Registered for SYS_SUSPEND_FRAME id %x\n",frame_id.sys_suspend);
        }
      }
      /* RESUME frame */
      else if (strcmp(frame, "SYS_RESUME_FRAME")== 0) {
        sscanf(fr_id, "%x", &frame_id.sys_resume);
        if (frame_id.sys_resume) {
          pTheCh->registerListener(frame_id.sys_resume, CwBase::MASK11, sys_resume, 0, CwBase::IFACE_ANY);
          DEBUG_LOG("Registered for SYS_RESUME_FRAME id %x\n",frame_id.sys_resume);
        }
      }
      /*SHUTDOWN frame */
      else if (strcmp(frame, "SYS_SHUTDOWN_FRAME")== 0) {
        sscanf(fr_id, "%x", &frame_id.sys_shutdown);
        if (frame_id.sys_shutdown) {
          pTheCh->registerListener(frame_id.sys_shutdown, CwBase::MASK11, sys_shutdown, 0, CwBase::IFACE_ANY);
          DEBUG_LOG("Registered for SYS_SHUTDOWN_FRAME id %x\n",frame_id.sys_shutdown);
        }
      }
      else if (strcmp(frame, "GPIO") == 0) {
        sscanf(id, "%d", &gpio);  /* GPIO interpreted as decimal value*/
        strncpy(gpio_num, id, sizeof(gpio_num));
        if (gpio) {
          fd = open("/sys/class/gpio/export", O_WRONLY);
          if ((write(fd,gpio_num,strlen(gpio_num))) == -1) {
            return;
          }
          DEBUG_LOG("Requested for GPIO %d\n",gpio);
        }
      }
    }
  }
  fclose(fp);
}

int main ()
{
  int sock, len, epoll_fd, cli_fd, j;
  struct epoll_event ev;
  struct sockaddr_un sock_addr_un;
  struct sockaddr cli_addr;
  socklen_t cli_len = sizeof(cli_addr);
  pthread_t tid1, tid2;

  /* Create power_manager_daemon socket */
  sock = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sock == -1) {
    ERR("Socket creation failed : %s\n",strerror(errno));
    exit(1);
  }
  DEBUG_LOG("Socket creation successful\n");

  fcntl(sock, F_SETFL, (fcntl (sock, F_GETFL, 0) | O_NONBLOCK));

  sock_addr_un.sun_family = AF_UNIX;
  strncpy(sock_addr_un.sun_path, ADDRESS, sizeof(sock_addr_un.sun_path));
  len = sizeof(sock_addr_un);
  unlink(ADDRESS);
  if ((bind(sock, (struct sockaddr*) &sock_addr_un, len))== -1) {
    ERR("bind() failed : %s\n",strerror(errno));
    exit(1);
  }
  DEBUG_LOG("Socket bind successful\n");

  if ((listen (sock, MAX_CLIENTS_COUNT)) == -1) {
    ERR("listen() failed : %s",strerror(errno));
    exit(1);
  }
  DEBUG_LOG("Socket listen successful\n");

  epoll_fd = epoll_create(1);
  if (epoll_fd == -1)
  {
    ERR("epoll_create() failed : %s\n",strerror(errno));
    abort ();
  }
  DEBUG_LOG("epoll_create successful\n");

  ev.data.fd = sock;
  ev.events = EPOLLIN | EPOLLET;
  if ((epoll_ctl (epoll_fd, EPOLL_CTL_ADD, sock, &ev)) == -1)
  {
    ERR("epoll_ctl() failed : %s\n",strerror(errno));
    exit(1);
  }
  DEBUG_LOG("Added fd : %d to the epoll\n",sock);

  struct epoll_event *epoll_events = (struct epoll_event *) calloc(MAXEVENTS, sizeof(ev));

  client_fd = (int*) calloc (MAX_CLIENTS_COUNT, sizeof(int));
  pthread_mutex_init(&ack_mutex, NULL);
  pthread_cond_init (&ack_wait, NULL);

  pTheCh = CanWrapper::getInstance();
  if (pTheCh != NULL)
    register_for_can_frame_ids();

  pthread_create(&tid1, NULL, wait_for_gpio_event, NULL);
  pthread_create(&tid2, NULL, wait_for_pwr_key_event, NULL);

  while (1)
  {
    int n, i;
    n = epoll_wait (epoll_fd, epoll_events, MAXEVENTS, -1);
    for (i = 0; i < n; i++)
    {
      if ((epoll_events[i].events & EPOLLERR) || (epoll_events[i].events & EPOLLHUP)) {
        DEBUG_LOG("Received EPOLLERR or EPOLLHUP event from client with fd : %d\n",epoll_events[i].data.fd);
        for (j = 0; j < client_count; j++) {
          if (epoll_events[i].data.fd == client_fd[j]) {
            client_fd[j] = -1;
            break;
          }
        }
        close(epoll_events[i].data.fd);
        continue;
      }

      if (sock == epoll_events[i].data.fd)   /* New connetion request from client */
      {
        cli_fd = accept (sock, (struct sockaddr *) &cli_addr, &cli_len);
        if (cli_fd == -1)
          break;
        DEBUG_LOG("Accepted socket connection from client with fd : %d\n",cli_fd);
        client_fd[client_count++] = cli_fd;
        ev.data.fd = cli_fd;
        ev.events = EPOLLIN | EPOLLET;
        if ((epoll_ctl (epoll_fd, EPOLL_CTL_ADD, cli_fd, &ev)) ==  -1)
        {
          ERR("New connection epoll_ctl() failed : %s\n",strerror(errno));
          exit(1);
        }
        DEBUG_LOG("Added client with fd : %d to the epoll\n",cli_fd);
      }
      else
      {
        /* Acknowledgment from client */
        client_ack_t client_ack;
        read(epoll_events[i].data.fd, &client_ack, sizeof(client_ack_t));
        if (client_ack.ack == SUSPEND_ACK || client_ack.ack == SHUTDOWN_ACK)
        {
          DEBUG_LOG("Received ack from client with fd %d\n",epoll_events[i].data.fd);
          pthread_mutex_lock(&ack_mutex);
          for (j = 0; j < client_count; j++)
          {
            if (epoll_events[i].data.fd == client_fd[j]) {
              ack &= ~(1<<j);
              break;
            }
          }
          if (ack == 0) {   /* Acknowledgement is received from all clients */
            DEBUG_LOG("Received ack from all the clients\n");
            pthread_cond_signal(&ack_wait);
          }
          pthread_mutex_unlock(&ack_mutex);
        }
      }
    }
  }

  free(epoll_events);
  free(client_fd);
  close(sock);
  pthread_cond_destroy(&ack_wait);
  pthread_mutex_destroy(&ack_mutex);
  return 0;
}
