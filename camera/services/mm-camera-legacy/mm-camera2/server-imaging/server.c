/* server.c
 *
 * This file contains the server implementation. All commands coming
 * from the HAL arrive here first.
 *
 * Copyright (c) 2012-2013 Qualcomm Technologies, Inc. All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#include "server_process.h"

#include "camera_dbg.h"
#include <sys/sysinfo.h>
#include <media/msm_cam_sensor.h>

#if 0
#undef CDBG
#define CDBG ALOGE
#endif

/** read_fd_type:
 *    defines server selected FD types
 **/
typedef enum _read_fd_type {
  RD_FD_HAL,
  RD_DS_FD_HAL,
  RD_PIPE_FD_MCT,
  RD_FD_NONE
} read_fd_type;

/** read_fd_info_t:
 *    @type    -- either domain socket fd or mct fd
 *    @session -- session index
 *    @fd      --
 *      in case of domain socket fd: fd[0]
 *      in case of mct pipe fd: fd[0] - server read fd
 *                              fd[1] - mct write fd
 **/
typedef struct _read_fd_info {
  read_fd_type type;
  unsigned int session;
  int fd[2];
} read_fd_info_t;

typedef struct _server_select_fds {
  fd_set fds;
  int select_fd;
} server_select_fds_t;

/** server_find_listen_fd:
 *
 *  Return: TRUE if fd is set
 **/
static boolean server_check_listen_fd(void *data1, void *data2)
{
  return FD_ISSET(((read_fd_info_t *)data1)->fd[0], (fd_set *)data2);
}

/** server_find_listen_fd:
 *    @data1:
 *    @data2:
 *
 * return TRUE if the two FDs match.
 **/
static boolean server_find_listen_fd(void *data1, void *data2)
{
  read_fd_info_t *fd_info_match = (read_fd_info_t *)data2;
  read_fd_info_t *fd_info       = (read_fd_info_t *)data1;

  return ((fd_info_match->type == fd_info->type) &&
          ((fd_info_match->session == fd_info->session)));
}

/** server_reset_select_fd:
 *    @data:
 *    @user_data:
 *
 **/
static boolean server_reset_select_fd(void *data, void *user_data)
{
  read_fd_info_t      *fd_info  = (read_fd_info_t *)data;
  server_select_fds_t *selected = (server_select_fds_t *)user_data;

  FD_SET(fd_info->fd[0], &(selected->fds));
  selected->select_fd = MTYPE_MAX(selected->select_fd, fd_info->fd[0]);

  return TRUE;
}

/** get_server_node_name:
 *    @node_name
 *
 *
 **/
static boolean get_server_node_name(char *node_name)
{

  int num_media_devices = 0;
  char dev_name[MAX_DEV_NAME_SIZE];
  int dev_fd = 0;
  int rc = 0;
  struct media_device_info mdev_info;
  int num_entities = 0;

  while (1) {
    snprintf(dev_name, sizeof(dev_name), "/dev/media%d", num_media_devices++);
    dev_fd = open(dev_name, O_RDWR | O_NONBLOCK);
    if (dev_fd < 0) {
      /* Done enumerating media devices */
      break;
    } else {
    }

    memset(&mdev_info, 0, sizeof(struct media_device_info));
    rc = ioctl(dev_fd, MEDIA_IOC_DEVICE_INFO, &mdev_info);
    if (rc < 0) {
      close(dev_fd);
      break;
    }

    if (strncmp(mdev_info.model, "msm_config", sizeof(mdev_info.model)) != 0) {
      close(dev_fd);
      continue;
    }

    num_entities = 1;
    while (1) {
      struct media_entity_desc entity;
      memset(&entity, 0, sizeof(entity));
      entity.id = num_entities++;
      rc = ioctl(dev_fd, MEDIA_IOC_ENUM_ENTITIES, &entity);
      if (rc < 0) {
        rc = 0;
        break;
      }

      if (entity.type == MEDIA_ENT_T_DEVNODE_V4L &&
          entity.group_id == QCAMERA_VNODE_GROUP_ID) {
        /* found the video device */
        strncpy(node_name, entity.name, MAX_DEV_NAME_SIZE);
      } else {
        close(dev_fd);
        return TRUE;
      }
    } /* enumerate entites */
    close(dev_fd);
  } /* enumerate media devices */

  return FALSE;
}

/** get_probe_done_node_name:
 *    @node_name
 *
 *
 **/
static boolean get_probe_done_node_name(char *node_name)
{

  int num_media_devices = 0;
  char dev_name[MAX_DEV_NAME_SIZE];
  int dev_fd = 0;
  int rc = 0;
  struct media_device_info mdev_info;
  int num_entities = 0;

  while (1) {
    snprintf(dev_name, sizeof(dev_name), "/dev/media%d", num_media_devices++);
    dev_fd = open(dev_name, O_RDWR | O_NONBLOCK);
    if (dev_fd < 0) {
      /* Done enumerating media devices */
      CDBG("Done enumerating media devices: 1\n");
      break;
    }

    memset(&mdev_info, 0, sizeof(struct media_device_info));
    rc = ioctl(dev_fd, MEDIA_IOC_DEVICE_INFO, &mdev_info);
    if (rc < 0) {
      CDBG("Done enumerating media devices: 2\n");
      close(dev_fd);
      break;
    }

    if (strncmp(mdev_info.model, "msm_config", sizeof(mdev_info.model)) != 0) {
      close(dev_fd);
      continue;
    }

    num_entities = 1;
    while (1) {
      struct media_entity_desc entity;
      memset(&entity, 0, sizeof(entity));
      entity.id = num_entities++;
      rc = ioctl(dev_fd, MEDIA_IOC_ENUM_ENTITIES, &entity);
      if (rc < 0) {
        CDBG("Done enumerating media entities\n");
        rc = 0;
        break;
      }

      CDBG("entity name %s type %d group id %d",
        entity.name, entity.type, entity.group_id);
      if (entity.type == MEDIA_ENT_T_V4L2_SUBDEV &&
          entity.group_id == MSM_CAMERA_SUBDEV_SENSOR_INIT) {
        /* found the video device */
        strlcpy(node_name, entity.name, MAX_DEV_NAME_SIZE);
    CDBG("node_name = %s\n", node_name);
       break;
      }
    } /* enumerate entites */
    close(dev_fd);
  } /* enumerate media devices */

  return TRUE;
}



/** server_t_handler:
 *    @data: signal value
 *
 * mct stuck thread
 **/
void server_t_handler(union sigval val)
{
  CDBG_ERROR("%s:Error Backend stuck during NEW/DEL session\n",__func__);
  raise(SIGABRT);
  sleep(1);
}

static void handler(int signum)
{
  CDBG_ERROR("Sigsegv\n");
  void* callstack[128];
  int i, frames = backtrace(callstack, 128);
  char** strs = backtrace_symbols(callstack, frames);
  for (i = 0; i < frames; ++i) {
    CDBG_ERROR("%s\n", strs[i]);
  }
  free(strs);
}

/** main:
 *
 **/
int main(int argc, char *argv[])
{
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);

  struct v4l2_event_subscription subscribe;
  struct v4l2_event              event;

  mct_list_t                *listen_fd_list = NULL;
  read_fd_info_t            *hal_fd = NULL, *hal_ds_fd = NULL, *mct_fds = NULL;
  server_select_fds_t        select_fds;
  serv_proc_ret_t            proc_ret;
  struct msm_v4l2_event_data *ret_data;
  char *serv_hal_node_name;
  char dev_name[MAX_DEV_NAME_SIZE];
  char probe_done_node_name[MAX_DEV_NAME_SIZE];
  char probe_done_dev_name[MAX_DEV_NAME_SIZE];
  int probe_done_fd;
  struct sensor_init_cfg_data cfg;
  int  ret, i, j;
  mode_t old_mode;
  struct sysinfo info;
  uint32_t result;

  int serv_t_ret = 0;
  struct sigevent serv_t_sig;
  timer_t serv_timerid;
  pthread_attr_t serv_t_attr;
  struct itimerspec serv_in, serv_out;
  int32_t enabled_savemem = 0;
  char savemem[92];

  old_mode = umask(S_IRWXO);

  CDBG_ERROR("CAMERA_DAEMON: [legacy] start of camera Daemon\n");
  /* 1. find server node name and open the node */
  serv_hal_node_name = malloc((size_t)MAX_DEV_NAME_SIZE);
  if (!serv_hal_node_name)
    goto initial_fail;

  if (get_server_node_name(serv_hal_node_name) == FALSE)
    goto bad_node_fd;

  hal_fd = malloc(sizeof(read_fd_info_t));
  if (hal_fd == NULL)
    goto bad_node_fd;

  snprintf(dev_name, sizeof(dev_name), "/dev/%s", serv_hal_node_name);
  hal_fd->fd[0] = open(dev_name, O_RDWR | O_NONBLOCK);
  if (hal_fd->fd[0] < 0)
    goto open_hal_fail;

  hal_fd->type = RD_FD_HAL;

  if (!(listen_fd_list = mct_list_append(listen_fd_list, hal_fd, NULL, NULL)))
    goto list_append_fail;

  property_get("cameradaemon.SaveMemAtBoot", savemem, "0");
  enabled_savemem = atoi(savemem);

  CDBG_ERROR("CAMERA_DAEMON: start all modules init\n");
  /* 2. after open node, initialize modules */
  if(server_process_module_sensor_init() == FALSE)
    goto module_init_fail;
  CDBG("CAMERA_DAEMON:End of all modules init\n");

  result = sysinfo(&info);

  if (enabled_savemem != 1) {
    if (server_process_module_init() == FALSE)
      goto module_init_fail;
  }

  /* Subcribe V4L2 event */
  memset(&subscribe, 0, sizeof(struct v4l2_event_subscription));
  subscribe.type = MSM_CAMERA_V4L2_EVENT_TYPE;
  for (i = MSM_CAMERA_EVENT_MIN + 1; i < MSM_CAMERA_EVENT_MAX; i++) {
    subscribe.id = i;
    if (ioctl(hal_fd->fd[0], VIDIOC_SUBSCRIBE_EVENT, &subscribe) < 0)
      goto subscribe_failed;
  }

  select_fds.select_fd = hal_fd->fd[0];

  /* create a timer */
  serv_t_sig.sigev_notify = SIGEV_THREAD;
  serv_t_sig.sigev_notify_function = server_t_handler;
  serv_t_sig.sigev_value.sival_ptr = NULL;
  pthread_attr_init(&serv_t_attr);
  serv_t_sig.sigev_notify_attributes = &serv_t_attr;

  serv_t_ret = timer_create(CLOCK_REALTIME, &serv_t_sig, &serv_timerid);
  if (!serv_t_ret) {
    serv_in.it_value.tv_sec = 0;
    serv_in.it_value.tv_nsec = 0;
    serv_in.it_interval.tv_sec = serv_in.it_value.tv_sec;
    serv_in.it_interval.tv_nsec = serv_in.it_value.tv_nsec;
    timer_settime(serv_timerid, 0, &serv_in, &serv_out);
  }

  if (get_probe_done_node_name(probe_done_node_name) == FALSE)
      goto subscribe_failed;

  snprintf(probe_done_dev_name, sizeof(probe_done_dev_name), "/dev/%s", probe_done_node_name);
  probe_done_fd = open(probe_done_dev_name, O_RDWR | O_NONBLOCK);

  if (probe_done_fd < 0)
    goto subscribe_failed;

  cfg.cfgtype = CFG_SINIT_PROBE_DONE;
  if (ioctl(probe_done_fd, VIDIOC_MSM_SENSOR_INIT_CFG, &cfg) < 0) {
    CDBG_ERROR("%s: failed\n", __func__);
    ret = FALSE;
  }
  close(probe_done_fd);


  CDBG("CAMERA_DAEMON:waiting for camera to open\n");
  do {

    FD_ZERO(&(select_fds.fds));
    mct_list_traverse(listen_fd_list, server_reset_select_fd, &select_fds);

    /* no timeout */
    ret = select(select_fds.select_fd + 1, &(select_fds.fds), NULL, NULL, NULL);

    if (ret > 0) {

      mct_list_t     *find_list;
      read_fd_info_t *fd_info;

      find_list = mct_list_find_custom(listen_fd_list, &(select_fds.fds),
        server_check_listen_fd);
      if (!find_list)
        continue;

      fd_info = (read_fd_info_t *)find_list->data;

      switch (fd_info->type) {
      case RD_FD_HAL: {

        if (ioctl(fd_info->fd[0], VIDIOC_DQEVENT, &event) < 0)
          continue;

        /* server process HAL event:
         *
         *   1. if it returns success, it means the event message has been
         *      posted to MCT, don't need to send CMD ACK back to kernel
         *      immediately, because MCT will notify us after process;
         *
         *   2. if it returns failure, it means the event message was not
         *      posted to MCT successfully, hence we need to send CMD ACK back
         *      to kernel immediately so that HAL thread which sends this
         *      event can be blocked.
         */
        if (event.id == MSM_CAMERA_NEW_SESSION) {
          serv_in.it_value.tv_sec = 5;
          serv_in.it_value.tv_nsec = 0;
          serv_in.it_interval.tv_sec = serv_in.it_value.tv_sec;
          serv_in.it_interval.tv_nsec = serv_in.it_value.tv_nsec;
          timer_settime(serv_timerid, 0, &serv_in, &serv_out);
        } else if (event.id == MSM_CAMERA_DEL_SESSION) {
          serv_in.it_value.tv_sec = 5;
          serv_in.it_value.tv_nsec = 0;
          serv_in.it_interval.tv_sec = serv_in.it_value.tv_sec;
          serv_in.it_interval.tv_nsec = serv_in.it_value.tv_nsec;
          timer_settime(serv_timerid, 0, &serv_in, &serv_out);
	}
        proc_ret = server_process_hal_event(&event);
        serv_in.it_value.tv_sec = 0;
        serv_in.it_value.tv_nsec = 0;
        serv_in.it_interval.tv_sec = serv_in.it_value.tv_sec;
        serv_in.it_interval.tv_nsec = serv_in.it_value.tv_nsec;
        timer_settime(serv_timerid, 0, &serv_in, &serv_out);
      }
        break;

      case RD_DS_FD_HAL:
        /* server process message sent by HAL through Domain Socket */
        proc_ret = server_process_hal_ds_packet(fd_info->fd[0],
          fd_info->session);
        break;

      case RD_PIPE_FD_MCT:
        /* server process message sent by media controller
         * through pipe: */
        proc_ret = server_process_mct_msg(fd_info->fd[0],
          fd_info->session);
        break;

      default:
        continue;
      } /* switch (fd_info->type) */

      switch (proc_ret.result) {
      case RESULT_NEW_SESSION: {
       struct msm_v4l2_event_data *ret_data =
        (struct msm_v4l2_event_data *)proc_ret.ret_to_hal.ret_event.u.data;
       if( ret_data->status == MSM_CAMERA_CMD_SUCESS) {
        hal_ds_fd = malloc(sizeof(read_fd_info_t));
        if (!hal_ds_fd) {
          /* Shouldn't end directly, need to shutdown MCT thread */
          goto server_proc_new_session_error;
        } else {
          hal_ds_fd->session = proc_ret.new_session_info.session_idx;
          hal_ds_fd->fd[0]   = proc_ret.new_session_info.hal_ds_fd;
          hal_ds_fd->type    = RD_DS_FD_HAL;
        }

        mct_fds = malloc(sizeof(read_fd_info_t));
        if (!mct_fds) {
          free(hal_ds_fd);
          goto server_proc_new_session_error;
        } else {
          mct_fds->session = proc_ret.new_session_info.session_idx;
          mct_fds->fd[0]   = proc_ret.new_session_info.mct_msg_rd_fd;
          mct_fds->fd[1]   = proc_ret.new_session_info.mct_msg_wt_fd;
          mct_fds->type    = RD_PIPE_FD_MCT;
        }

        if (!(listen_fd_list = mct_list_append(listen_fd_list,
            hal_ds_fd, NULL, NULL))) {
          free(hal_ds_fd);
          free(mct_fds);
          goto server_proc_new_session_error;
        }

        if (!(listen_fd_list = mct_list_append
             (listen_fd_list, mct_fds, NULL, NULL))) {
          free(hal_ds_fd);
          free(mct_fds);
          goto server_proc_new_session_error;
        }
        } else {
          CDBG_ERROR("%s: New session [%d] creation failed with error",
            __func__, ret_data->session_id);
        }
        goto check_proc_ret;
      } /* RESULT_NEW_SESSION */
        break;

      case RESULT_DEL_SESSION: {
        mct_list_t     *find_list;
        read_fd_info_t fd_info_match;
        read_fd_info_t *ds_fd_info = NULL, *mct_fd_info = NULL;
        struct msm_v4l2_event_data *event_data = (struct msm_v4l2_event_data *)
          &proc_ret.ret_to_hal.ret_event.u.data[0];

        /* this is for Domain Socket FD */
        fd_info_match.type    = RD_DS_FD_HAL;
        fd_info_match.session = event_data->session_id;

        find_list = mct_list_find_custom(listen_fd_list, &fd_info_match,
          server_find_listen_fd);
        if (find_list) {
          ds_fd_info = (read_fd_info_t *)find_list->data;
          listen_fd_list = mct_list_remove(listen_fd_list, ds_fd_info);
          FD_CLR(ds_fd_info->fd[0], &select_fds.fds);
          close(ds_fd_info->fd[0]);
          free(ds_fd_info);
        }

        /* this is for MCT FD */
        fd_info_match.type    = RD_PIPE_FD_MCT;

        find_list = mct_list_find_custom(listen_fd_list, &fd_info_match,
          server_find_listen_fd);
        if (find_list) {
          mct_fd_info = (read_fd_info_t *)find_list->data;
          listen_fd_list = mct_list_remove(listen_fd_list, mct_fd_info);
          FD_CLR(mct_fd_info->fd[0], &select_fds.fds);
          close(mct_fd_info->fd[0]);
          close(mct_fd_info->fd[1]);
          free(mct_fd_info);
        }
      } /* case RESULT_DEL_SESSION */
         goto check_proc_ret;
        break;

      case RESULT_FAILURE:
        goto server_proc_error;
        break;

      case RESULT_SUCCESS:
        goto check_proc_ret;
        break;

      default:
        break;
      } /* switch (proc_ret.result) */

server_proc_new_session_error:
      event.id = MSM_CAMERA_DEL_SESSION;
      server_process_hal_event(&event);

server_proc_error:
      proc_ret.ret_to_hal.ret = TRUE;

check_proc_ret:
      if (proc_ret.ret_to_hal.ret == TRUE) {
        switch (proc_ret.ret_to_hal.ret_type) {
        /* @MSM_CAM_V4L2_IOCTL_CMD_ACK is Ack-knowledge to HAL's
         *   control command, which has command processing status.
         */
        case SERV_RET_TO_HAL_CMDACK:
          ioctl(hal_fd->fd[0], MSM_CAM_V4L2_IOCTL_CMD_ACK,
            (struct msm_v4l2_event_data *)&(proc_ret.ret_to_hal.ret_event.u.data));
          break;

        /* @MSM_CAM_V4L2_IOCTL_NOTIFY is MCT originated event such
         *   as meta data, SOF etc. Normally it comes
         *  1. domain socket buffer mapping process;
         *  2. from MCT.
         */
        case SERV_RET_TO_HAL_NOTIFY:
          ioctl(hal_fd->fd[0], MSM_CAM_V4L2_IOCTL_NOTIFY,
            &(proc_ret.ret_to_hal.ret_event.u.data));
          break;

        /* @MMSM_CAM_V4L2_IOCTL_NOTIFY_META is Meta data notification
         *   sent back to HAL during streaming. It is generated by
         *   BUS message.
         */
        case SERV_RET_TO_KERNEL_NOTIFY_POSSIBLE_FREEZE:
          ioctl(hal_fd->fd[0], MSM_CAM_V4L2_IOCTL_NOTIFY_DEBUG,
            &(proc_ret.ret_to_hal.ret_event.u.data));
          break;
        case SERV_RET_TO_HAL_NOTIFY_ERROR:
          ioctl(hal_fd->fd[0], MSM_CAM_V4L2_IOCTL_NOTIFY_ERROR,
            &(proc_ret.ret_to_hal.ret_event.u.data));
          break;

        default:
          break;
        }
      }
    } else {
      /* select failed. it cannot time out.*/
      /* TO DO: HANDLE ERROR */
    }
  } while (1);

server_down:
subscribe_failed:
  memset(&subscribe, 0, sizeof(struct v4l2_event_subscription));
  subscribe.type = MSM_CAMERA_V4L2_EVENT_TYPE;
  for (j = MSM_CAMERA_EVENT_MIN + 1; j < i; j++)  {
    subscribe.id = j;
    ioctl(hal_fd->fd[0], VIDIOC_UNSUBSCRIBE_EVENT, &subscribe);
  }
module_init_fail:
  mct_list_remove(listen_fd_list, hal_fd);
list_append_fail:
  close(hal_fd->fd[0]);
open_hal_fail:
  free(hal_fd);
bad_node_fd:
  CDBG_ERROR("Bad fd freo hal.\n");
  free(serv_hal_node_name);
initial_fail:
  return FALSE;
}
