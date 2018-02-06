/**
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <QSEEComAPI.h>

typedef uint32_t (*ta_get_size_t)(void);
typedef int (*ta_pre_load_t)(char const * path);
typedef int (*ta_post_load_t)(void);
typedef int (*ta_send_cmd_t)(void * handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen);
typedef int (*ta_send_modified_cmd_t)(void * handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen, struct QSEECom_ion_fd_info *info);


#define DECLARE_GET_SIZE_FUNC(name) \
  uint32_t (name)(void)
#define DECLARE_PRE_LOAD_FUNC(name) \
  int (name)(char const * path)
#define DECLARE_POST_LOAD_FUNC(name) \
  int (name)(void)
#define DECLARE_SEND_CMD_FUNC(name) \
  int (name)(void * handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen)
#define DECLARE_SEND_MODIFIED_CMD_FUNC(name) \
  int (name)(void * handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen, struct QSEECom_ion_fd_info *info)


typedef struct {
  char const * const name;
  ta_get_size_t get_size;
  ta_pre_load_t pre_load;
  ta_post_load_t post_load;
  ta_send_cmd_t send_cmd;
  ta_send_modified_cmd_t send_modified_cmd;
} ta_interface_t;

extern ta_interface_t const supportedTA[];
extern size_t const supportedTALen;
