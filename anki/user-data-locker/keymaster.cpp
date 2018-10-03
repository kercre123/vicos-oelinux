/**
 * File: keymaster.cpp
 *
 * Author: Stuart Eichert
 * Created: 9/7/2018
 *
 * Description: Get/Use TrustZone crypto keys
 *
 * Copyright: Anki, Inc. 2018
 *
 * Portions of this code are inspired by the keymaster_qcom.cpp implementation
 * from the Android Open Source Project (AOSP) at hardware/qcom/keymaster/keymaster_qcom.cpp
 *
 * The below AOSP copyright and license notice is for those portions and not an indication of
 * a contribution to that project
 **/

/*
 *  Copyright (C) 2012 The Android Open Source Project
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you
 *  may not use this file except in compliance with the License.  You may
 *  obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *  implied.  See the License for the specific language governing
 *  permissions and limitations under the License.
 */

#include "keymaster.h"
#include "keymaster_common.h"
#include "keymaster_qcom.h"
#include "QSEEComAPI.h"
#include "log.h"

#include <stddef.h>
#include <linux/ioctl.h>
#include <linux/msm_ion.h>
#include <sys/mman.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/ioctl.h>

#include <dlfcn.h>
#include <unistd.h>

#include <algorithm>

struct qcom_keymaster_handle {
    struct QSEECom_handle *qseecom;
    void *libhandle;
    int (*QSEECom_start_app)(struct QSEECom_handle ** handle, char* path,
                          char* appname, uint32_t size);
    int (*QSEECom_shutdown_app)(struct QSEECom_handle **handle);
    int (*QSEECom_send_cmd)(struct QSEECom_handle* handle, void *cbuf,
                          uint32_t clen, void *rbuf, uint32_t rlen);
    int (*QSEECom_send_modified_cmd)(struct QSEECom_handle* handle, void *cbuf,
                          uint32_t clen, void *rbuf, uint32_t rlen,
                          struct QSEECom_ion_fd_info *ihandle);
    int (*QSEECom_set_bandwidth)(struct QSEECom_handle* handle, bool high);
};
typedef struct qcom_keymaster_handle qcom_keymaster_handle_t;

typedef struct qcom_km_key_blob qcom_km_key_blob_t;
typedef struct keymaster_gen_keypair_cmd keymaster_gen_keypair_cmd_t;
typedef struct keymaster_gen_keypair_resp keymaster_gen_keypair_resp_t;
typedef struct keymaster_sign_data_cmd keymaster_sign_data_cmd_t;
typedef struct keymaster_sign_data_resp keymaster_sign_data_resp_t;
typedef struct QSEECom_handle QSEECom_handle_t;
typedef struct QSEECom_ion_fd_data QSEECom_ion_fd_data_t;
typedef struct QSEECom_ion_fd_info QSEECom_ion_fd_info_t;

struct qcom_km_ion_info {
  int32_t ion_fd;
  int32_t ifd_data_fd;
  struct ion_handle_data ion_alloc_handle;
  unsigned char* ion_sbuffer;
  uint32_t sbuf_len;
};
typedef struct qcom_km_ion_info qcom_km_ion_info_t;

static void * malloc_zero(size_t size) {
  uint8_t* p = (uint8_t *) malloc(size);
  if (p) {
    memset(p, 0, size);
  }
  return p;
}


#ifndef O_DSYNC
#define O_DSYNC 010000
#endif
static int32_t qcom_km_ION_memalloc(qcom_km_ion_info_t *handle,
                                    uint32_t size)
{
  int32_t ret = 0;
  unsigned char *v_addr;
  struct ion_allocation_data ion_alloc_data;
  int32_t ion_fd;
  int32_t rc;
  struct ion_fd_data ifd_data;
  struct ion_handle_data handle_data;

  /* open ION device for memory management
   * O_DSYNC -> uncached memory
   */
  if(handle == NULL){
    return -1;
  }
  ion_fd  = open("/dev/ion", O_RDONLY | O_DSYNC);
  if (ion_fd < 0) {
    return -1;
  }
  handle->ion_sbuffer = NULL;
  handle->ifd_data_fd = 0;

  /* Size of allocation */
  ion_alloc_data.len = (size + 4095) & (~4095);

  /* 4K aligned */
  ion_alloc_data.align = 4096;

  /* memory is allocated from EBI heap */
  ion_alloc_data.heap_id_mask = ION_HEAP(ION_QSECOM_HEAP_ID);

  /* Set the memory to be uncached */
  ion_alloc_data.flags = 0;

  /* IOCTL call to ION for memory request */
  rc = ioctl(ion_fd, ION_IOC_ALLOC, &ion_alloc_data);
  if (rc) {
    ret = -1;
    goto alloc_fail;
  }

  if (ion_alloc_data.handle) {
    ifd_data.handle = ion_alloc_data.handle;
  } else {
    ret = -1;
    goto alloc_fail;
  }
  /* Call MAP ioctl to retrieve the ifd_data.fd file descriptor */
  rc = ioctl(ion_fd, ION_IOC_MAP, &ifd_data);
  if (rc) {
    ret = -1;
    goto ioctl_fail;
  }

  /* Make the ion mmap call */
  v_addr = (unsigned char *)mmap(NULL, ion_alloc_data.len,
                                 PROT_READ | PROT_WRITE,
                                 MAP_SHARED, ifd_data.fd, 0);
  if (v_addr == MAP_FAILED) {
    ret = -1;
    goto map_fail;
  }
  handle->ion_fd = ion_fd;
  handle->ifd_data_fd = ifd_data.fd;
  handle->ion_sbuffer = v_addr;
  handle->ion_alloc_handle.handle = ion_alloc_data.handle;
  handle->sbuf_len = size;
  return ret;

map_fail:
  if (handle->ion_sbuffer != NULL) {
    (void) munmap(handle->ion_sbuffer, ion_alloc_data.len);
  }

ioctl_fail:
  handle_data.handle = ion_alloc_data.handle;
  if (handle->ifd_data_fd) {
    close(handle->ifd_data_fd);
  }
  (void) ioctl(ion_fd, ION_IOC_FREE, &handle_data);


alloc_fail:
  if (ion_fd > 0) {
    close(ion_fd);
  }
  return ret;
}

/** @brief: Deallocate ION memory
 *
 *
 */
static int32_t qcom_km_ion_dealloc(qcom_km_ion_info_t *handle)
{
  struct ion_handle_data handle_data;
  int32_t ret = 0;

  /* Deallocate the memory for the listener */
  ret = munmap(handle->ion_sbuffer, (handle->sbuf_len + 4095) & (~4095));

  handle_data.handle = handle->ion_alloc_handle.handle;
  close(handle->ifd_data_fd);
  ret = ioctl(handle->ion_fd, ION_IOC_FREE, &handle_data);

  close(handle->ion_fd);
  return ret;
}

static int qcom_km_get_lib_sym(qcom_keymaster_handle_t* km_handle) {
  km_handle->libhandle = dlopen("libQseeComApi.so", RTLD_NOW);
  if (!km_handle->libhandle) {
    loge("Failed to dlopen libQseeComApi.so");
    return -1;
  }
  *(void **)(&km_handle->QSEECom_start_app) =
      dlsym(km_handle->libhandle,"QSEECom_start_app");
  if (km_handle->QSEECom_start_app == nullptr) {
    loge("Failed to get sym for QSEECom_start_app");
    dlclose(km_handle->libhandle);
    km_handle->libhandle  = nullptr;
    return -1;
  }
  *(void **)(&km_handle->QSEECom_shutdown_app) =
      dlsym(km_handle->libhandle,"QSEECom_shutdown_app");
  if (km_handle->QSEECom_shutdown_app == nullptr) {
    loge("Failed to get sym for QSEECom_shutdown_app");
    dlclose(km_handle->libhandle);
    km_handle->libhandle  = nullptr;
    return -1;
  }
  *(void **)(&km_handle->QSEECom_send_cmd) =
      dlsym(km_handle->libhandle,"QSEECom_send_cmd");
  if (km_handle->QSEECom_send_cmd == nullptr) {
    loge("Failed to get sym for QSEECom_send_cmd");
    dlclose(km_handle->libhandle);
    km_handle->libhandle  = nullptr;
    return -1;
  }
  *(void **)(&km_handle->QSEECom_send_modified_cmd) =
      dlsym(km_handle->libhandle,"QSEECom_send_modified_cmd");
  if (km_handle->QSEECom_send_modified_cmd == nullptr) {
    loge("Failed to get sym for QSEECom_send_modified_cmd");
    dlclose(km_handle->libhandle );
    km_handle->libhandle  = nullptr;
    return -1;
  }
  *(void **)(&km_handle->QSEECom_set_bandwidth) =
      dlsym(km_handle->libhandle,"QSEECom_set_bandwidth");
  if (km_handle->QSEECom_set_bandwidth == nullptr) {
    loge("Failed to get sym for QSEECom_set_bandwidth");
    dlclose(km_handle->libhandle );
    km_handle->libhandle  = nullptr;
    return -1;
  }

  return 0;
}

static int qcom_km_open(qcom_keymaster_handle_t** p_km_handle)
{
  *p_km_handle = nullptr;
  int ret = 0;
  qcom_keymaster_handle_t* km_handle;

  km_handle = (qcom_keymaster_handle_t *)malloc_zero(sizeof(*km_handle));
  if (km_handle == nullptr) {
    loge("Failed to allocate memory for qcom_keymaster_handle_t");
    return -1;
  }

  km_handle->qseecom = nullptr;
  km_handle->libhandle = nullptr;
  ret = qcom_km_get_lib_sym(km_handle);
  if (ret) {
    loge("Failed to get symbols for libQseeComApi");
    free(km_handle); km_handle = nullptr;
    return -1;
  }

  char path[32];
  (void) strcpy(path, "/firmware/image");
  char appname[16];
  (void) strcpy(appname, "keymaste");
  ret = (*km_handle->QSEECom_start_app)((struct QSEECom_handle **)&km_handle->qseecom, path,
                                        appname, 4096*2);
  if (ret) {
    (void) strcpy(appname, "keymaster");
    ret = (*km_handle->QSEECom_start_app)((struct QSEECom_handle **)&km_handle->qseecom, path,
                                          appname, 4096*2);
  }
  if (ret) {
    loge("Failed to start KeyMaster TrustLet ret = %d errno = %d (%s)",
         ret, errno, strerror(errno));
    free(km_handle); km_handle = nullptr;
    return -1;
  }
  *p_km_handle = km_handle;
  return 0;
}


KeyMaster::KeyMaster() {
  keymaster_handle = nullptr;
}

KeyMaster::~KeyMaster() {
  qcom_keymaster_handle_t* km_handle = (qcom_keymaster_handle_t *)keymaster_handle;
  if (km_handle) {
    if (km_handle->qseecom) {
      (*km_handle->QSEECom_shutdown_app)(&km_handle->qseecom);
    }
    free(km_handle) ; km_handle = nullptr;
  }
  keymaster_handle = nullptr;
}

int KeyMaster::Initialize() {
  if (keymaster_handle) {
    return 0;
  }
  qcom_keymaster_handle_t* km_handle;
  int ret = qcom_km_open(&km_handle);
  if (!ret) {
    keymaster_handle = km_handle;
  }
  return ret;
}

int KeyMaster::GenerateKeyBlob(std::vector<uint8_t>& keyBlob) {
  keyBlob.clear();
  int ret = Initialize();
  if (ret) {
    return ret;
  }
  qcom_keymaster_handle_t* km_handle = (qcom_keymaster_handle_t *)keymaster_handle;
  keymaster_gen_keypair_cmd_t* send_cmd = NULL;
  keymaster_gen_keypair_resp_t* resp = NULL;
  QSEECom_handle_t* handle = NULL;

  handle = km_handle->qseecom;
  send_cmd = (keymaster_gen_keypair_cmd_t *)handle->ion_sbuffer;
  resp = (keymaster_gen_keypair_resp_t *)(handle->ion_sbuffer + QSEECOM_ALIGN(sizeof(*send_cmd)));
  send_cmd->cmd_id = KEYMASTER_GENERATE_KEYPAIR;
  send_cmd->key_type = TYPE_RSA;
  send_cmd->rsa_params.modulus_size = 1024;
  send_cmd->rsa_params.public_exponent = 3;
  resp->status = KEYMASTER_FAILURE;
  resp->key_blob_len = sizeof(qcom_km_key_blob_t);

  ret = (*km_handle->QSEECom_set_bandwidth)(handle, true);
  if (ret < 0) {
    loge("GenerateKeyBlob : Failed to enable clks. ret = %d", ret);
    return -1;
  }

  ret = (*km_handle->QSEECom_send_cmd)(handle,
                                       send_cmd, QSEECOM_ALIGN(sizeof(*send_cmd)),
                                       resp, QSEECOM_ALIGN(sizeof(*resp)));
  if ((*km_handle->QSEECom_set_bandwidth)(handle, false)) {
    logw("GenerateKeyBlob: Failed to disable clks");
  }

  if ( (ret < 0) || (resp->status < 0) ) {
    loge("GenerateKeyBlob: Failed with resp->status = %d ret = %d", resp->status, ret);
    return -1;
  }

  keyBlob.reserve(resp->key_blob_len);
  uint8_t* blob = (uint8_t *)(&resp->key_blob);
  std::copy(blob, blob + resp->key_blob_len, std::back_inserter(keyBlob));

  return 0;
}

int KeyMaster::SignData(const std::vector<uint8_t>& keyBlob,
                        const std::vector<uint8_t>& data,
                        std::vector<uint8_t>& signature) {
  signature.clear();
  int ret = Initialize();
  if (ret) {
    return ret;
  }
  qcom_keymaster_handle_t* km_handle = (qcom_keymaster_handle_t *)keymaster_handle;
  QSEECom_handle_t* handle = (QSEECom_handle_t *)(km_handle->qseecom);
  QSEECom_ion_fd_info_t ion_fd_info = {0};
  qcom_km_ion_info_t ihandle;
  keymaster_sign_data_cmd_t* sign_data_cmd = nullptr;
  keymaster_sign_data_resp_t* resp = nullptr;

  ihandle.ion_fd = 0;
  ihandle.ion_alloc_handle.handle = 0;
  if (qcom_km_ION_memalloc(&ihandle, data.size()) < 0) {
    return -1;
  }
  memset(&ion_fd_info, 0, sizeof(ion_fd_info));

  ion_fd_info.data[0].fd = ihandle.ifd_data_fd;
  ion_fd_info.data[0].cmd_buf_offset = sizeof(enum keymaster_cmd_t)
      + sizeof(qcom_km_key_blob_t) + sizeof(keymaster_rsa_sign_params_t);

  sign_data_cmd = (keymaster_sign_data_cmd_t *)handle->ion_sbuffer;
  resp = (keymaster_sign_data_resp_t *)(handle->ion_sbuffer + QSEECOM_ALIGN(sizeof(*sign_data_cmd)));

  sign_data_cmd->cmd_id = KEYMASTER_SIGN_DATA;
  sign_data_cmd->sign_param.digest_type = DIGEST_NONE;
  sign_data_cmd->sign_param.padding_type = PADDING_NONE;
  memcpy((unsigned char *)(&sign_data_cmd->key_blob), keyBlob.data(), keyBlob.size());
  memcpy((unsigned char *)ihandle.ion_sbuffer, data.data(), data.size());
  sign_data_cmd->data = (uint32_t) ihandle.ion_sbuffer;
  sign_data_cmd->dlen = data.size();

  resp->status = KEYMASTER_FAILURE;
  resp->sig_len = KM_KEY_SIZE_MAX;

  //Sending the command
  ret = (*km_handle->QSEECom_set_bandwidth)(handle, true);
  if (ret < 0) {
    loge("SignData: Failed to enable clks. ret = %d", ret);
    qcom_km_ion_dealloc(&ihandle);
    return -errno;
  }

  ret = (*km_handle->QSEECom_send_modified_cmd)(handle,
                                                sign_data_cmd,
                                                QSEECOM_ALIGN(sizeof(*sign_data_cmd)),
                                                resp,
                                                QSEECOM_ALIGN(sizeof(*resp)),
                                                &ion_fd_info);

  if ((*km_handle->QSEECom_set_bandwidth)(handle, false)) {
    logw("SignData: Failed to disable clks");
  }

  if ( (ret < 0) || (resp->status < 0 )) {
    loge("SignData: command failed resp->status = %d ret = %d", resp->status, ret);
    qcom_km_ion_dealloc(&ihandle);
    return -1;
  }
  uint8_t* signed_data = (uint8_t *) (&resp->signed_data);
  signature.reserve(resp->sig_len);
  std::copy(signed_data, signed_data + resp->sig_len, std::back_inserter(signature));

  qcom_km_ion_dealloc(&ihandle);
  return 0;
}
