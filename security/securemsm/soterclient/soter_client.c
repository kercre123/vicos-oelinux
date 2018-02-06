/********************************************************************
---------------------------------------------------------------------
 Copyright (c) 2015-2016 Qualcomm Technologies, Inc.
 All Rights Reserved.
 Confidential and Proprietary - Qualcomm Technologies, Inc.
----------------------------------------------------------------------
SOTER Client app.
*********************************************************************/
/*===========================================================================

                      EDIT HISTORY FOR FILE

when       who     what, where, why
--------   ---     ----------------------------------------------------------
11/30/15   dy      Add soter client to support ATTK provision

===========================================================================*/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/eventfd.h>
#include <errno.h>
#include <linux/msm_ion.h>
#include <utils/Log.h>
#include "common_log.h"
#include <sys/mman.h>
#include <getopt.h>
#include "comdef.h"
#include <dlfcn.h>

#include <hardware/keymaster1.h>
#include <hardware/keymaster_defs.h>

/** adb log */
#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "SOTER_CLIENT: "
#ifdef LOG_NDDEBUG
#undef LOG_NDDEBUG
#endif
#define LOG_NDDEBUG 0 //Define to enable LOGD
#ifdef LOG_NDEBUG
#undef LOG_NDEBUG
#endif
#define LOG_NDEBUG  0 //Define to enable LOGV

#define PUBKEY_BUFF_SIZE               1024
#define DEVICE_ID_SIZE                 16
#define ATTK_COPY_NUM_DEFAULT          3
#define CMD_GENERATE_ATTK_KEY_PAIR     1
#define CMD_VERIFY_ATTK_KEY_PAIR       2
#define CMD_EXPORT_ATTK_PUBLIC_KEY     3
#define CMD_GET_DEVICE_ID              4

static void qsc_usage(void)
{

    printf("*************************************************************\n");
    printf("************           SOTER CLIENT              ************\n");
    printf("*************************************************************\n");
    printf("\n"
        "Usage: soter_client [CMD]\n"
        "CMD can be:\n"
        " \t\t 1: Generate ATTK key pair\n"
        " \t\t 2: Verify ATTK key pair\n"
        " \t\t 3: Export ATTK public key \n"
        " \t\t 4: Get device ID \n"
        "---------------------------------------------------------\n\n\n");
}

static int keymaster_device_initialize(keymaster1_device_t** dev) {
    const hw_module_t* mod;
    int ret = KM_ERROR_UNKNOWN_ERROR;

    ret = hw_get_module_by_class(KEYSTORE_HARDWARE_MODULE_ID, NULL, &mod);
    if (ret) {
        ALOGI("Could not find any keystore module");
        return KM_ERROR_UNKNOWN_ERROR;
    }

    if (mod->module_api_version != KEYMASTER_MODULE_API_VERSION_1_0) {
        ALOGI("Keystore module api version is not 1.0");
        return KM_ERROR_VERSION_MISMATCH;
    }

    ret = keymaster1_open(mod, dev);
    if (ret) {
        LOGE("could not open keymaster device in %s (%s)",
            KEYSTORE_HARDWARE_MODULE_ID, strerror(-ret));
    }

    return ret;
}

int get_device_id(void)
{
    keymaster1_device_t* dev = 0;
    uint8_t device_id[DEVICE_ID_SIZE];
    size_t device_id_length = DEVICE_ID_SIZE;
    int ret = KM_ERROR_UNKNOWN_ERROR;
    uint32_t i;

    if (keymaster_device_initialize(&dev)) {
        LOGE("keystore keymaster could not be initialized; exiting");
        return KM_ERROR_UNKNOWN_ERROR;
    }

    //call apis
    if (!dev || dev->get_device_id == NULL)
    {
        LOGE("get_device_id not supported");
        goto ret_handle;
    }

    ret = dev->get_device_id(dev, device_id, device_id_length);
    if (ret != KM_ERROR_OK) {
        LOGE("Failed to export attk, error %d", ret);
        printf("fail %d\n", ret);
    } else {
        for(i=0; i<device_id_length; i++)
            printf("%02x",device_id[i]);
        printf("\n");
    }

ret_handle:
    if(dev)
        keymaster1_close(dev);
    return ret;
}
int verify_attk_key_pair(void)
{
    keymaster1_device_t* dev = 0;
    int ret = KM_ERROR_UNKNOWN_ERROR;

    if (keymaster_device_initialize(&dev)) {
        LOGE("keystore keymaster could not be initialized; exiting");
        return KM_ERROR_UNKNOWN_ERROR;
    }

    //call apis
    if (!dev || dev->verify_attk_key_pair == NULL)
    {
        LOGE("generate_attk_key_pair not supported");
        goto ret_handle;
    }

    ret = dev->verify_attk_key_pair(dev);
    if (ret == KM_ERROR_OK) {
        LOGD("Succeed in verifying exsiting attk");
        printf("pass\n");
    } else {
        LOGE("Failed to verify attk, error %d", ret);
        printf("fail %d\n", ret);
    }

ret_handle:
    if(dev)
        keymaster1_close(dev);
    return ret;
}

int export_attk_public_key(void)
{
    keymaster1_device_t* dev = 0;
    unsigned char *pub_key_ptr = NULL;
    size_t pub_key_data_length = PUBKEY_BUFF_SIZE;
    int ret = KM_ERROR_UNKNOWN_ERROR;

    if (keymaster_device_initialize(&dev)) {
        LOGE("keystore keymaster could not be initialized; exiting");
        return KM_ERROR_UNKNOWN_ERROR;
    }

    //call apis
    if (!dev || dev->export_attk_public_key == NULL)
    {
        LOGE("export_attk_public_key not supported");
        goto ret_handle;
    }

    pub_key_ptr = malloc(pub_key_data_length);
    if (pub_key_ptr == NULL)
    {
        LOGE("Allocation failed in %s for size = %ld", __func__,
                   pub_key_data_length);
        ret = KM_ERROR_MEMORY_ALLOCATION_FAILED;
        goto ret_handle;
    }
    memset(pub_key_ptr, 0, pub_key_data_length);
    ret = dev->export_attk_public_key(dev, pub_key_ptr, pub_key_data_length);
    if (ret != KM_ERROR_OK) {
        LOGE("Failed to export attk, error %d", ret);
        printf("fail %d\n", ret);
    } else
        printf("%s", pub_key_ptr);

ret_handle:
    if(dev)
        keymaster1_close(dev);
    return ret;
}

int generate_attk_key_pair(void)
{
    keymaster1_device_t* dev = 0;
    uint8_t copy_num = ATTK_COPY_NUM_DEFAULT;
    int ret = KM_ERROR_UNKNOWN_ERROR;

    if (keymaster_device_initialize(&dev)) {
        LOGE("keystore keymaster could not be initialized; exiting");
        return KM_ERROR_UNKNOWN_ERROR;
    }

    //call apis
    if (!dev || dev->generate_attk_key_pair == NULL)
    {
        LOGE("generate_attk_key_pair not supported");
        goto ret_handle;
    }

    ret = dev->generate_attk_key_pair(dev, copy_num);
    if (ret != KM_ERROR_OK) {
        LOGE("Failed to generate attk, error %d", ret);
        printf("fail %d\n", ret);
    } else
        printf("pass\n");

ret_handle:
    if(dev)
        keymaster1_close(dev);
    return ret;
}

int qsc_run_soter_cmd(char *argv[])
{
    int ret = KM_ERROR_UNKNOWN_ERROR;

    if(argv == NULL) {
        LOGE("No arguments to process, exiting!");
        return KM_ERROR_UNEXPECTED_NULL_POINTER;
    }
    /* CMD_ID */
    switch(atoi(argv[1])) {
    case CMD_GENERATE_ATTK_KEY_PAIR:
        ret = generate_attk_key_pair();
        break;
    case CMD_VERIFY_ATTK_KEY_PAIR:
        ret = verify_attk_key_pair();
        break;
    case CMD_EXPORT_ATTK_PUBLIC_KEY:
        ret = export_attk_public_key();
        break;
    case CMD_GET_DEVICE_ID:
        ret = get_device_id();
        break;
    default:
        LOGD("command not supported.");
        qsc_usage();
        break;
    }

    return ret;
}

int main(int argc, char *argv[])
{
    int ret = KM_ERROR_UNKNOWN_ERROR;

    if( !argv || (argc != 2) || (!strcmp(argv[1],"h")))
        qsc_usage();
    else {
        ret = qsc_run_soter_cmd(argv);
        if (ret)
            LOGE("qsc_run_soter_cmd failed,ret=%d", ret);
    }
    return ret;
}
