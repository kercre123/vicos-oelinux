/*=============================================================================
Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

/**
 * @file spdaemon.c
 * @brief - Secure Processor Daemon (spdaemon)
 *
 * This driver is responsible for loading SP Applications that
 * doesn't have an owner HLOS Application.
 */

/*-------------------------------------------------------------------------
 * Include Files
 * ----------------------------------------------------------------------*/
#include <stdlib.h>    /* malloc() */
#include <stdio.h>     /* fopen() */
#include <fcntl.h>     /* O_RDONLY */
#include <unistd.h>    /* sleep() / usleep() */
#include <errno.h>     /* ENODEV */
#include <memory.h>
#include <pthread.h>
#include <time.h>

#include <cutils/log.h>        /* SLOGE() */

#include <spcomlib.h>

#include "mdm_detect.h"
#include "pm-service.h"
#include <cutils/properties.h> // property_get()

/*-------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * ----------------------------------------------------------------------*/
#ifdef PRINT_LOG_TO_STDOUT
    #define LOGD(fmt, x...) printf("spdaemon: dbg: %s: " fmt, __func__, ##x)
    #define LOGE(fmt, x...) printf("spdaemon: err: %s: " fmt, __func__, ##x)
#else /* print to system log a.k.a logcat */
    #undef LOG_TAG
    #undef LOGD
    #undef LOGE
    #define LOG_TAG "spdaemon"
    #define LOGD SLOGD
    #define LOGE SLOGE
#endif

#define SZ_1K (1024)
#define SZ_8K (8 * 1024)
#define SZ_1M (1024 * 1024)

#define KEYMASTER_ION_BUF_SIZE (4 * SZ_1K)

#ifndef OFFSET_OF
    #define OFFSET_OF(field, base) ((int)((char*)(field) - (char*)(base)))
#endif

/** Command Opcode */
#define KEYMASTER_INIT_CMD_ID  0xAA

/*-------------------------------------------------------------------------
 * Structures and enums
 * ----------------------------------------------------------------------*/

struct sp_app_info {
    const char *ch_name;
    const char *file_path;
    int swap_size;
};

struct keymaster_init_request {
    uint32_t cmd_opcode;
    uint64_t ion_buf_virt_addr;
} __attribute__((packed));

struct keymaster_init_response {
    uint32_t cmd_opcode;
    uint32_t error_code;
}  __attribute__((packed));

/* Note: The request and response structure format should be packed */

/*-------------------------------------------------------------------------
 * Global Variables
 * ----------------------------------------------------------------------*/

enum spss_firmware_type {
    SPSS_FW_TYPE_DEV = 'd',
    SPSS_FW_TYPE_TEST = 't',
    SPSS_FW_TYPE_PROD = 'p',
};

static enum spss_firmware_type firmware_type = SPSS_FW_TYPE_TEST;

/* SPSS Apps - Dev */
static const struct sp_app_info keymaster_app_info_dev = {
        .ch_name = "sp_keymaster",
        .file_path = "/firmware/image/keym_d.sig", /* FAT 8.3 chars format */
        .swap_size = 256 * SZ_1K,
};

static const struct sp_app_info cryptoapp_app_info_dev = {
        .ch_name = "cryptoapp",
        .file_path = "/firmware/image/crypt_d.sig",
        .swap_size = 256 * SZ_1K,
};

/* SPSS Apps - Test */
static const struct sp_app_info keymaster_app_info_test = {
        .ch_name = "sp_keymaster",
        .file_path = "/firmware/image/keym_t.sig", /* FAT 8.3 chars format */
        .swap_size = 256 * SZ_1K,
};

static const struct sp_app_info cryptoapp_app_info_test = {
        .ch_name = "cryptoapp",
        .file_path = "/firmware/image/crypt_t.sig",
        .swap_size = 256 * SZ_1K,
};

/* SPSS Apps - Prod */
static const struct sp_app_info keymaster_app_info_prod = {
        .ch_name = "sp_keymaster",
        .file_path = "/firmware/image/keym_p.sig",
        .swap_size = 256 * SZ_1K,
};

static const struct sp_app_info cryptoapp_app_info_prod = {
        .ch_name = "cryptoapp",
        .file_path = "/firmware/image/crypt_p.sig",
        .swap_size = 256 * SZ_1K,
};

/* The spss_utils kernel driver sysfs path depends on the linux kernel version */
static const char *test_fuse_state_path =  "./sys/devices/platform/soc/soc:qcom,spss_utils/test_fuse_state";
static const char *test_fuse_state_path2 = "./sys/devices/soc/soc:qcom,spss_utils/test_fuse_state";

static int keymaster_ion_fd = -1;
static void *keymaster_ion_vbuf = NULL;
static struct spcom_client *keymaster_spcom_client = NULL;

static pthread_mutex_t pthread_mutex;
static pthread_cond_t  pthread_cond;
static bool sp_reset_detected = false;

static void *pm_spss_handle = NULL;

/*-------------------------------------------------------------------------
 * Function Implementations
 * ----------------------------------------------------------------------*/
static inline void msleep(int msec)
{
    usleep(msec * 1000);
}

static enum spss_firmware_type get_firmware_type(void)
{
    int fd;
    int ret;
    char type[10] = {};
    const char *path = test_fuse_state_path;

    fd = open(path, O_RDONLY);
    if (fd < 0) {
	LOGD("open() file [%s] Failed, ret [%d] try another path.\n", path, fd);
	path = test_fuse_state_path2; /* Alternative path */

	fd = open(path, O_RDONLY);
	if (fd < 0) {
		LOGE("open() file [%s] Failed, ret =%d.\n", path, fd);
		return SPSS_FW_TYPE_TEST;
	}
    }

    ret = read(fd, type, sizeof(type));
    if (ret < 0) {
        LOGE("read() file [%s] Failed, ret =%d.\n", path, ret);
    } else {
        if (strcmp("dev", type) == 0) {
            LOGD("Dev FW.\n");
            return SPSS_FW_TYPE_DEV;
        }
        if (strcmp("test", type) == 0) {
            LOGD("Test FW.\n");
            return SPSS_FW_TYPE_TEST;
        }
        if (strcmp("prod", type) == 0) {
            LOGD("Prod FW.\n");
            return SPSS_FW_TYPE_PROD;
        }
    }

    return SPSS_FW_TYPE_TEST;
}

static void suspend_me(void)
{
    pthread_mutex_lock(&pthread_mutex);
    pthread_cond_wait(&pthread_cond, &pthread_mutex);
}

static void resume_me(void)
{
    pthread_cond_signal(&pthread_cond);
    pthread_mutex_unlock(&pthread_mutex);
}

static int load_app(const struct sp_app_info *info)
{
    int ret;
    bool is_loaded = false;
    int timeout_msec = 60 * 1000;
    int time_msec = 0;
    const char *ch_name = info->ch_name;

    /* check if already loaded */
    is_loaded = spcom_is_app_loaded(ch_name);
    if (is_loaded) {
        LOGD("SP App [%s] already loaded.\n", ch_name);
        return 0;
    }

    LOGD("Load SP App [%s].\n", ch_name);

    /* Load the app */
    ret = spcom_load_app(info->ch_name,
            info->file_path,
            info->swap_size);
    if (ret < 0) {
        LOGE("Loading SP App [%s] failed. ret [%d].\n", ch_name, ret);
        return ret;
    }

    LOGD("Verify is_app_loaded() is set for app [%s].\n", ch_name);

    while (!is_loaded) {
        is_loaded = spcom_is_app_loaded(ch_name);
        msleep(10);
        time_msec += 10;
        if (time_msec >= timeout_msec) {
            LOGE("Timeout wait for char dev creation.\n");
            return -ETIMEDOUT;
        }
    }

    LOGD("SP App [%s] is loaded successfully.\n", ch_name);

    return 0;
}

/**
 * spcom_notify_ssr_cb() - a callback to notify on SP SubSystem-Reset (SSR).
 *
 * The spcom shall notify upon SP reset.
 * This callback should wait until the SP is up again (LINK is UP),
 * and then re-load the SP Applications and do any init sequence required.
 */
static void spcom_notify_ssr_cb(void)
{
    LOGD("SP subsystem reset detected.\n");

    sp_reset_detected = true;

    resume_me();
}

/**
 * Init Keymaster App.
 *
 * The SP App needs a "work-buffer" carved from the DDR by ION.
 *
 */
static int sp_keymaster_init(void)
{
    int ret;
    int ion_fd = -1;
    void *ion_vbuf = NULL;
    struct keymaster_init_request req = {};
    struct keymaster_init_response resp = {};
    struct spcom_client *client = NULL;
    struct spcom_client_info reg_info = {};
    bool is_connected = false;
    int timeout_msec = 60 * 1000;
    int time_msec = 0;
    struct spcom_ion_info_table ion_info_table;
    struct spcom_ion_info *ion_info = ion_info_table.ion_info;
    uint32_t cmd_timeout_msec = 1000; // timeout for SP to handle the command
    /* Note: same channel name for "test" / "prod" / "hybr" */
    const char *ch_name = keymaster_app_info_test.ch_name;

    ion_info_table.ion_info[0].fd = -1 ; // mark as invalid
    ion_info_table.ion_info[1].fd = -1 ; // mark as invalid
    ion_info_table.ion_info[2].fd = -1 ; // mark as invalid
    ion_info_table.ion_info[3].fd = -1 ; // mark as invalid

    reg_info.ch_name = ch_name;
    reg_info.notify_ssr_cb = spcom_notify_ssr_cb;

    /* register to spcom for sending request */
    client = spcom_register_client(&reg_info);
    if (client == NULL) {
        LOGE("spcom register failed.\n");
        goto exit_err;
    }

    /* wait for remote SP App to connect */
    while (!is_connected) {
        is_connected = spcom_client_is_server_connected(client);
        msleep(10);
        time_msec += 10;
        if (time_msec >= timeout_msec) {
            LOGE("Timeout wait for ch CONNECT.\n");
            goto exit_err;
        }
    }

    LOGD("Unlock and free previous allocated buffers.\n");
    ret = spcom_unlock_ion_buffer(ch_name, SPCOM_ION_FD_UNLOCK_ALL);
    keymaster_ion_fd = -1;
    keymaster_ion_vbuf = NULL;

    ion_vbuf = spcom_ion_alloc(KEYMASTER_ION_BUF_SIZE, &ion_fd);
    if (ion_vbuf == NULL) {
        LOGE("Failed to allocate ION buffer.\n");
        goto exit_err;
    }

    ion_info->fd = ion_fd;
    ion_info->buf_offset = OFFSET_OF(&req.ion_buf_virt_addr, &req);

    req.cmd_opcode = KEYMASTER_INIT_CMD_ID;
    req.ion_buf_virt_addr = (uint64_t) ion_vbuf;

    ret = spcom_client_send_modified_command(client,
            &req, sizeof(req),
            &resp, sizeof(resp),
            &ion_info_table,
            cmd_timeout_msec);
    if (ret <= 0) {
        LOGE("send command failed.\n");
        goto exit_err;
    }

    if (resp.error_code != 0) {
        LOGE("response error_code [%d]\n", resp.error_code);
        goto exit_err;
    }

    ret = spcom_lock_ion_buffer(ch_name, ion_fd);
    if (ret < 0) {
        LOGE("lock ION buf failed.\n");
        goto exit_err;
    }

    /* Note1: don't unregister spcom for SSR awareness */

    /*
     * ION alloc/free API is a wrapper for using /dev/ion from user space.
     * ION lock/unlock is done by the spcom kernel.
     * The buffer is locked for SPSS to use it safely.
     * However, if a buffer is lock, the kernel ION driver doesn't free the buffer
     * until the ION buffer is unlocked.
     * This client unlock the ION buffer after SSR / daemon re-restart.
     * The ION buf should be unlocked while the channel is connected.
     */
    spcom_ion_free(ion_vbuf, KEYMASTER_ION_BUF_SIZE, ion_fd);

    LOGD("Keymaster init completed ok.\n");
    keymaster_ion_fd = ion_fd;
    keymaster_ion_vbuf = ion_vbuf;
    keymaster_spcom_client = client;

    return 0;
exit_err:
    if (client != NULL)
        spcom_unregister_client(client);
    if (ion_vbuf != NULL)
        spcom_ion_free(ion_vbuf, KEYMASTER_ION_BUF_SIZE, ion_fd);

    return -EFAULT;
}

static int register_ssr_callback(void)
{
    int ret;
    struct spcom_client *client = NULL;
    struct spcom_client_info reg_info = {};
    /* Note: same channel name for "test" / "prod" / "hybr" */
    const char *ch_name = keymaster_app_info_test.ch_name;

    reg_info.ch_name = ch_name;
    reg_info.notify_ssr_cb = spcom_notify_ssr_cb;

    /* register to spcom for sending request */
    client = spcom_register_client(&reg_info);
    if (client == NULL) {
        LOGE("spcom register failed.\n");
	return -EFAULT;
    }

    /* Note1: don't unregister spcom for SSR awareness */

    LOGD("SSR callback registered ok.\n");
    keymaster_spcom_client = client;

    return 0;
}


/**
 * Keymaster App exit cleanup.
 *
 */
static int sp_keymaster_exit(void)
{
    spcom_unregister_client(keymaster_spcom_client);

    keymaster_spcom_client = NULL;

    return 0;
}


/**
 *  Wait until SP is up and running.
 */
static int wait_for_sp_link_up(int timeout_sec)
{
    bool sp_is_up = false;
    int timeout_msec = timeout_sec * 1000;
    int time_msec = 0;

    while (!sp_is_up) {
        sp_is_up = spcom_is_sp_subsystem_link_up();
        msleep(10);
        time_msec += 10;
        if (time_msec >= timeout_msec) {
            LOGE("Timeout wait for SP link UP.\n");
            return -ETIMEDOUT;
        }
    }


    LOGD("SP LINK is UP in [%d] msec.\n", time_msec);

    return 0;
}

/**
 * Load SP App
 */
static int keymaster_load_and_init(void)
{
    int ret;

    switch (firmware_type) {
    case SPSS_FW_TYPE_DEV:
        ret = load_app(&keymaster_app_info_dev);
        break;
    case SPSS_FW_TYPE_TEST:
        ret = load_app(&keymaster_app_info_test);
        break;
    case SPSS_FW_TYPE_PROD:
        ret = load_app(&keymaster_app_info_prod);
        break;
    default:
        return -EINVAL;
    }

    if (ret < 0)
        return ret;

    ret = sp_keymaster_init();
    if (ret < 0)
        return ret;

    LOGD("SP keymaster App is initialized successfully.\n");

    return 0;
}

static void pm_spss_event_notifier(void *client_data, enum pm_event event)
{
    char *client_name = (char*) client_data;

    LOGD("client [%s] got event [%d] notification", client_name , (int) event);

    /* Don't really care much here about events.Just ack whatever comes in. */
    pm_client_event_acknowledge(pm_spss_handle, event);
}

/*
 * PIL load sp
 */
static int pil_load_sp(void)
{
    int i, ret = 0;
    struct dev_info devinfo = {};
    enum pm_event event = 0;
    bool spss_found = false;
    void *client_data = "spdaemon";

    LOGD("Starting to get system info");
    ret = get_system_info(&devinfo);
    if (ret != RET_SUCCESS) {
        LOGE("Failed to get_system_info");
        goto error;
    }

    LOGD("devinfo.num_additional_subsystems = %d",
            devinfo.num_additional_subsystems);
    for (i = 0; i < devinfo.num_additional_subsystems; i++) {
        LOGD("devinfo.subsystem_list[%d].type = %d",
                i, devinfo.subsystem_list[i].type);
        if (devinfo.subsystem_list[i].type == SUBSYS_TYPE_SPSS) {
            spss_found = true;
            LOGD("Found spss subsystem.");
            LOGD("devinfo.subsystem_list[%d].mdm_name = %s",
                    i , devinfo.subsystem_list[i].mdm_name);
            ret = pm_client_register(pm_spss_event_notifier,
                    client_data,
                    devinfo.subsystem_list[i].mdm_name, /* "spss" dev name */
                    "spdaemon", /* client name */
                    &event,
                    &pm_spss_handle);
            if (ret != PM_RET_SUCCESS) {
                LOGE("pm_client_register failed. ret [%d].\n", ret);
                goto error;
            }
            LOGD("pm-spss-thread Voting for spss subsystem");
            ret = pm_client_connect(pm_spss_handle);
            if (ret != PM_RET_SUCCESS) {
                LOGE("pm_client_connect() for spss fail. ret [%d].\n", ret);
                goto error;
            }
        }
    }

    if (!spss_found) {
        LOGE("SUBSYS_TYPE_SPSS not found.\n");
        goto error;
    }

    LOGD("SPSS-PIL completed successfully");

    return 0;
error:
    LOGD("SPSS-PIL failed.");
    return -EFAULT;
}

/**
 *  re-load SP App after SP reset
 */
static int handle_sp_reset_event(void)
{
    int ret;
    int link_up_timeout_sec = 60;

    ret = wait_for_sp_link_up(link_up_timeout_sec);
    if (ret < 0) {
        LOGE("wait_for_sp_link_up failed. ret [%d].\n", ret);
        return ret;
    }

    switch (firmware_type) {
    case SPSS_FW_TYPE_DEV:
        ret = load_app(&cryptoapp_app_info_dev);
        break;
    case SPSS_FW_TYPE_TEST:
        ret = load_app(&cryptoapp_app_info_test);
        break;
    case SPSS_FW_TYPE_PROD:
        ret = load_app(&cryptoapp_app_info_prod);
        break;
    default:
        return -EINVAL;
    }

    if (ret < 0) {
        LOGE("Loading SP cryptoapp App failed. ret [%d].\n", ret);
        return ret;
    }

    /*
     * Cleanup after SSR:
     * - Free ION buffers used by the SP app.
     * - Unregister old client - cleanup old SSR thread before re-register.
     */
    sp_keymaster_exit();

    ret = keymaster_load_and_init();
    if (ret < 0) {
        LOGE("keymaster_load_and_init failed. ret [%d].\n", ret);
        return ret;
    }

    LOGD("keymaster init successfully after SSR.\n");

    return 0;
}

int main(void)
{
    int ret = -1;
    int link_up_timeout_sec = 60;
    pthread_t self_id = pthread_self();
    char bootmode[PROPERTY_VALUE_MAX];

    LOGD("Version 2.2 07-July-2017.\n");

    firmware_type = get_firmware_type();
    LOGD("firmware_type [%d].\n", (int) firmware_type);

    // In FFBM mode - don't load spss
    property_get("ro.bootmode", bootmode, NULL);
    if (!strncmp(bootmode, "ffbm", 4)) {
        LOGD("FFBM mode, SPSS will not be loaded");
        goto load_app_error;
    }

    ret = pil_load_sp();
    if (ret != 0) {
        LOGE("pil_load_sp failed. ret [%d].\n", ret);
        goto pil_error;
    }

    LOGD("Wait for sp link up.\n");
    ret = wait_for_sp_link_up(link_up_timeout_sec);
    if (ret < 0) {
        LOGE("wait for link up. ret [%d].\n", ret);
        goto pil_error;
    }

    LOGD("Check if cryptoapp app is already loaded.\n");
    if (!spcom_is_app_loaded(cryptoapp_app_info_test.ch_name)) {

		LOGD("Cryptoapp app is NOT loaded.\n");

        switch (firmware_type) {
        case SPSS_FW_TYPE_DEV:
			LOGD("SPSS_FW_TYPE_DEV.\n");
            ret = load_app(&cryptoapp_app_info_dev);
            break;
        case SPSS_FW_TYPE_TEST:
			LOGD("SPSS_FW_TYPE_TEST.\n");
            ret = load_app(&cryptoapp_app_info_test);
            break;
        case SPSS_FW_TYPE_PROD:
			LOGD("SPSS_FW_TYPE_PROD.\n");
            ret = load_app(&cryptoapp_app_info_prod);
            break;
        default:
			LOGE("Invalid SPSS FW TYPE.\n");
			goto load_app_error;
        }

        if (ret < 0) {
            LOGE("Loading SP cryptoapp App failed. ret [%d].\n", ret);
            goto load_app_error;
        }
    }

    LOGD("Check if keymaster app is already loaded.\n");
    if (!spcom_is_app_loaded(keymaster_app_info_test.ch_name)) {
        ret = keymaster_load_and_init();
        if (ret < 0) {
            LOGE("Loading SP keymaster App failed. ret [%d].\n", ret);
            goto load_app_error;
        }
    } else {
		register_ssr_callback();
    }

    while(1) {
        /* go to sleep , nothing to do now , wake up upon SP reset event */
        LOGD("Go to sleep , nothing to do now , wake up upon SP reset event.\n");
        pthread_cond_init(&pthread_cond, NULL);
        pthread_mutex_init(&pthread_mutex, NULL);

        sp_reset_detected = false;
        suspend_me();
        if (sp_reset_detected)
            handle_sp_reset_event();
    }

    sp_keymaster_exit(); /* should probably never happen */

    return 0;

pil_error:
    LOGD("SPSS-PIL Load failure, power down the SPSS.\n");
    pm_client_disconnect(pm_spss_handle) ;
    pm_client_unregister(pm_spss_handle) ;

load_app_error:
    /* Avoid exit, since init process will start the daemon again */
    while(1) {
        LOGD("going to sleep for a day.\n");
        sleep(60*60*24); /* sleep 60 sec x 60 min x 24 hours = 1 day */
    }

    return -ENODEV; /* never happens */
}
