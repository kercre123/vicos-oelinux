/*!
 * @section LICENSE
 *
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *------------------------------------------------------------------------------
 *  Disclaimer
 *
 * Common: Bosch Sensortec products are developed for the consumer goods
 * industry. They may only be used within the parameters of the respective valid
 * product data sheet.  Bosch Sensortec products are provided with the express
 * understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive
 * systems or any system or device that may lead to bodily harm or property
 * damage if the system or device malfunctions. In addition, Bosch Sensortec
 * products are not fit for use in products which interact with motor vehicle
 * systems.  The resale and/or use of products are at the purchaser's own risk
 * and his own responsibility. The examination of fitness for the intended use
 * is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims,
 * including any claims for incidental, or consequential damages, arising from
 * any product use not covered by the parameters of the respective valid product
 * data sheet or not approved by Bosch Sensortec and reimburse Bosch Sensortec
 * for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety and inform Bosch Sensortec without
 * delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary
 * from the valid technical specifications of the product series. They are
 * therefore not intended or fit for resale to third parties or for use in end
 * products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series.
 * Bosch Sensortec assumes no liability for the use of engineering samples. By
 * accepting the engineering samples, the Purchaser agrees to indemnify Bosch
 * Sensortec from all claims arising from the use of engineering samples.
 *
 * Special: This software module (hereinafter called "Software") and any
 * information on application-sheets (hereinafter called "Information") is
 * provided free of charge for the sole purpose to support your application
 * work. The Software and Information is subject to the following terms and
 * conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch
 * Sensortec products by personnel who have special experience and training. Do
 * not use this Software if you do not have the proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or
 * implied warranties, including without limitation, the implied warranties of
 * merchantability and fitness for a particular purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for
 * the functional impairment of this Software in terms of fitness, performance
 * and safety. Bosch Sensortec and their representatives and agents shall not be
 * liable for any direct or indirect damages or injury, except as otherwise
 * stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch
 * Sensortec assumes no responsibility for the consequences of use of such
 * Information nor for any infringement of patents or other rights of third
 * parties which may result from its use.
 *
 * @file         sensord_pltf.c
 * @date         "Thu Feb 4 13:36:50 2016 +0800"
 * @commit       "7020e4d"
 *
 * @brief
 *
 * @detail
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "sensord_def.h"
#include "sensord_cfg.h"
#include "sensord_pltf.h"

#define SENSORD_TRACE_FILE (PATH_DIR_SENSOR_STORAGE "/sensord.log")
#define DATA_IN_FILE (PATH_DIR_SENSOR_STORAGE "/data_in.log")
#define BSX_DATA_LOG (PATH_DIR_SENSOR_STORAGE "/bsx_datalog.log")

static FILE *g_fp_trace = NULL;
static FILE *g_dlog_input = NULL;
static FILE *g_bsx_dlog = NULL;

static inline void storage_init()
{
    char *path = NULL;
    int ret = 0;
    struct stat st;

    path = (char *) (PATH_DIR_SENSOR_STORAGE);

    ret = stat(path, &st);
    if (0 == ret)
    {
        if (S_IFDIR == (st.st_mode & S_IFMT))
        {
            /*already exist*/
            ret = chmod(path, 0766);
            if (ret)
            {
                printf("error chmod on %s", path);
            }
        }

        return;
    }

    ret = mkdir(path, 0766);
    if (ret)
    {
        printf("error creating storage dir\n");
    }
    chmod(path, 0766); //notice the "umask" could mask some privilege when mkdir

    return;
}

void sensord_trace_init()
{
    g_fp_trace = fopen(SENSORD_TRACE_FILE, "w");
    if(NULL == g_fp_trace)
    {
        printf("sensord_trace_init: fail to open log file %s! \n", SENSORD_TRACE_FILE);
        g_fp_trace = stdout;
        return;
    }

    chmod(SENSORD_TRACE_FILE, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);

    return;
}

int64_t sensord_get_tmstmp_ns(void)
{

    int64_t ap_time;
    struct timespec ts;

    clock_gettime(CLOCK_BOOTTIME, &ts);
    ap_time = (int64_t) ts.tv_sec * 1000000000 + ts.tv_nsec;

    return ap_time;

}

void trace_log(uint32_t level, const char *fmt, ...)
{
    int ret = 0;
    va_list ap;
#if !defined(PLTF_LINUX_ENABLED)
    char buffer[256] = { 0 };
#endif

    if (0 == trace_to_logcat)
    {
        if (0 == (trace_level & level))
        {
            return;
        }

        va_start(ap, fmt);
        ret = vfprintf(g_fp_trace, fmt, ap);
        va_end(ap);

        // otherwise, data is buffered rather than be wrote to file
        // therefore when stopped by signal, NO data left in file!
        fflush(g_fp_trace);

        if (ret < 0)
        {
            printf("trace_log: fprintf(g_fp_trace, fmt, ap)  fail!!\n");
        }
    }
    else
    {

#if !defined(PLTF_LINUX_ENABLED)
        /**
         * here use android api
         * Let it use Android trace level.
         */
#include<android/log.h>
#define BST_LOG_TAG    "sensord"

        va_start(ap, fmt);
        vsnprintf(buffer, sizeof(buffer) - 1, fmt, ap);
        va_end(ap);

        switch (level)
        {
            case LOG_LEVEL_N:
                __android_log_print(ANDROID_LOG_FATAL, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_E:
                __android_log_print(ANDROID_LOG_ERROR, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_W:
                __android_log_print(ANDROID_LOG_WARN, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_I:
                __android_log_print(ANDROID_LOG_INFO, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_D:
                __android_log_print(ANDROID_LOG_DEBUG, BST_LOG_TAG, "%s", buffer);
                break;
            case LOG_LEVEL_LADON:
                __android_log_print(ANDROID_LOG_WARN, BST_LOG_TAG, "%s", buffer);
                break;
            default:
                break;
        }

#else

        if(0 == (trace_level & level))
        {
            return;
        }

        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);

#endif
    }

    return;
}

static void generic_data_log(const char*dest_path, FILE **p_dest_fp, char *info_str)
{
    if (NULL == (*p_dest_fp))
    {
        (*p_dest_fp) = fopen(dest_path, "w");
        if (NULL == (*p_dest_fp))
        {
            printf("fail to open file %s! \n", dest_path);
            return;
        }

        chmod(dest_path, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
    }

    fprintf((*p_dest_fp), "%s", info_str);

    // otherwise, data is buffered rather than be wrote to file
    // therefore when stopped by signal, NO data left in file!
    fflush((*p_dest_fp));

}

void data_log_algo_input(char *info_str)
{
    generic_data_log(DATA_IN_FILE, &g_dlog_input, info_str);
}

void bsx_datalog_algo(char *info_str)
{
    generic_data_log(BSX_DATA_LOG, &g_bsx_dlog, info_str);
}


void sensord_pltf_init(void)
{
    storage_init();

    sensord_trace_init();

    return;
}

void sensord_pltf_clearup(void)
{
    fclose(g_fp_trace);

    if (g_dlog_input)
    {
        fclose(g_dlog_input);
    }
    if(g_bsx_dlog)
    {
        fclose(g_bsx_dlog);
    }
    return;
}

