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
 * @file         BstSensor.h
 * @date         "Fri Feb 5 15:40:38 2016 +0800"
 * @commit       "666efb6"
 *
 * @brief
 *
 * @detail
 *
 */

#ifndef ANDROID_BST_SENSOR_H
#define ANDROID_BST_SENSOR_H

#include <stdio.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <signal.h>
#include <pthread.h>
#if !defined(PLTF_LINUX_ENABLED)
#include <hardware/sensors.h>
#else
#include "sensors.h"
#endif
#include "sensord_def.h"
#include "bstsimple_list.h"

typedef struct
{
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    BstSimpleList *p_list;
} SENSORD_SHARED_MEM;

class BstSensor
{
public:
    static BstSensor *getInstance();
    static void destroy();

    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t ns);
    int read_events(sensors_event_t* data, int count);
    int batch(int handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns);
    int flush(int handle);
#if defined(SENSORS_DEVICE_API_VERSION_1_4)
    int inject_sensor_data(const sensors_event_t *data);
#endif
    uint32_t get_sensorlist(struct sensor_t const** p_sSensorList);
    void sensord_read_rawdata();
    void sensord_deliver_event(sensors_event_t *p_event);

    int send_flush_event(int32_t sensor_id);

    int (*pfun_activate)(int handle, int enabled);
    int (*pfun_batch)(int handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns);
    int (*pfun_flush)(BstSensor *bstsensor, int handle);
    uint32_t (*pfun_get_sensorlist)(struct sensor_t const** p_sSensorList);
    uint32_t (*pfun_hw_deliver_sensordata)(BstSensor *bstsensor);

    BstSimpleList *tmplist_hwcntl_acclraw;
    BstSimpleList *tmplist_hwcntl_gyroraw;
    BstSimpleList *tmplist_hwcntl_magnraw;

    BstSimpleList *tmplist_sensord_acclraw;
    BstSimpleList *tmplist_sensord_gyroraw;
    BstSimpleList *tmplist_sensord_magnraw;

    SENSORD_SHARED_MEM shmem_hwcntl;
    int HALpipe_fd[2];

private:
    BstSensor();
    BstSensor(const BstSensor & other); //for cppcheck "noCopyConstructor"
    ~BstSensor();

    static BstSensor *instance;
    void sensord_cfg_init();

    pthread_t thread_sensord;
    pthread_t thread_hwcntl;
};

#endif  // ANDROID_BST_SENSOR_H
