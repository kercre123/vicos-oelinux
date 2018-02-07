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
 * @file         sensord_def.h
 * @date         "Thu Aug 27 01:58:58 2015 -0400"
 * @commit       "7184568"
 *
 * @brief
 *
 * @detail
 *
 */

#ifndef __SENSORD_DEF_H
#define __SENSORD_DEF_H

#if defined(__FASTEST_MODE_100HZ__)
#define BST_SENSOR_MINDELAY_uS 10000
#else
#define BST_SENSOR_MINDELAY_uS 5000
#endif

/* this value depends on the reporting mode:
 *
 *   continuous: minimum sample period allowed in microseconds
 *   on-change : 0
 *   one-shot  :-1
 *   special   : 0, unless otherwise noted
 */
#define SENSOR_MINDELAY_ONCHANGE 200000 //BSX4 library can only support minimum 200ms(200000us) delay
#define SENSOR_MINDELAY_ONESHOT (-1)
#define SENSOR_MINDELAY_SPECIAL 0

/* This value is defined only for continuous mode and on-change sensors. It is the delay between
 * two sensor events corresponding to the lowest frequency that this sensor supports. When lower
 * frequencies are requested through batch()/setDelay() the events will be generated at this
 * frequency instead. It can be used by the framework or applications to estimate when the batch
 * FIFO may be full.
 *
 * NOTE: 1) period_ns is in nanoseconds where as maxDelay/minDelay are in microseconds.
 *              continuous, on-change: maximum sampling period allowed in microseconds.
 *              one-shot, special : 0
 *   2) maxDelay should always fit within a 32 bit signed integer. It is declared as 64 bit
 *      on 64 bit architectures only for binary compatibility reasons.
 * Availability: SENSORS_DEVICE_API_VERSION_1_3
 */
#define SENSOR_MAXDELAY_ONESHOT 0
#define SENSOR_MAXDELAY_SPECIAL 0
/*for ON_CHANGE type, sample period is actually a delay period between events reporting
 * so define a default value here
 */
#define SENSOR_MAXDELAY_ONCHANGE 1800000000 //BSX4 library support maximum 1800s(1800000000us) delay

#define BATCH_RSV_FRAME_COUNT 0
#define BATCH_MAX_FRAME_COUNT 2000

typedef struct
{
    uint32_t id;
    union
    {
        struct
        {
            float x;
            float y;
            float z;
        };
        struct
        {
            float azimuth;
            float pitch;
            float roll;
        }; //for ORIENTATION
        float pressure;
        float temperature;
        float data[4]; // for RV, Game RV, ALSH debug raw data
        struct
        {
            float x_uncalib;
            float y_uncalib;
            float z_uncalib;
            float x_bias;
            float y_bias;
            float z_bias;
        }; //for GYROSCOPE/MAGNETIC UNCALIBRATED
        uint64_t step_counter;
        struct
        {
            float heart_rate_bpm;
            int8_t heart_rate_status;
        };
        float relative_humidity;
        float ambient_temperature;
        float light;
        float proximity;

        //ALSH private virtual sensor
        int32_t power_consumption;
        int32_t activity_wakeup;
    };

    int8_t accuracy;
    int64_t timestamp;
} HW_DATA_UNION;

/* this path must exist and user <system> must have permission to write to it */
#if !defined(PLTF_LINUX_ENABLED)
#define PATH_DIR_SENSOR_STORAGE "/data/misc/sensord_stor"
#else
#if defined(SENSORD_STOR)
#define PATH_DIR_SENSOR_STORAGE SENSORD_STOR"/sensord_stor"
#else
#define PATH_DIR_SENSOR_STORAGE "/root/My_C/sensord_stor"
#endif
#endif


#endif
