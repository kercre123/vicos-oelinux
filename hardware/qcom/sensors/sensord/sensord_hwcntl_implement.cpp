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
 * @file         sensord_hwcntl_implement.cpp
 * @date         "Fri Feb 5 15:40:38 2016 +0800"
 * @commit       "666efb6"
 *
 * @brief
 *
 * @detail
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/cdefs.h>
#include <sys/stat.h>
#include <linux/input.h>
#include <sys/timerfd.h>
#include <poll.h>
#include <fcntl.h>
#include <dirent.h>

#if !defined(PLTF_LINUX_ENABLED)
#include<android/log.h>
/*Android utils headers*/
#include <utils/SystemClock.h>
#include <utils/Timers.h>
#endif

#include "BstSensor.h"

#include "axis_remap.h"
#include "sensord_hwcntl.h"
#include "sensord_pltf.h"
#include "sensord_cfg.h"
#include "sensord_algo.h"
#include "util_misc.h"
#include "sensord_hwcntl_iio.h"

/* input event definition
struct input_event {
    struct timeval time;
    __u16 type;
    __u16 code;
    __s32 value;
};

EV_SYN            0x00 EV_KEY            0x01 EV_REL            0x02
EV_ABS            0x03 EV_MSC            0x04 EV_SW            0x05 EV_LED            0x11
EV_SND            0x12 EV_REP            0x14 EV_FF            0x15 EV_PWR            0x16
EV_FF_STATUS        0x17 EV_MAX            0x1f EV_CNT            (EV_MAX+1)
*/


#define FIFO_HEAD_A        0x84
#define FIFO_HEAD_G        0x88
#define FIFO_HEAD_M        0x90

#define BMI160_ACCEL_ODR_RESERVED       0x00
#define BMI160_ACCEL_ODR_0_78HZ         0x01
#define BMI160_ACCEL_ODR_1_56HZ         0x02
#define BMI160_ACCEL_ODR_3_12HZ         0x03
#define BMI160_ACCEL_ODR_6_25HZ         0x04
#define BMI160_ACCEL_ODR_12_5HZ         0x05
#define BMI160_ACCEL_ODR_25HZ           0x06
#define BMI160_ACCEL_ODR_50HZ           0x07
#define BMI160_ACCEL_ODR_100HZ          0x08
#define BMI160_ACCEL_ODR_200HZ          0x09
#define BMI160_ACCEL_ODR_400HZ          0x0A
#define BMI160_ACCEL_ODR_800HZ          0x0B
#define BMI160_ACCEL_ODR_1600HZ         0x0C

/* BMI160 Gyro ODR */
#define BMI160_GYRO_ODR_RESERVED   0x00
#define BMI160_GYRO_ODR_25HZ   0x06
#define BMI160_GYRO_ODR_50HZ   0x07
#define BMI160_GYRO_ODR_100HZ   0x08
#define BMI160_GYRO_ODR_200HZ   0x09
#define BMI160_GYRO_ODR_400HZ   0x0A
#define BMI160_GYRO_ODR_800HZ   0x0B
#define BMI160_GYRO_ODR_1600HZ   0x0C
#define BMI160_GYRO_ODR_3200HZ   0x0D

/* BMI160 Mag ODR */
#define BMI160_MAG_ODR_RESERVED       0x00
#define BMI160_MAG_ODR_0_78HZ         0x01
#define BMI160_MAG_ODR_1_56HZ         0x02
#define BMI160_MAG_ODR_3_12HZ         0x03
#define BMI160_MAG_ODR_6_25HZ         0x04
#define BMI160_MAG_ODR_12_5HZ         0x05
#define BMI160_MAG_ODR_25HZ           0x06
#define BMI160_MAG_ODR_50HZ           0x07
#define BMI160_MAG_ODR_100HZ          0x08
#define BMI160_MAG_ODR_200HZ          0x09
#define BMI160_MAG_ODR_400HZ          0x0A
#define BMI160_MAG_ODR_800HZ          0x0B
#define BMI160_MAG_ODR_1600HZ         0x0C

/**for BMA2x2, ODR = Filter Bandwidth x 2*/
#define BMA2X2_BW_7_81HZ        0x08
#define BMA2X2_BW_15_63HZ       0x09
#define BMA2X2_BW_31_25HZ       0x0A
#define BMA2X2_BW_62_50HZ       0x0B
#define BMA2X2_BW_125HZ         0x0C
#define BMA2X2_BW_250HZ         0x0D
#define BMA2X2_BW_500HZ         0x0E
#define BMA2X2_BW_1000HZ        0x0F
#define BMA2X2_ODR_15_63HZ       BMA2X2_BW_7_81HZ
#define BMA2X2_ODR_31_25HZ       BMA2X2_BW_15_63HZ
#define BMA2X2_ODR_62_50HZ       BMA2X2_BW_31_25HZ
#define BMA2X2_ODR_125HZ         BMA2X2_BW_62_50HZ
#define BMA2X2_ODR_250HZ         BMA2X2_BW_125HZ
#define BMA2X2_ODR_500HZ         BMA2X2_BW_250HZ
#define BMA2X2_ODR_1000HZ        BMA2X2_BW_500HZ
#define BMA2X2_ODR_2000HZ        BMA2X2_BW_1000HZ

/**for BMG160, ODR mapped to Filter Bandwidth according to this table
 * ODR_Hz      Bandwidth
 *  100         32Hz
 *  200         64Hz
 *  100         12Hz    --- optimal for 100Hz
 *  200         23Hz    --- optimal for 200Hz
 *  400         47Hz
 *  1000        116Hz
 *  2000        230Hz   --- optimal for 2000Hz
 *  2000        523Hz(Unfiltered)*/
#define BMG160_BW_12HZ          0x05
#define BMG160_BW_23HZ          0x04
#define BMG160_BW_32HZ          0x07
#define BMG160_BW_47HZ          0x03
#define BMG160_BW_64HZ          0x06
#define BMG160_BW_116HZ         0x02
#define BMG160_BW_230HZ         0x01
#define BMG160_BW_523HZ         0x00
#define BMG160_ODR_100HZ        BMG160_BW_12HZ
#define BMG160_ODR_200HZ        BMG160_BW_23HZ
#define BMG160_ODR_400HZ        BMG160_BW_47HZ
#define BMG160_ODR_1000HZ       BMG160_BW_116HZ
#define BMG160_ODR_2000HZ       BMG160_BW_230HZ

#define BMI160_ACCEL_RANGE_2G   3
#define BMI160_ACCEL_RANGE_4G   5
#define BMI160_ACCEL_RANGE_8G   8
#define BMI160_ACCEL_RANGE_16G  12

#define BMA2X2_RANGE_2G     3
#define BMA2X2_RANGE_4G     5
#define BMA2X2_RANGE_8G     8
#define BMA2X2_RANGE_16G    12

#define BMA2x2_FIFO_PASSBY  0
#define BMA2x2_FIFO_STREAM  2
#define BMG160_FIFO_PASSBY  0
#define BMG160_FIFO_STREAM  2

#define BMA222E_ADC_BITS    8
#define BMA250E_ADC_BITS    10
#define BMA255_ADC_BITS     12
#define BMA280_ADC_BITS     14
#define BMM_COMPVAL_TO_uT (16) //bmm compensation value need divide 16 to become uT
#define AKM09912_COMPVAL_TO_uT (0.15) //AKM09912 output is in unit of 0.15uT
#define AKM09911_COMPVAL_TO_uT (0.6) //AKM09911 output is in unit of 0.6uT
#define YAS5xx_COMPVAL_TO_uT (0.001) //YAS5xx output is in unit of 0.001uT

static char iio_dev0_dir_name[128] = { 0 };

/**
 * test by practice, if IIO buffer is set too small, data will loose
 * now hwdata_unit_toread = 10 and default_watermark = 10 work well on 200Hz
 */

static uint32_t hwdata_unit_toread = 10;
static uint32_t default_watermark = 10;


static int32_t accl_scan_size, gyro_scan_size, magn_scan_size;
static int32_t accl_iio_fd = -1;
static int32_t gyro_iio_fd = -1;
static int32_t magn_iio_fd = -1;
static int acc_input_fd = -1;
static int acc_input_num = 0;
static int magn_input_fd = -1;
static int magn_input_num = 0;
static int gyr_input_fd = -1;
static int gyr_input_num = 0;

static char mag_input_dir_name[128] = {0};
static char acc_input_dir_name[128] = {0};
static char gyr_input_dir_name[128] = {0};

static float BMI160_acc_resl = 0.061; //16bit ADC, default range +-2000 mg. algorithm input requires "mg"
static float BMA255_acc_resl = 0.97656; //12bit ADC, default range +-2000 mg. algorithm input requires "mg"

/**
 *
 * @param p_sSensorList
 * @return
 */
uint32_t ap_get_sensorlist(struct sensor_t const** p_sSensorList)
{
    uint64_t avail_sens_regval = 0;
    uint32_t sensor_amount = 0;
    int32_t i;
    int32_t j;

    if (0 == bst_sensorlist.list_len)
    {
        switch(accl_range){
            case ACC_CHIP_RANGCONF_2G:
            case ACC_CHIP_RANGCONF_4G:
            case ACC_CHIP_RANGCONF_8G:
            case ACC_CHIP_RANGCONF_16G:
                bst_all_sensors[SENSORLIST_INX_ACCELEROMETER].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_LINEAR_ACCELERATION].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_GRAVITY].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_ACCELEROMETER].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_LINEAR_ACCELERATION].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_GRAVITY].maxRange = accl_range * GRAVITY_EARTH;
                break;
            default:
                PWARN("Invalid accl_range: %d", accl_range);
                break;
        }

        if(MAG_CHIP_AKM09912 ==  magn_chip || MAG_CHIP_AKM09911 ==  magn_chip)
        {
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].name = "AKM Magnetic Field Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].vendor = "AKM";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].maxRange = 4900.0f;
            if(MAG_CHIP_AKM09912 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.15f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 1.0f;
            }else if(MAG_CHIP_AKM09911 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.6f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 2.4f;
            }
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].name = "AKM Magnetic Field Uncalibrated Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].vendor = "AKM";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 4900.0f;
            if(MAG_CHIP_AKM09912 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 1.0f;
            }else if(MAG_CHIP_AKM09911 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.6f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 2.4f;
            }

            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].name = "AKM Magnetic Field (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].vendor = "AKM";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].maxRange = 4900.0f;
            if(MAG_CHIP_AKM09912 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.15f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 1.0f;
            }else if(MAG_CHIP_AKM09911 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.6f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 2.4f;
            }
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].name = "AKM Magnetic Field Uncalibrated (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].vendor = "AKM";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 4900.0f;
            if(MAG_CHIP_AKM09912 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 1.0f;
            }else if(MAG_CHIP_AKM09911 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.6f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 2.4f;
            }
        }

        if(MAG_CHIP_YAS537 == magn_chip || MAG_CHIP_YAS532 == magn_chip)
        {
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].name = "YAS Magnetic Field Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].vendor = "YAS";
            if(MAG_CHIP_YAS537 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].maxRange = 2000.0f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.3f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 1.8f;
            }else if(MAG_CHIP_YAS532 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].maxRange = 1200.0f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.15f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 2.6f;
            }
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].name = "YAS Magnetic Field Uncalibrated Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].vendor = "YAS";
            if(MAG_CHIP_YAS537 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 2000.0f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.3f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 1.8f;
            }else if(MAG_CHIP_YAS532 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 1200.0f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15f;
                bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 2.6f;
            }

            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].name = "YAS Magnetic Field (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].vendor = "YAS";
            if(MAG_CHIP_YAS537 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].maxRange = 2000.0f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.3f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 1.8f;
            }else if(MAG_CHIP_YAS532 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].maxRange = 1200.0f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.15f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 2.6f;
            }
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].name = "YAS Magnetic Field Uncalibrated (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].vendor = "YAS";
            if(MAG_CHIP_YAS537 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 2000.0f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.3f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 1.8f;
            }else if(MAG_CHIP_YAS532 ==  magn_chip){
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 1200.0f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15f;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 2.6f;
            }
        }

        if(SOLUTION_MDOF == solution_type)
        {
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].minDelay = 20000;
        }
        /*So far compass / m4g subset library support max 25/50Hz on virtual sensors*/
        else if(SOLUTION_ECOMPASS == solution_type)
        {
            bst_all_sensors[SENSORLIST_INX_ORIENTATION].minDelay = 40000;
            bst_all_sensors[SENSORLIST_INX_GRAVITY].minDelay = 40000;
            bst_all_sensors[SENSORLIST_INX_LINEAR_ACCELERATION].minDelay = 40000;
            bst_all_sensors[SENSORLIST_INX_ROTATION_VECTOR].minDelay = 40000;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_ROTATION_VECTOR].minDelay = 40000;
        }else if(SOLUTION_M4G == solution_type){
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_ORIENTATION].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_GYROSCOPE].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_GRAVITY].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_LINEAR_ACCELERATION].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_ROTATION_VECTOR].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_GYROSCOPE_UNCALIBRATED].minDelay = 20000;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_ROTATION_VECTOR].minDelay = 20000;
        }

        avail_sens_regval = 0x14002;

        sensor_amount = sensord_popcount_64(avail_sens_regval);

        bst_sensorlist.list = (struct sensor_t *) malloc(sensor_amount * sizeof(struct sensor_t));
        if (NULL == bst_sensorlist.list)
        {

            PERR("fail to malloc %d * sizeof(struct sensor_t)(=%d)",
                    sensor_amount, sizeof(struct sensor_t));
            return 0;
        }

        bst_sensorlist.bsx_list_index = (int32_t *) malloc(sensor_amount * sizeof(int32_t));
        if (NULL == bst_sensorlist.bsx_list_index)
        {

            PERR("fail to malloc %d * 4", sensor_amount);
            free(bst_sensorlist.list);
            return 0;
        }

        for (i = 0, j = 0; i < SENSORLIST_INX_END; i++, avail_sens_regval >>= 1)
        {
            if (0x0 == avail_sens_regval)
            {
                break;
            }

            if (avail_sens_regval & 0x1)
            {
                memcpy(&(bst_sensorlist.list[j]), &(bst_all_sensors[i]), sizeof(struct sensor_t));
                bst_sensorlist.bsx_list_index[j] = i;
                j++;
            }
        }

        bst_sensorlist.list_len = sensor_amount;
    }

    *p_sSensorList = bst_sensorlist.list;
    return bst_sensorlist.list_len;
}

static inline int32_t BMI160_convert_ODR(int32_t bsx_list_inx, float Hz)
{
    if (SENSORLIST_INX_ACCELEROMETER == bsx_list_inx)
    {

        if (Hz > 200)
        {
            return BMI160_ACCEL_ODR_400HZ;
        }
        if (Hz > 100 && Hz <= 200)
        {
            return BMI160_ACCEL_ODR_200HZ;
        }
        if (Hz > 50 && Hz <= 100)
        {
            return BMI160_ACCEL_ODR_100HZ;
        }
        if (Hz > 25 && Hz <= 50)
        {
            return BMI160_ACCEL_ODR_50HZ;
        }
        if (Hz > 12.5 && Hz <= 25)
        {
            return BMI160_ACCEL_ODR_25HZ;
        }
        if (Hz > 6.25 && Hz <= 12.5)
        {
            return BMI160_ACCEL_ODR_12_5HZ;
        }
        if (Hz > 1 && Hz <= 6.25)
        {
            return BMI160_ACCEL_ODR_6_25HZ;
        }
        if (Hz > 0 && Hz <= 1)
        {
            return BMI160_ACCEL_ODR_0_78HZ;
        }

        return BMI160_ACCEL_ODR_RESERVED;
    }
    else if (SENSORLIST_INX_GYROSCOPE_UNCALIBRATED == bsx_list_inx)
    {

        if (Hz > 200)
        {
            return BMI160_GYRO_ODR_400HZ;
        }
        if (Hz > 100 && Hz <= 200)
        {
            return BMI160_GYRO_ODR_200HZ;
        }
        if (Hz > 50 && Hz <= 100)
        {
            return BMI160_GYRO_ODR_100HZ;
        }
        if (Hz > 25 && Hz <= 50)
        {
            return BMI160_GYRO_ODR_50HZ;
        }
        if (Hz > 0 && Hz <= 25)
        {
            return BMI160_GYRO_ODR_25HZ;
        }

        return BMI160_GYRO_ODR_RESERVED;
    }
    else if (SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED == bsx_list_inx)
    {

        if (Hz > 200)
        {
            return BMI160_MAG_ODR_400HZ;
        }
        if (Hz > 100 && Hz <= 200)
        {
            return BMI160_MAG_ODR_200HZ;
        }
        if (Hz > 50 && Hz <= 100)
        {
            return BMI160_MAG_ODR_100HZ;
        }
        if (Hz > 25 && Hz <= 50)
        {
            return BMI160_MAG_ODR_50HZ;
        }
        if (Hz > 12.5 && Hz <= 25)
        {
            return BMI160_MAG_ODR_25HZ;
        }
        if (Hz > 6.25 && Hz <= 12.5)
        {
            return BMI160_MAG_ODR_12_5HZ;
        }
        if (Hz > 1 && Hz <= 6.25)
        {
            return BMI160_MAG_ODR_6_25HZ;
        }
        if (Hz > 0 && Hz <= 1)
        {
            return BMI160_MAG_ODR_0_78HZ;
        }

        return BMI160_MAG_ODR_RESERVED;
    }

    return 0;
}

/**
 * @param Hz
 * @param p_bandwith
 * @return physically support Hz
 */
static inline float BMA2x2_convert_ODR(float Hz, int32_t *p_bandwith)
{
    if (Hz > 200)
    {
        *p_bandwith = BMA2X2_ODR_500HZ;
        return 500.f;
    }
    if (Hz > 100 && Hz <= 200)
    {
        *p_bandwith = BMA2X2_ODR_250HZ;
        return 250.f;
    }
    if (Hz > 50 && Hz <= 100)
    {
        *p_bandwith = BMA2X2_ODR_125HZ;
        return 125.f;
    }
    if (Hz > 25 && Hz <= 50)
    {
        *p_bandwith = BMA2X2_ODR_62_50HZ;
        return 62.5f;
    }
    if (Hz > 12.5 && Hz <= 25)
    {
        *p_bandwith = BMA2X2_ODR_31_25HZ;
        return 31.25f;
    }
    if (Hz > 6.25 && Hz <= 12.5)
    {
        *p_bandwith = BMA2X2_ODR_15_63HZ;
        return 15.63f;
    }
    if (Hz > 1 && Hz <= 6.25)
    {
        *p_bandwith = BMA2X2_ODR_15_63HZ;
        return 15.63f;
    }

    return 0;

}

/**
 * @param Hz
 * @param p_bandwith
 * @return physical support Hz
 */
static inline float BMG160_convert_ODR(float Hz, int32_t *p_bandwith)
{
    if (Hz > 200)
    {
        *p_bandwith = BMG160_ODR_400HZ;
        return 400.f;
    }
    if (Hz > 100 && Hz <= 200)
    {
        *p_bandwith = BMG160_ODR_200HZ;
        return 200.f;
    }
    if (Hz > 1 && Hz <= 100)
    {
        *p_bandwith = BMG160_ODR_100HZ;
        return 100.f;
    }

    return 0;

}


static inline int32_t ap_convert_latency(
        uint16_t alshconf_maxlatency,
        uint8_t alshconf_latencyunit,
        int32_t acc_odr, int32_t gyro_odr, int32_t magn_odr)
{

#if 0
    uint64_t maxlatency_ns = 0;

    switch(alshconf_latencyunit)
    {
        case ALSH_CONFSTR_UNITns:
        maxlatency_ns = (uint64_t)alshconf_maxlatency;
        break;
        case ALSH_CONFSTR_UNITus:
        maxlatency_ns = (uint64_t)alshconf_maxlatency * 1000;
        break;
        case ALSH_CONFSTR_UNITms:
        maxlatency_ns = (uint64_t)alshconf_maxlatency * 1000000;
        break;
        case ALSH_CONFSTR_UNITs:
        maxlatency_ns = (uint64_t)alshconf_maxlatency * 1000000000;
        break;
    }

    //caculate out water mark
    if(maxlatency_ns < xxx)
    if(acc_odr .., gyro_odr .., magn_odr ..)
    return xxx
    else if(maxlatency_ns < xxx)
    if(acc_odr .., gyro_odr .., magn_odr ..)
    return xxx
#else
    (void) alshconf_maxlatency;
    (void) alshconf_latencyunit;
    (void) acc_odr;
    (void) gyro_odr;
    (void) magn_odr;

    return default_watermark;
#endif
}


static int32_t is_acc_open = 0;
static int32_t is_gyr_open = 0;
static int32_t is_mag_open = 0;

static int poll_timer_fd = -1;
static int acc_poll_mulp = 0;
static int gyr_poll_mulp = 0;
static int mag_poll_mulp = 0;
static int acc_poll_cnt = 0;
static int gyr_poll_cnt = 0;
static int mag_poll_cnt = 0;
#define MIN_TIMER_INTERVAL_ms 20
#define SET_POLL_PERIOD(period_ms, poll_mulp, poll_cnt) \
{\
    poll_mulp = period_ms / MIN_TIMER_INTERVAL_ms;\
    poll_cnt = 1;\
}

static int64_t bma2x2_poll_start_tm = 0;
static int64_t bma2x2_last_frm_tm = 0;
static uint32_t bma2x2_total_frm = 0;
static int32_t bma2x2_if_start_up = 0;
static float bma2x2_odr_set = 0;
#define BMA2x2_RESET_TMESTIMATE(odr_set) \
{\
    bma2x2_poll_start_tm = 0;\
    bma2x2_last_frm_tm = 0;\
    bma2x2_total_frm = 0;\
    bma2x2_if_start_up = 0;\
    bma2x2_odr_set = odr_set;\
}
static int32_t bma2x2_expected_frms = 5;

static int32_t bma2x2_raw_cnt = 0;
static int32_t bma2x2_pre_raw[3] = {0};
static int64_t bma2x2_pre_tm = 0;
#define BMA2x2_RESET_RESAMPLE() \
{\
    bma2x2_raw_cnt = 0;\
    bma2x2_pre_raw[0] = 0;\
    bma2x2_pre_raw[1] = 0;\
    bma2x2_pre_raw[2] = 0;\
    bma2x2_pre_tm = 0;\
}

static int64_t bmg160_poll_start_tm = 0;
static int64_t bmg160_last_frm_tm = 0;
static uint32_t bmg160_total_frm = 0;
static int32_t bmg160_if_start_up = 0;
static float bmg160_odr_set = 0;
#define BMG160_RESET_TMESTIMATE(odr_set) \
{\
    bmg160_poll_start_tm = 0;\
    bmg160_last_frm_tm = 0;\
    bmg160_total_frm = 0;\
    bmg160_if_start_up = 0;\
    bmg160_odr_set = odr_set;\
}
static int32_t bmg160_expected_frms = 4;

static int64_t mag_polltm_hist_buf[5] = {0};
static int32_t mag_polltm_hist_cnt = 0;
static int64_t mag_poll_starttm = 0;
#define MAG_POLLTM_HIST_CLN() \
{\
    memset(mag_polltm_hist_buf, 0, sizeof(mag_polltm_hist_buf));\
    mag_polltm_hist_cnt = 0;\
    mag_poll_starttm = 0;\
}

#define ALIGN_BUF_MAXLEN 24
#define HIST_GYR_MAX 4
static HW_DATA_UNION *p_BMA2x2smpl_alignbuf[ALIGN_BUF_MAXLEN] = {0};
static uint32_t BMA2x2smpl_alignbuf_len = 0;
static HW_DATA_UNION *p_BMG160smpl_alignbuf[ALIGN_BUF_MAXLEN] = {0};
static uint32_t BMG160smpl_alignbuf_len = 0;
static HW_DATA_UNION *p_BMM150smpl_alignbuf[ALIGN_BUF_MAXLEN] = {0};
static uint32_t BMM150smpl_alignbuf_len = 0;
static void BMI055_tm_alignbuf_clean()
{
    uint32_t i;
    for (i = 0; i < BMA2x2smpl_alignbuf_len; ++i) {
        free(p_BMA2x2smpl_alignbuf[i]);
        p_BMA2x2smpl_alignbuf[i] = NULL;
    }
    BMA2x2smpl_alignbuf_len = 0;

    for (i = 0; i < BMG160smpl_alignbuf_len; ++i) {
        free(p_BMG160smpl_alignbuf[i]);
        p_BMG160smpl_alignbuf[i] = NULL;
    }
    BMG160smpl_alignbuf_len = 0;

    for (i = 0; i < BMM150smpl_alignbuf_len; ++i) {
        free(p_BMM150smpl_alignbuf[i]);
        p_BMM150smpl_alignbuf[i] = NULL;
    }
    BMM150smpl_alignbuf_len = 0;
}


static void ap_config_phyACC(bsx_f32_t sample_rate)
{
    int32_t ret = 0;
    int32_t odr_Hz;
    int32_t bandwidth = 0;
    int32_t fifo_data_sel_regval;
    float physical_Hz = 0;
    uint32_t period_ms = 0;
    static float pre_poll_rate = 0;

    PINFO("set physical ACC rate %f", sample_rate);

    if(ACC_CHIP_BMI160 == accl_chip)
    {
        ret = rd_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, &fifo_data_sel_regval);
        if (0 != ret)
        {
            PERR("read fifo_data_sel fail, ret = %d, set fifo_data_sel_regval = 0", ret);
            /*Keep on trying*/
            fifo_data_sel_regval = 0;
        }

        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_acc_open)
            {
                PDEBUG("shutdown acc");
                /** when closing in BMI160, set op mode firstly. */
                ret = wr_sysfs_oneint("acc_op_mode", iio_dev0_dir_name, SENSOR_PM_SUSPEND);
                fifo_data_sel_regval &= ~(1 << 0);
                wr_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, fifo_data_sel_regval);

                is_acc_open = 0;
            }
        }else
        {
            PDEBUG("set acc odr: %f", sample_rate);
            odr_Hz = BMI160_convert_ODR(SENSORLIST_INX_ACCELEROMETER, sample_rate);
            ret = wr_sysfs_oneint("acc_odr", iio_dev0_dir_name, odr_Hz);

            /*activate is included*/
            if (0 == is_acc_open)
            {
                /** when opening in BMI160, set op mode at last. */
                fifo_data_sel_regval |= (1 << 0);
                wr_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, fifo_data_sel_regval);
                ret = wr_sysfs_oneint("acc_op_mode", iio_dev0_dir_name, SENSOR_PM_NORMAL);

                is_acc_open = 1;
            }
        }
    }else if(ACC_CHIP_BMA2x2 == accl_chip)
    {
        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_acc_open)
            {
                PDEBUG("shutdown acc");

                pre_poll_rate = 0;

                ret = wr_sysfs_oneint("fifo_mode", acc_input_dir_name, BMA2x2_FIFO_PASSBY);
                ret = wr_sysfs_oneint("op_mode", acc_input_dir_name, SENSOR_PM_SUSPEND);

                is_acc_open = 0;
            }
        }else
        {
            PDEBUG("set acc odr: %f", sample_rate);

            physical_Hz = BMA2x2_convert_ODR(sample_rate, &bandwidth);
            ret = wr_sysfs_oneint("bandwidth", acc_input_dir_name, bandwidth);

            /*activate is included*/
            if (0 == is_acc_open)
            {
                ret = wr_sysfs_oneint("op_mode", acc_input_dir_name, SENSOR_PM_NORMAL);
                ret = wr_sysfs_oneint("fifo_mode", acc_input_dir_name, BMA2x2_FIFO_STREAM);

                is_acc_open = 1;
            }

            if(pre_poll_rate != physical_Hz)
            {
                /*For polled samples, duplicate timer set and tm_est reset may impact the performance*/
                BMA2x2_RESET_TMESTIMATE(physical_Hz);
                BMA2x2_RESET_RESAMPLE();

                period_ms = (uint32_t)(bma2x2_expected_frms * (1000.0f / physical_Hz));
                SET_POLL_PERIOD(period_ms, acc_poll_mulp, acc_poll_cnt);
                pre_poll_rate = physical_Hz;
            }
        }

    }

    return;
}

static void ap_config_phyGYR(bsx_f32_t sample_rate)
{
    int32_t ret = 0;
    int32_t odr_Hz;
    int32_t bandwidth = 0;
    int32_t fifo_data_sel_regval;
    float physical_Hz = 0;
    uint32_t period_ms = 0;
    static float pre_poll_rate = 0;

    PINFO("set physical GYRO rate %f", sample_rate);

    if(GYR_CHIP_BMI160 == gyro_chip)
    {
        ret = rd_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, &fifo_data_sel_regval);
        if (0 != ret)
        {
            PERR("read fifo_data_sel fail, ret = %d, set fifo_data_sel_regval = 0", ret);
            /*Keep on trying*/
            fifo_data_sel_regval = 0;
        }

        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_gyr_open)
            {
                PDEBUG("shutdown gyro");
                /** when closing in BMI160, set op mode firstly. */
                ret = wr_sysfs_oneint("gyro_op_mode", iio_dev0_dir_name, SENSOR_PM_SUSPEND);
                fifo_data_sel_regval &= ~(1 << 1);
                wr_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, fifo_data_sel_regval);

                is_gyr_open = 0;
            }
        }else
        {
            PDEBUG("set gyr odr: %f", sample_rate);
            odr_Hz = BMI160_convert_ODR(SENSORLIST_INX_GYROSCOPE_UNCALIBRATED, sample_rate);
            ret = wr_sysfs_oneint("gyro_odr", iio_dev0_dir_name, odr_Hz);

            /*activate is included*/
            if (0 == is_gyr_open)
            {
                /** when opening in BMI160, set op mode at last. */
                fifo_data_sel_regval |= (1 << 1);
                wr_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, fifo_data_sel_regval);
                ret = wr_sysfs_oneint("gyro_op_mode", iio_dev0_dir_name, SENSOR_PM_NORMAL);

                is_gyr_open = 1;
            }
        }
    }else if(GYR_CHIP_BMG160 == gyro_chip)
    {
        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_gyr_open)
            {
                PDEBUG("shutdown gyro");

                pre_poll_rate = 0;

                ret = wr_sysfs_oneint("fifo_mode", gyr_input_dir_name, BMG160_FIFO_PASSBY);
                ret = wr_sysfs_oneint("op_mode", gyr_input_dir_name, SENSOR_PM_SUSPEND);

                is_gyr_open = 0;
            }
        }else
        {
            PDEBUG("set gyr odr: %f", sample_rate);

            physical_Hz = BMG160_convert_ODR(sample_rate, &bandwidth);
            ret = wr_sysfs_oneint("bandwidth", gyr_input_dir_name, bandwidth);

            /*activate is included*/
            if (0 == is_gyr_open)
            {
                ret = wr_sysfs_oneint("op_mode", gyr_input_dir_name, SENSOR_PM_NORMAL);
                ret = wr_sysfs_oneint("fifo_mode", gyr_input_dir_name, BMG160_FIFO_STREAM);

                is_gyr_open = 1;
            }

            if(pre_poll_rate != physical_Hz)
            {
                /*For polled samples, duplicate timer set and tm_est reset may impact the performance*/
                BMG160_RESET_TMESTIMATE(physical_Hz);

                period_ms = (uint32_t)(bmg160_expected_frms * (1000.0f / physical_Hz));
                SET_POLL_PERIOD(period_ms, gyr_poll_mulp, gyr_poll_cnt);
                pre_poll_rate = physical_Hz;
            }
        }

    }

    return;
}

static void ap_config_phyMAG(bsx_f32_t sample_rate)
{
    int32_t ret = 0;
    int32_t odr_Hz;
    int32_t fifo_data_sel_regval;
    uint32_t Hz_to_delay_ms = 0;
    struct itimerspec timerspec;
    static bsx_f32_t pre_poll_rate = 0;

    PINFO("set physical MAG rate %f", sample_rate);

    if(MAG_CHIP_BMI160 == magn_chip)
    {
        ret = rd_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, &fifo_data_sel_regval);
        if (0 != ret)
        {
            PERR("read fifo_data_sel fail, ret = %d, set fifo_data_sel_regval = 0", ret);
            /*Keep on trying*/
            fifo_data_sel_regval = 0;
        }

        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_mag_open)
            {
                PDEBUG("shutdown magn");
                /** when closing in BMI160, set op mode firstly. */
                ret = wr_sysfs_oneint("mag_op_mode", iio_dev0_dir_name, SENSOR_PM_SUSPEND);
                fifo_data_sel_regval &= ~(1 << 2);
                wr_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, fifo_data_sel_regval);

                is_mag_open = 0;
            }
        }else
        {
            PDEBUG("set magn odr: %f", sample_rate);
            odr_Hz = BMI160_convert_ODR(SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED, sample_rate);
            ret = wr_sysfs_oneint("mag_odr", iio_dev0_dir_name, odr_Hz);

            /*activate is included*/
            if (0 == is_mag_open)
            {
                /** when opening in BMI160, set op mode at last. */
                fifo_data_sel_regval |= (1 << 2);
                wr_sysfs_oneint("fifo_data_sel", iio_dev0_dir_name, fifo_data_sel_regval);
                ret = wr_sysfs_oneint("mag_op_mode", iio_dev0_dir_name, SENSOR_PM_NORMAL);

                is_mag_open = 1;
            }
        }
    }else if(MAG_CHIP_BMM150 == magn_chip || MAG_CHIP_AKM09912 ==  magn_chip || MAG_CHIP_AKM09911 ==  magn_chip ||
            MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip)
    {
        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_mag_open)
            {
                PDEBUG("shutdown magn");

                if(SOLUTION_MDOF == solution_type && ACC_CHIP_BMI160 == accl_chip && GYR_CHIP_BMI160 == gyro_chip)
                {
                    timerspec.it_value.tv_sec = 0;
                    timerspec.it_value.tv_nsec = 0;
                    timerspec.it_interval.tv_sec = 0; /*is needed, please ignore the man info*/
                    timerspec.it_interval.tv_nsec = 0;/*is needed, please ignore the man info*/
                    ret = timerfd_settime(poll_timer_fd, 0, &timerspec, NULL);
                }

                pre_poll_rate = 0;

                ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_SUSPEND);

                is_mag_open = 0;
            }
        }else
        {
            PDEBUG("set magn odr: %f", sample_rate);

            /*activate is included*/
            if (0 == is_mag_open)
            {
                ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_NORMAL);

                if(MAG_CHIP_BMM150 == magn_chip){
                    /*set filter parameter to reduce noise.
                     * only can do on our BMM but open this to customer on AKM*/
                    ret = wr_sysfs_oneint("rept_xy", mag_input_dir_name, 4);
                    ret = wr_sysfs_oneint("rept_z", mag_input_dir_name, 15);
                }

                is_mag_open = 1;
            }

            if(pre_poll_rate != sample_rate)
            {
                /*For polled samples, duplicate timer set and tm_est reset may impact the performance*/
                MAG_POLLTM_HIST_CLN();

                Hz_to_delay_ms = (uint32_t)(1000.0f / sample_rate);
                if(SOLUTION_MDOF == solution_type && ACC_CHIP_BMI160 == accl_chip && GYR_CHIP_BMI160 == gyro_chip)
                {
                    timerspec.it_value.tv_sec = Hz_to_delay_ms / 1000;
                    timerspec.it_value.tv_nsec = (Hz_to_delay_ms % 1000) * 1000000;
                    timerspec.it_interval.tv_sec = timerspec.it_value.tv_sec;
                    timerspec.it_interval.tv_nsec = timerspec.it_value.tv_nsec;
                    ret = timerfd_settime(poll_timer_fd, 0, &timerspec, NULL);
                }else
                {
                    SET_POLL_PERIOD(Hz_to_delay_ms, mag_poll_mulp, mag_poll_cnt);
                }

                pre_poll_rate = sample_rate;
            }
        }
    }

    return;
}


static void ap_config_physensor(bsx_u32_t input_id, bsx_f32_t sample_rate)
{
    int32_t ret = 0;

    if (BSX_PHYSICAL_SENSOR_ID_INVALID == input_id)
    { //TODO: library may has bug
        return;
    }

    switch (input_id)
    {
        case BSX_INPUT_ID_ACCELERATION:
            ap_config_phyACC(sample_rate);
            break;
        case BSX_INPUT_ID_MAGNETICFIELD:
            ap_config_phyMAG(sample_rate);
            break;
        case BSX_INPUT_ID_ANGULARRATE:
            ap_config_phyGYR(sample_rate);
            break;
        default:
            PWARN("unknown input id: %d", input_id);
            ret = 0;
            break;
    }

    if (ret < 0)
    {
        PERR("write_sysfs() fail");
    }

    return;
}



/**
 *
 */
static void ap_send_config(int32_t bsx_list_inx)
{
    const struct sensor_t *p_sensor;
    BSX_SENSOR_CONFIG *p_config;
    bsx_sensor_configuration_t bsx_config_output[2];
    int32_t bsx_supplier_id;
    int32_t list_inx_base;
    bsx_u32_t input_id;

    if (bsx_list_inx <= SENSORLIST_INX_AMBIENT_IAQ)
    {
        list_inx_base = SENSORLIST_INX_GAS_RESIST;
        p_config = BSX_sensor_config_nonwk;
    }
    else
    {
        list_inx_base = SENSORLIST_INX_WAKEUP_SIGNI_PRESSURE;
        p_config = BSX_sensor_config_wk;
    }

    bsx_supplier_id = convert_BSX_ListInx(bsx_list_inx);
    if (BSX_VIRTUAL_SENSOR_ID_INVALID == bsx_supplier_id)
    {
        PWARN("invalid list index: %d, when matching supplier id", bsx_list_inx);
        return;
    }

    {
        bsx_config_output[0].sensor_id = bsx_supplier_id;

        p_sensor = &(bst_all_sensors[bsx_list_inx]);
        if(SENSOR_FLAG_ON_CHANGE_MODE == (p_sensor->flags & REPORTING_MODE_MASK))
        {
            bsx_config_output[0].sample_rate = p_config[bsx_list_inx - list_inx_base].delay_onchange_Hz;
        }else
        {
            CONVERT_DATARATE_CODE(p_config[bsx_list_inx - list_inx_base].data_rate, bsx_config_output[0].sample_rate);
        }


        /**
         * For test app, AR should be defined as a on change sensor,
         * But for BSX4 Library, only can send rate 0 on AR
         */
        if (SENSORLIST_INX_WAKEUP_ACTIVITY == bsx_list_inx)
        {
            bsx_config_output[0].sample_rate = 0;
        }
    }

    if(0 == active_nonwksensor_cnt + active_wksensor_cnt){
        BMI055_tm_alignbuf_clean();
    }

    {
        switch (bsx_list_inx) {
            case SENSORLIST_INX_ACCELEROMETER:
                input_id = BSX_INPUT_ID_ACCELERATION;
                break;
            case SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED:
                input_id = BSX_INPUT_ID_MAGNETICFIELD;
                break;
            case SENSORLIST_INX_GYROSCOPE_UNCALIBRATED:
                input_id = BSX_INPUT_ID_ANGULARRATE;
                break;
            default:
                PERR("wrong list index: %d", bsx_list_inx);
                return;
        }

        ap_config_physensor(input_id, bsx_config_output[0].sample_rate);
    }

    return;
}

/**
 *
 * @param bsx_list_inx
 */
static void ap_send_disable_config(int32_t bsx_list_inx)
{
    int32_t bsx_supplier_id;
    bsx_u32_t input_id;

    bsx_supplier_id = convert_BSX_ListInx(bsx_list_inx);
    if (BSX_VIRTUAL_SENSOR_ID_INVALID == bsx_supplier_id)
    {
        PWARN("invalid list index: %d, when matching supplier id", bsx_list_inx);
        return;
    }

    {
        switch (bsx_list_inx) {
            case SENSORLIST_INX_ACCELEROMETER:
                input_id = BSX_INPUT_ID_ACCELERATION;
                break;
            case SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED:
                input_id = BSX_INPUT_ID_MAGNETICFIELD;
                break;
            case SENSORLIST_INX_GYROSCOPE_UNCALIBRATED:
                input_id = BSX_INPUT_ID_ANGULARRATE;
                break;
            default:
                PERR("wrong list index: %d", bsx_list_inx);
                return;
        }

        ap_config_physensor(input_id, SAMPLE_RATE_DISABLED);
    }
    return;
}

int32_t ap_activate(int32_t handle, int32_t enabled)
{
    struct sensor_t *p_sensor;
    int32_t bsx_list_inx;
    int32_t ret;

    if (BSX_SENSOR_ID_INVALID == handle)
    {
        /*private sensors which not supported in library and Android yet*/
        return 0;
    }

    PDEBUG("ap_activate(handle: %d, enabled: %d)", handle, enabled);

    get_sensor_t(handle, &p_sensor, &bsx_list_inx);
    if (NULL == p_sensor)
    {
        PWARN("invalid id: %d", handle);
        return -EINVAL;
    }

    /*To adapt BSX4 algorithm's way of configuration string, activate_configref_resort() is employed*/
    ret = activate_configref_resort(bsx_list_inx, enabled);
    if (ret)
    {
        if (enabled)
        {
            ap_send_config(bsx_list_inx);
        }
        else
        {
            ap_send_disable_config(bsx_list_inx);
        }
    }

    return 0;
}

int32_t ap_batch(int32_t handle, int32_t flags, int64_t sampling_period_ns, int64_t max_report_latency_ns)
{

    (void) flags; //Deprecated in SENSORS_DEVICE_API_VERSION_1_3
    struct sensor_t *p_sensor;
    int32_t bsx_list_inx;
    int32_t ret;
    float delay_Hz_onchange = 0;

    //Sensors whoes fifoMaxEventCount==0 can be called batching according to spec
    get_sensor_t(handle, &p_sensor, &bsx_list_inx);
    if (NULL == p_sensor)
    {
        PWARN("invalid id: %d", handle);
        /*If maxReportingLatency is set to 0, this function must succeed*/
        if (0 == max_report_latency_ns)
        {
            return 0;
        }
        else
        {
            return -EINVAL;
        }
    }

    /*One-shot sensors are sometimes referred to as trigger sensors.
     The sampling_period_ns and max_report_latency_ns parameters
     passed to the batch function is ignored.
     Events from one-shot events cannot be stored in hardware FIFOs:
     the events must be reported as soon as they are generated.
     */
    if (SENSOR_FLAG_ONE_SHOT_MODE == (p_sensor->flags & REPORTING_MODE_MASK))
    {
        return 0;
    }

    /*For special reporting-mode sensors, process each one according to spec*/
    if (SENSOR_FLAG_SPECIAL_REPORTING_MODE == (p_sensor->flags & REPORTING_MODE_MASK))
    {
        //...
        return 0;
    }

    PDEBUG("batch(handle: %d, sampling_period_ns = %lld)", handle, sampling_period_ns);

    /*
     For continuous and on-change sensors
     1. if sampling_period_ns is less than sensor_t.minDelay,
     then the HAL implementation must silently clamp it to max(sensor_t.minDelay, 1ms).
     Android does not support the generation of events at more than 1000Hz.
     2. if sampling_period_ns is greater than sensor_t.maxDelay,
     then the HAL implementation must silently truncate it to sensor_t.maxDelay.
     */
    if (sampling_period_ns < (int64_t)(p_sensor->minDelay) * 1000LL)
    {
        sampling_period_ns = (int64_t)(p_sensor->minDelay) * 1000LL;
    }
    else if (sampling_period_ns > (int64_t)(p_sensor->maxDelay) * 1000LL)
    {
        sampling_period_ns = (int64_t)(p_sensor->maxDelay) * 1000LL;
    }

    /**
     * store sampling period as delay(us) for on change type sensor
     */
    if(SENSOR_FLAG_ON_CHANGE_MODE == (p_sensor->flags & REPORTING_MODE_MASK))
    {
        delay_Hz_onchange = (float)1000000000LL / (float)sampling_period_ns;
        sampling_period_ns = 0;
    }

    /*In Android's perspective, a sensor can be configured no matter if it's active.
     To adapt BSX4 algorithm's way of configuration string, batch_configref_resort() is employed*/
    ret = batch_configref_resort(bsx_list_inx, sampling_period_ns, max_report_latency_ns, delay_Hz_onchange);
    if (ret)
    {
        ap_send_config(bsx_list_inx);
    }

    return 0;
}

int32_t ap_flush(BstSensor *bstsensor, int32_t handle)
{
    struct sensor_t *p_sensor;
    int32_t bsx_list_inx;

    get_sensor_t(handle, &p_sensor, &bsx_list_inx);
    if (NULL == p_sensor)
    {
        PWARN("invalid id: %d", handle);
        return -EINVAL;
    }

    /*flush does not apply to one-shot sensors:
     if sensor_handle refers to a one-shot sensor,
     flush must return -EINVAL and not generate any flush complete metadata event.
     */
    if (SENSOR_FLAG_ONE_SHOT_MODE == (p_sensor->flags & REPORTING_MODE_MASK))
    {
        PWARN("invalid flags for id: %d", handle);
        return -EINVAL;
    }


    //Now the driver can not support this specification, so has to work around
    (void) bstsensor->send_flush_event(handle);

    /*If the specified sensor has no FIFO (no buffering possible),
     or if the FIFO was empty at the time of the call,
     flush must still succeed and send a flush complete event for that sensor.
     This applies to all sensors
     (except one-shot sensors, and already filted in former code).
     */

    return 0;
}


static void ap_hw_process_IIO_frame(int32_t header, char *data, BstSimpleList *dest_list)
{

    HW_DATA_UNION *p_hwdata;
    int16_t rawdata_tmp = 0;
    int64_t raw_timestamp = 0;
    int32_t ret;

    p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata)
    {
        PERR("malloc fail");
        return;
    }

    /*
     size_from_channelarray bytes 1 name =in_x location=0 bytes=2
     size_from_channelarray bytes 1 name =in_y location=2 bytes=2
     size_from_channelarray bytes 1 name =in_z location=4 bytes=2
     size_from_channelarray bytes 1 name =in_timestamp location=8 bytes=8
     size_from_channelarray bytes=16
     */

    switch (header)
    {
        case FIFO_HEAD_A:
            p_hwdata->id = SENSOR_TYPE_ACCELEROMETER;
            memcpy(&rawdata_tmp, data, sizeof(rawdata_tmp));
            p_hwdata->x = (float) rawdata_tmp * BMI160_acc_resl;

            memcpy(&rawdata_tmp, data + 2, sizeof(rawdata_tmp));
            p_hwdata->y = (float) rawdata_tmp * BMI160_acc_resl;

            memcpy(&rawdata_tmp, data + 4, sizeof(rawdata_tmp));
            p_hwdata->z = (float) rawdata_tmp * BMI160_acc_resl;

            memcpy(&raw_timestamp, data + 8, sizeof(raw_timestamp));
            p_hwdata->timestamp = raw_timestamp * 1000;

            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_a);

            ret = dest_list->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }

            break;
        case FIFO_HEAD_G:
            p_hwdata->id = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;

            memcpy(&rawdata_tmp, data, sizeof(rawdata_tmp));
            p_hwdata->x_uncalib = (float) rawdata_tmp;

            memcpy(&rawdata_tmp, data + 2, sizeof(rawdata_tmp));
            p_hwdata->y_uncalib = (float) rawdata_tmp;

            memcpy(&rawdata_tmp, data + 4, sizeof(rawdata_tmp));
            p_hwdata->z_uncalib = (float) rawdata_tmp;

            memcpy(&raw_timestamp, data + 8, sizeof(raw_timestamp));
            p_hwdata->timestamp = raw_timestamp * 1000;

            hw_remap_sensor_data(&(p_hwdata->x_uncalib), &(p_hwdata->y_uncalib), &(p_hwdata->z_uncalib), g_place_g);

            ret = dest_list->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }

            break;
        case FIFO_HEAD_M:
            p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;

            memcpy(&rawdata_tmp, data, sizeof(rawdata_tmp));
            p_hwdata->x_uncalib = (float) rawdata_tmp * 10 / BMM_COMPVAL_TO_uT; // library requires input is in 0.1uT

            memcpy(&rawdata_tmp, data + 2, sizeof(rawdata_tmp));
            p_hwdata->y_uncalib = (float) rawdata_tmp * 10 / BMM_COMPVAL_TO_uT;

            memcpy(&rawdata_tmp, data + 4, sizeof(rawdata_tmp));
            p_hwdata->z_uncalib = (float) rawdata_tmp * 10 / BMM_COMPVAL_TO_uT;

            memcpy(&raw_timestamp, data + 8, sizeof(raw_timestamp));
            p_hwdata->timestamp = raw_timestamp * 1000;

            hw_remap_sensor_data(&(p_hwdata->x_uncalib), &(p_hwdata->y_uncalib), &(p_hwdata->z_uncalib), g_place_m);

            ret = dest_list->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }

            break;
        default:
            PERR("Unknown header %d", header);
            free(p_hwdata);

            break;
    }

    return;
}

static void ap_hw_poll_bma2x2(BstSimpleList *dest_list)
{
    int32_t ret;
    int32_t fifo_fd = -1;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    uint8_t fifo_data_buf[32*3*2] = {0}; //32 frames, each frame contains 2Bytes X, 2Bytes Y and 2Bytes Z
    HW_DATA_UNION *p_hwdata;
    int16_t X_tmp = 0;
    int16_t Y_tmp = 0;
    int16_t Z_tmp = 0;
    struct timespec tmspec;
    int64_t sys_tm = 0;
    int32_t frames = 0;
    int32_t i;
    int64_t average_interval;
    int8_t resample_valid = 0;
    int32_t to_resample_raw[3] = {0};
    int64_t to_resample_tm = 0;


    clock_gettime(CLOCK_MONOTONIC, &tmspec);
    sys_tm = tmspec.tv_sec * 1000000000 + tmspec.tv_nsec;

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", acc_input_dir_name, "fifo_data_frame");
    }

    fifo_fd = open(fname_buf, O_RDONLY | O_NONBLOCK);
    if(-1 == fifo_fd){
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    ret = read(fifo_fd, fifo_data_buf, sizeof(fifo_data_buf));
    close(fifo_fd);
    if(ret < 0 ){
        PERR("read %d fail: ret = %d, errno = %d(%s)", fifo_fd, ret, errno, strerror(errno));
        return;
    }

    frames = ret/6;/*2BytesX 2BytesY 2BytesZ*/
    if(0 == frames)
    {
        return;
    }

    /*there's a start up time needed for BMA2x2*/
    if(0 == bma2x2_if_start_up)
    {
        /*update time stamp estimation base line*/
        bma2x2_poll_start_tm = sys_tm;
        bma2x2_last_frm_tm = bma2x2_poll_start_tm;
        average_interval = (int64_t)(1000000000.f / bma2x2_odr_set);

        for (i = 0; i < frames; ++i)
        {
            /*2 Bytes output from all BMA ASIC, valid bits are located high*/
            X_tmp = fifo_data_buf[i*6+0] | (fifo_data_buf[i*6+1] << 8);
            X_tmp >>= (16 - BMA255_ADC_BITS);
            Y_tmp = fifo_data_buf[i*6+2] | (fifo_data_buf[i*6+3] << 8);
            Y_tmp >>= (16 - BMA255_ADC_BITS);
            Z_tmp = fifo_data_buf[i*6+4] | (fifo_data_buf[i*6+5] << 8);
            Z_tmp >>= (16 - BMA255_ADC_BITS);

            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                continue;
            }

            p_hwdata->id = SENSOR_TYPE_ACCELEROMETER;
            p_hwdata->x = X_tmp * BMA255_acc_resl;
            p_hwdata->y = Y_tmp * BMA255_acc_resl;
            p_hwdata->z = Z_tmp * BMA255_acc_resl;
            p_hwdata->timestamp = sys_tm - ((frames - 1 - i) * average_interval);

            /*resample and deliver*/
            to_resample_raw[0] = (int32_t)p_hwdata->x;
            to_resample_raw[1] = (int32_t)p_hwdata->y;
            to_resample_raw[2] = (int32_t)p_hwdata->z;
            to_resample_tm = p_hwdata->timestamp;

            resample_valid = sensord_resample5to4(to_resample_raw, &to_resample_tm,
                                                    bma2x2_pre_raw, &bma2x2_pre_tm,
                                                    bma2x2_raw_cnt++);
            if(resample_valid && 0 != to_resample_tm)
            {
                /*if(0 == to_resample_tm), it's the first raw data, omit*/
                p_hwdata->x = (float)to_resample_raw[0];
                p_hwdata->y = (float)to_resample_raw[1];
                p_hwdata->z = (float)to_resample_raw[2];
                p_hwdata->timestamp = to_resample_tm;

                hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_a);

                ret = dest_list->list_add_rear((void *) p_hwdata);
                if (ret)
                {
                    PERR("list_add_rear() fail, ret = %d", ret);
                    if(-1 == ret){
                        free(p_hwdata);
                    }
                }
            }else
            {
                free(p_hwdata);
            }
        }

        bma2x2_if_start_up = 1;
        return;
    }

    bma2x2_total_frm += frames; //no matter deliver successfully or not
    average_interval = (sys_tm - bma2x2_poll_start_tm) / bma2x2_total_frm;

    for (i = 0; i < frames; ++i)
    {
        /*2 Bytes output from all BMA ASIC, valid bits are located high*/
        X_tmp = fifo_data_buf[i*6+0] | (fifo_data_buf[i*6+1] << 8);
        X_tmp >>= (16 - BMA255_ADC_BITS);
        Y_tmp = fifo_data_buf[i*6+2] | (fifo_data_buf[i*6+3] << 8);
        Y_tmp >>= (16 - BMA255_ADC_BITS);
        Z_tmp = fifo_data_buf[i*6+4] | (fifo_data_buf[i*6+5] << 8);
        Z_tmp >>= (16 - BMA255_ADC_BITS);

        p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
        if (NULL == p_hwdata)
        {
            PERR("malloc fail");
            continue;
        }

        p_hwdata->id = SENSOR_TYPE_ACCELEROMETER;
        p_hwdata->x = X_tmp * BMA255_acc_resl;
        p_hwdata->y = Y_tmp * BMA255_acc_resl;
        p_hwdata->z = Z_tmp * BMA255_acc_resl;
        p_hwdata->timestamp = bma2x2_last_frm_tm + average_interval;
        bma2x2_last_frm_tm += average_interval;

        if(p_hwdata->timestamp > sys_tm){
            p_hwdata->timestamp = sys_tm;
            bma2x2_last_frm_tm = p_hwdata->timestamp;
        }

        /*resample and deliver*/
        to_resample_raw[0] = (int32_t)p_hwdata->x;
        to_resample_raw[1] = (int32_t)p_hwdata->y;
        to_resample_raw[2] = (int32_t)p_hwdata->z;
        to_resample_tm = p_hwdata->timestamp;

        resample_valid = sensord_resample5to4(to_resample_raw, &to_resample_tm,
                                                bma2x2_pre_raw, &bma2x2_pre_tm,
                                                bma2x2_raw_cnt++);
        if(resample_valid && 0 != to_resample_tm)
        {
            /*if(0 == to_resample_tm), it's the first raw data, omit*/
            p_hwdata->x = (float)to_resample_raw[0];
            p_hwdata->y = (float)to_resample_raw[1];
            p_hwdata->z = (float)to_resample_raw[2];
            p_hwdata->timestamp = to_resample_tm;

            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_a);

            ret = dest_list->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }
        }else
        {
            free(p_hwdata);
        }
    }

    return;
}

static void ap_hw_poll_bmg160(BstSimpleList *dest_list)
{
    int32_t ret;
    int32_t fifo_fd = -1;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    uint8_t fifo_data_buf[100*3*2] = {0}; //100 frames, each frame contains 2Bytes X, 2Bytes Y and 2Bytes Z
    HW_DATA_UNION *p_hwdata;
    int16_t X_tmp = 0;
    int16_t Y_tmp = 0;
    int16_t Z_tmp = 0;
#if defined(PLTF_LINUX_ENABLED)
    struct timespec tmspec;
#endif
    int64_t sys_tm = 0;
    int32_t frames = 0;
    int32_t i;
    int64_t average_interval;

#if !defined(PLTF_LINUX_ENABLED)
    sys_tm = android::elapsedRealtimeNano();
#else
    clock_gettime(CLOCK_MONOTONIC, &tmspec);
    sys_tm = tmspec.tv_sec * 1000000000 + tmspec.tv_nsec;
#endif

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", gyr_input_dir_name, "fifo_data_frame");
    }

    fifo_fd = open(fname_buf, O_RDONLY | O_NONBLOCK);
    if(-1 == fifo_fd){
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    ret = read(fifo_fd, fifo_data_buf, sizeof(fifo_data_buf));
    close(fifo_fd);
    if(ret < 0 ){
        PERR("read %d fail: ret = %d, errno = %d(%s)", fifo_fd, ret, errno, strerror(errno));
        return;
    }

    frames = ret/6;/*2BytesX 2BytesY 2BytesZ*/
    if(0 == frames)
    {
        return;
    }

    /*there's a start up time needed for BMG160*/
    if(0 == bmg160_if_start_up)
    {
        /*update time stamp estimation base line*/
        bmg160_poll_start_tm = sys_tm;
        bmg160_last_frm_tm = bmg160_poll_start_tm;
        average_interval = (int64_t)(1000000000.f / bmg160_odr_set);

        for (i = 0; i < frames; ++i)
        {
            X_tmp = fifo_data_buf[i*6+0] | (fifo_data_buf[i*6+1] << 8);
            Y_tmp = fifo_data_buf[i*6+2] | (fifo_data_buf[i*6+3] << 8);
            Z_tmp = fifo_data_buf[i*6+4] | (fifo_data_buf[i*6+5] << 8);

            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                continue;
            }

            p_hwdata->id = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
            p_hwdata->x_uncalib = X_tmp;
            p_hwdata->y_uncalib = Y_tmp;
            p_hwdata->z_uncalib = Z_tmp;
            p_hwdata->timestamp = sys_tm - ((frames - 1 - i) * average_interval);

            hw_remap_sensor_data(&(p_hwdata->x_uncalib), &(p_hwdata->y_uncalib), &(p_hwdata->z_uncalib), g_place_g);

            ret = dest_list->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }
        }

        bmg160_if_start_up = 1;
        return;
    }


    bmg160_total_frm += frames; //no matter deliver successfully or not
    average_interval = (sys_tm - bmg160_poll_start_tm) / bmg160_total_frm;

    for (i = 0; i < frames; ++i)
    {
        X_tmp = fifo_data_buf[i*6+0] | (fifo_data_buf[i*6+1] << 8);
        Y_tmp = fifo_data_buf[i*6+2] | (fifo_data_buf[i*6+3] << 8);
        Z_tmp = fifo_data_buf[i*6+4] | (fifo_data_buf[i*6+5] << 8);

        p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
        if (NULL == p_hwdata)
        {
            PERR("malloc fail");
            continue;
        }

        p_hwdata->id = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
        p_hwdata->x_uncalib = X_tmp;
        p_hwdata->y_uncalib = Y_tmp;
        p_hwdata->z_uncalib = Z_tmp;
        p_hwdata->timestamp = bmg160_last_frm_tm + average_interval;
        bmg160_last_frm_tm += average_interval;

        if(p_hwdata->timestamp > sys_tm)
        {
            /*BMG160 looks to generate more samples than required.*/
            bmg160_last_frm_tm = sys_tm;
            free(p_hwdata);

            break;
        }else
        {
            hw_remap_sensor_data(&(p_hwdata->x_uncalib), &(p_hwdata->y_uncalib), &(p_hwdata->z_uncalib), g_place_g);

            ret = dest_list->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }
        }
    }

    return;
}

/**
 * @param cur_tm, the latest times tamp got from system API
 * @return = tm_base + (average interval of historical time stamps)
 * tm_base update in every calculation.
 */
static int64_t mag_poll_tmfilt(int64_t cur_tm)
{
    int64_t total_interval = 0;
    int i;

    if( ARRAY_ELEMENTS(mag_polltm_hist_buf) != mag_polltm_hist_cnt)
    {
        mag_polltm_hist_buf[mag_polltm_hist_cnt] = cur_tm;
        mag_polltm_hist_cnt++;
    }else
    {
        memmove(mag_polltm_hist_buf, &mag_polltm_hist_buf[1], sizeof(mag_polltm_hist_buf[0]) * (ARRAY_ELEMENTS(mag_polltm_hist_buf)-1));
        mag_polltm_hist_buf[mag_polltm_hist_cnt - 1] = cur_tm;
    }

    if(1 == mag_polltm_hist_cnt)
    {
        mag_poll_starttm = cur_tm;

    }else
    {
        for (i = 1; i < mag_polltm_hist_cnt; ++i) {
            total_interval += mag_polltm_hist_buf[i] - mag_polltm_hist_buf[i-1];
        }

        mag_poll_starttm += total_interval/(mag_polltm_hist_cnt-1);
    }

    if(mag_poll_starttm > cur_tm){
        /*reduce the drift between estimation time and system time*/
        mag_poll_starttm = cur_tm;
    }

    return mag_poll_starttm;
}

static void ap_hw_poll_bmm150(BstSimpleList *dest_list)
{
    int32_t ret;
    FILE *value_fp = NULL;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    HW_DATA_UNION *p_hwdata;
    int32_t X_tmp = 0;
    int32_t Y_tmp = 0;
    int32_t Z_tmp = 0;
    int64_t sys_tm = 0;
#if defined(PLTF_LINUX_ENABLED)
    struct timespec tmspec;
#endif

#if !defined(PLTF_LINUX_ENABLED)
    sys_tm = android::elapsedRealtimeNano();
#else
    clock_gettime(CLOCK_MONOTONIC, &tmspec);
    sys_tm = tmspec.tv_sec * 1000000000 + tmspec.tv_nsec;
#endif

    //force update mag sample, otherwise it update only in 10Hz
    ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_LP1);

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", mag_input_dir_name, "value");
    }

    value_fp = fopen(fname_buf, "r");
    if (NULL == value_fp)
    {
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    ret = fscanf(value_fp, "%8d %8d %8d", &X_tmp, &Y_tmp, &Z_tmp);
    fclose(value_fp);
    if(ret <= 0){
        PERR("fscanf fail, errno = %d(%s)", errno, strerror(errno));
        return;
    }

    p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata)
    {
        PERR("malloc fail");
        return;
    }

    p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    p_hwdata->x_uncalib = X_tmp * 10 / BMM_COMPVAL_TO_uT; // library requires input is in 0.1uT
    p_hwdata->y_uncalib = Y_tmp * 10 / BMM_COMPVAL_TO_uT;
    p_hwdata->z_uncalib = Z_tmp * 10 / BMM_COMPVAL_TO_uT;
    p_hwdata->timestamp = mag_poll_tmfilt(sys_tm);

    hw_remap_sensor_data(&(p_hwdata->x_uncalib), &(p_hwdata->y_uncalib), &(p_hwdata->z_uncalib), g_place_m);

    ret = dest_list->list_add_rear((void *) p_hwdata);
    if (ret)
    {
        PERR("list_add_rear() fail, ret = %d", ret);
        if(-1 == ret){
            free(p_hwdata);
        }
    }

    return;
}

static void ap_hw_poll_akm099xx(BstSimpleList *dest_list)
{
    int32_t ret;
    FILE *value_fp = NULL;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    HW_DATA_UNION *p_hwdata;
    int32_t X_tmp = 0;
    int32_t Y_tmp = 0;
    int32_t Z_tmp = 0;
    int64_t sys_tm = 0;
#if defined(PLTF_LINUX_ENABLED)
    struct timespec tmspec;
#endif

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", mag_input_dir_name, "value");
    }

    value_fp = fopen(fname_buf, "r");
    if (NULL == value_fp)
    {
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

#if !defined(PLTF_LINUX_ENABLED)
    sys_tm = android::elapsedRealtimeNano();
#else
    clock_gettime(CLOCK_MONOTONIC, &tmspec);
    sys_tm = tmspec.tv_sec * 1000000000 + tmspec.tv_nsec;
#endif

    /*AKM need reset power mode to let it update data register*/
    ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_NORMAL);

    ret = fscanf(value_fp, "%8d %8d %8d", &X_tmp, &Y_tmp, &Z_tmp);
    fclose(value_fp);
    if(ret <= 0){
        PERR("fscanf fail, errno = %d(%s)", errno, strerror(errno));
        return;
    }

    p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata)
    {
        PERR("malloc fail");
        return;
    }

    p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    if(MAG_CHIP_AKM09912 == magn_chip){
        p_hwdata->x_uncalib = X_tmp * 10 * AKM09912_COMPVAL_TO_uT;
        p_hwdata->y_uncalib = Y_tmp * 10 * AKM09912_COMPVAL_TO_uT;
        p_hwdata->z_uncalib = Z_tmp * 10 * AKM09912_COMPVAL_TO_uT;
    }else if(MAG_CHIP_AKM09911 == magn_chip){
        p_hwdata->x_uncalib = X_tmp * 10 * AKM09911_COMPVAL_TO_uT;
        p_hwdata->y_uncalib = Y_tmp * 10 * AKM09911_COMPVAL_TO_uT;
        p_hwdata->z_uncalib = Z_tmp * 10 * AKM09911_COMPVAL_TO_uT;
    }
    p_hwdata->timestamp = mag_poll_tmfilt(sys_tm);

    hw_remap_sensor_data(&(p_hwdata->x_uncalib), &(p_hwdata->y_uncalib), &(p_hwdata->z_uncalib), g_place_m);

    ret = dest_list->list_add_rear((void *) p_hwdata);
    if (ret)
    {
        PERR("list_add_rear() fail, ret = %d", ret);
        if(-1 == ret){
            free(p_hwdata);
        }
    }

    return;
}

static void ap_hw_poll_yas5xx(BstSimpleList *dest_list)
{
    int32_t ret;
    FILE *value_fp = NULL;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    HW_DATA_UNION *p_hwdata;
    int32_t X_tmp = 0;
    int32_t Y_tmp = 0;
    int32_t Z_tmp = 0;
    int64_t sys_tm = 0;
#if defined(PLTF_LINUX_ENABLED)
    struct timespec tmspec;
#endif

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", mag_input_dir_name, "value");
    }

    value_fp = fopen(fname_buf, "r");
    if (NULL == value_fp)
    {
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

#if !defined(PLTF_LINUX_ENABLED)
    sys_tm = android::elapsedRealtimeNano();
#else
    clock_gettime(CLOCK_MONOTONIC, &tmspec);
    sys_tm = tmspec.tv_sec * 1000000000 + tmspec.tv_nsec;
#endif

    /*YAS need reset power mode to let it update data register*/
    ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_NORMAL);

    ret = fscanf(value_fp, "%8d %8d %8d", &X_tmp, &Y_tmp, &Z_tmp);
    fclose(value_fp);
    if(ret <= 0){
        PERR("fscanf fail, errno = %d(%s)", errno, strerror(errno));
        return;
    }

    p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata)
    {
        PERR("malloc fail");
        return;
    }

    p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    p_hwdata->x_uncalib = X_tmp * 10 * YAS5xx_COMPVAL_TO_uT;
    p_hwdata->y_uncalib = Y_tmp * 10 * YAS5xx_COMPVAL_TO_uT;
    p_hwdata->z_uncalib = Z_tmp * 10 * YAS5xx_COMPVAL_TO_uT;
    p_hwdata->timestamp = mag_poll_tmfilt(sys_tm);

    hw_remap_sensor_data(&(p_hwdata->x_uncalib), &(p_hwdata->y_uncalib), &(p_hwdata->z_uncalib), g_place_m);

    ret = dest_list->list_add_rear((void *) p_hwdata);
    if (ret)
    {
        PERR("list_add_rear() fail, ret = %d", ret);
        if(-1 == ret){
            free(p_hwdata);
        }
    }

    return;
}


static void ap_hw_poll(BstSimpleList *acc_dest_list, BstSimpleList *gyr_dest_list, BstSimpleList *mag_dest_list)
{
    if(ACC_CHIP_BMA2x2 == accl_chip && is_acc_open)
    {
        if(acc_poll_cnt == acc_poll_mulp)
        {
            ap_hw_poll_bma2x2(acc_dest_list);
            acc_poll_cnt = 1;
        }else{
            acc_poll_cnt++;
        }
    }

    if(GYR_CHIP_BMG160 == gyro_chip && is_gyr_open)
    {
        if(gyr_poll_cnt == gyr_poll_mulp)
        {
            ap_hw_poll_bmg160(gyr_dest_list);
            gyr_poll_cnt = 1;
        }else{
            gyr_poll_cnt++;
        }
    }

    if((MAG_CHIP_BMM150 == magn_chip || MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip ||
            MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip) &&
            is_mag_open)
    {
        if(mag_poll_cnt == mag_poll_mulp)
        {
            if(MAG_CHIP_BMM150 == magn_chip){
                ap_hw_poll_bmm150(mag_dest_list);
            }else if(MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip){
                ap_hw_poll_akm099xx(mag_dest_list);
            }else if(MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip){
                ap_hw_poll_yas5xx(mag_dest_list);
            }
            mag_poll_cnt = 1;
        }else{
            mag_poll_cnt++;
        }
    }
}

/*
static void dump_samples(BstSimpleList *al, BstSimpleList *gl, BstSimpleList *ml)
{
    HW_DATA_UNION *p_hwdata;
    struct list_node *pnode;
    uint32_t i;

    pnode = al->head;
    for (i = 0; i < al->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(pnode->p_data);
        PNOTE("**ACCL tm = %lld", p_hwdata->timestamp);
        pnode = pnode->next;
    }

    pnode = gl->head;
    for (i = 0; i < gl->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(pnode->p_data);
        PNOTE("--GYRO tm = %lld", p_hwdata->timestamp);
        pnode = pnode->next;
    }

    pnode = ml->head;
    for (i = 0; i < ml->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(pnode->p_data);
        PNOTE("##MAGN tm = %lld", p_hwdata->timestamp);
        pnode = pnode->next;
    }

    PNOTE("==============================");

    return;
}
*/

/*align Magnetic signal's tm*/
static void syncTM_pollMAG_BMI160(BstSimpleList *al, BstSimpleList *gl, BstSimpleList *ml)
{
#define MAX_TM_DEVI 4000000
#define MAX_TMBUF_LEN 20
#define MAX_MAGTMBUF_LEN 4
    uint32_t i;
    uint32_t j;
    uint32_t loop = 0;
    int64_t MAG_tm_buf[MAX_MAGTMBUF_LEN] = {0};
    uint32_t initlen_MAG_tmbuf = 0;
    HW_DATA_UNION *MAG_p_hwdata = NULL;
    struct list_node *tmp_node = NULL;
    HW_DATA_UNION *p_hwdata = NULL;
    int64_t base_tm_buf[MAX_TMBUF_LEN] = {0};

    if(0 == gl->list_len + al->list_len){
        /*gyro and acc seems not opened, nothing to do*/;
        return;
    }

    if(0 == ml->list_len){
        return;
    }else if(ml->list_len > MAX_MAGTMBUF_LEN)
    {
        /*This should not happen*/
        PWARN("mag samples amount %d exceed assumed %d", ml->list_len, MAX_MAGTMBUF_LEN);
        while(MAX_MAGTMBUF_LEN < ml->list_len){
            ml->list_get_headdata((void **)&MAG_p_hwdata);
            free(MAG_p_hwdata);
        }
    }

    tmp_node = ml->head;
    for (i = 0; i < ml->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(tmp_node->p_data);
        MAG_tm_buf[i] = p_hwdata->timestamp;
        tmp_node = tmp_node->next;
    }

    if(gl->list_len)
    {
        loop = gl->list_len;
        tmp_node = gl->head;
    }else if(al->list_len)
    {
        loop = al->list_len;
        tmp_node = al->head;
    }

    /*While BMI160 open accl/gyro in process, it's FIFO will drop several samples to synchronize
     * accl and gyro output. But the magn samples are still reporting at that time. So we need
     * to detect that and drop obselete magn sample also to synchronize magn output with BMI160 FIFO*/
    p_hwdata = (HW_DATA_UNION *)(tmp_node->p_data);
    initlen_MAG_tmbuf = ml->list_len;
    for (i = 0; i < initlen_MAG_tmbuf; ++i)
    {
        if(MAG_tm_buf[i] < p_hwdata->timestamp){
            ml->list_get_headdata((void **)&MAG_p_hwdata);
            free(MAG_p_hwdata);
            memmove(&MAG_tm_buf[0], &MAG_tm_buf[1], ml->list_len * sizeof(MAG_tm_buf[0]));
        }
    }

    if(0 == ml->list_len){
        /*magn samples are all obsolete, return*/
        return;
    }

    if(loop <= MAX_TMBUF_LEN)
    {
        /*more elegant sync*/
        for (i = 0; i < loop; ++i){
            p_hwdata = (HW_DATA_UNION *)(tmp_node->p_data);
            base_tm_buf[i] = p_hwdata->timestamp;
            tmp_node = tmp_node->next;
        }

        for (j = 0; j < ml->list_len; ++j)
        {
            for (i = 0; i < loop; ++i)
            {
                if((0 == i && MAG_tm_buf[j] == base_tm_buf[i]) ||
                        (i == loop - 1 && MAG_tm_buf[j] > base_tm_buf[i]))
                {
                    MAG_tm_buf[j] = base_tm_buf[i];
                    break;
                }else if(MAG_tm_buf[j] <= base_tm_buf[i] && MAG_tm_buf[j] > base_tm_buf[i-1])
                {
                    if(MAG_tm_buf[j] - base_tm_buf[i-1] < base_tm_buf[i] - MAG_tm_buf[j])
                    {
                        MAG_tm_buf[j] = base_tm_buf[i-1];
                    }else
                    {
                        MAG_tm_buf[j] = base_tm_buf[i];
                    }
                    break;
                }
            }
        }
    }else
    {
        /*use a roughly sync*/
        PWARN("too long list to buffer tm, list_len = %d", loop);
        for (j = 0; j < ml->list_len; ++j)
        {
            for (i = 0; i < loop; ++i)
            {
                p_hwdata = (HW_DATA_UNION *)(tmp_node->p_data);
                if(ABS(p_hwdata->timestamp - MAG_tm_buf[j]) < MAX_TM_DEVI ||
                        p_hwdata->timestamp > MAG_tm_buf[j])
                {
                    MAG_tm_buf[j] = p_hwdata->timestamp;
                    break;
                }

                tmp_node = tmp_node->next;
            }

            if(i == loop)
            {
                MAG_tm_buf[j] = p_hwdata->timestamp;
            }
        }
    }

    /*assign the buffered timestamp to samples*/
    tmp_node = ml->head;
    for (i = 0; i < ml->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(tmp_node->p_data);
        p_hwdata->timestamp = MAG_tm_buf[i];
        tmp_node = tmp_node->next;
    }

    return;
}

/*align Magnetic tm, BMG160 and BMA2x2(BMI055)*/
static void syncTM_pollMAG_BMI055(BstSimpleList *al, BstSimpleList *gl, BstSimpleList *ml)
{
    uint32_t i;
    uint32_t loop;
    uint32_t rm_cnt;
    HW_DATA_UNION *MAG_p_hwdata = NULL;
    HW_DATA_UNION *base_p_hwdata = NULL;
    uint32_t gyr_cnt_tosend = 0;
    uint32_t mag_cnt_tosend = 0;

    if(0 == gl->list_len + al->list_len){
        /*gyro and acc seems not opened, nothing to do*/;
        return;
    }

    if(ml->list_len > 1)
    {
        PWARN("more than one! ml->list_len = %d", ml->list_len);
        while(1 != ml->list_len){
            ml->list_get_headdata((void **)&MAG_p_hwdata);
            free(MAG_p_hwdata);
        }
    }

    if(1 == ml->list_len)
    {
        /* align M firstly.
         * for open sequence is always A->M->G, so that polled BMA2x2 has a fixed mapping with polled MAG in tm
         * so it's sufficient to align MAG samples with last A samples*/
        if(al->list_len){
            MAG_p_hwdata = (HW_DATA_UNION *)(ml->head->p_data);
            base_p_hwdata = (HW_DATA_UNION *)(al->tail->p_data);
            MAG_p_hwdata->timestamp = base_p_hwdata->timestamp;
        }else
        {
            MAG_p_hwdata = (HW_DATA_UNION *)(ml->head->p_data);
            base_p_hwdata = (HW_DATA_UNION *)(gl->tail->p_data);
            MAG_p_hwdata->timestamp = base_p_hwdata->timestamp;
        }
    }

    /* align G to A
     * Since M has been aligned to A, so A should be standard to more properly sync*/
    if(gl->list_len && al->list_len)
    {
        /*read list to align buffer*/
        if(al->list_len > ALIGN_BUF_MAXLEN)
        {
            PERR("al->list_len = %u, while ALIGN_BUF_MAXLEN = %u", al->list_len, ALIGN_BUF_MAXLEN);
            loop = al->list_len - ALIGN_BUF_MAXLEN;
            for (i = 0; i < loop; ++i)
            {
                al->list_get_headdata((void **)&base_p_hwdata);
                free(base_p_hwdata);
            }
        }

        loop = al->list_len;
        for (i = 0; i < loop; ++i)
        {
            al->list_get_headdata((void **)(&p_BMA2x2smpl_alignbuf[i]));
        }
        BMA2x2smpl_alignbuf_len = loop;


        if(gl->list_len > ALIGN_BUF_MAXLEN)
        {
            PERR("gl->list_len = %u, while ALIGN_BUF_MAXLEN = %u", gl->list_len, ALIGN_BUF_MAXLEN);
            loop = gl->list_len - ALIGN_BUF_MAXLEN;
            for (i = 0; i < loop; ++i)
            {
                gl->list_get_headdata((void **)&base_p_hwdata);
                free(base_p_hwdata);
            }
        }


        if(BMG160smpl_alignbuf_len + gl->list_len > ALIGN_BUF_MAXLEN)
        {
            rm_cnt = BMG160smpl_alignbuf_len + gl->list_len - ALIGN_BUF_MAXLEN;
            for (i = 0; i < rm_cnt; ++i) {
                free(p_BMG160smpl_alignbuf[i]);
            }
            BMG160smpl_alignbuf_len = BMG160smpl_alignbuf_len - rm_cnt;
            memmove(&p_BMG160smpl_alignbuf[0], &p_BMG160smpl_alignbuf[rm_cnt],
                    BMG160smpl_alignbuf_len * sizeof(HW_DATA_UNION *));
        }

        loop = gl->list_len;
        for (i = 0; i < loop; ++i)
        {
            gl->list_get_headdata((void **)(&p_BMG160smpl_alignbuf[BMG160smpl_alignbuf_len]));
            BMG160smpl_alignbuf_len++;
        }

        /*read M sample to buffer to see if send it this time or
         * remain it because matched A|G sample is remained this time*/
        loop = ml->list_len;
        for (i = 0; i < loop; ++i)
        {
            if(BMM150smpl_alignbuf_len == ALIGN_BUF_MAXLEN){
                /*Can not arrive here*/
                PERR("bmm150 align buffer overflow");
                break;
            }

            ml->list_get_headdata((void **)(&p_BMM150smpl_alignbuf[BMM150smpl_alignbuf_len]));
                        BMM150smpl_alignbuf_len++;
        }

        /*assign tm*/
        gyr_cnt_tosend = 0;

        if(BMA2x2smpl_alignbuf_len >= BMG160smpl_alignbuf_len)
        {
            for (i = 0; i < BMG160smpl_alignbuf_len; ++i)
            {
                p_BMG160smpl_alignbuf[BMG160smpl_alignbuf_len-1-i]->timestamp =
                        p_BMA2x2smpl_alignbuf[BMA2x2smpl_alignbuf_len-1-i]->timestamp;
            }
            gyr_cnt_tosend = BMG160smpl_alignbuf_len;
        }else
        {
            for (i = 0; i < BMA2x2smpl_alignbuf_len; ++i)
            {
                p_BMG160smpl_alignbuf[i]->timestamp = p_BMA2x2smpl_alignbuf[i]->timestamp;
            }
            gyr_cnt_tosend = BMA2x2smpl_alignbuf_len;
        }

        /*deliver aligned samples and buffer the left for the next alignment*/
        for (i = 0; i < BMA2x2smpl_alignbuf_len; ++i) {
            al->list_add_rear(p_BMA2x2smpl_alignbuf[i]);
        }
        BMA2x2smpl_alignbuf_len = 0;

        for (i = 0; i < gyr_cnt_tosend; ++i) {
            gl->list_add_rear(p_BMG160smpl_alignbuf[i]);
        }
        BMG160smpl_alignbuf_len = BMG160smpl_alignbuf_len - gyr_cnt_tosend;
        memmove(&p_BMG160smpl_alignbuf[0], &p_BMG160smpl_alignbuf[gyr_cnt_tosend],
                BMG160smpl_alignbuf_len * sizeof(HW_DATA_UNION *));
        if(BMG160smpl_alignbuf_len > HIST_GYR_MAX)
        {
            /*BMG can produce more samples than BMA. Then drop some history samples(usually one sample need to remove)*/
            rm_cnt = BMG160smpl_alignbuf_len - HIST_GYR_MAX;
            for (i = 0; i < rm_cnt; ++i) {
                free(p_BMG160smpl_alignbuf[i]);
            }
            BMG160smpl_alignbuf_len = BMG160smpl_alignbuf_len - rm_cnt;
            memmove(&p_BMG160smpl_alignbuf[0], &p_BMG160smpl_alignbuf[rm_cnt],
                    BMG160smpl_alignbuf_len * sizeof(HW_DATA_UNION *));
        }

        mag_cnt_tosend = 0;
        /*if no A sample to send, also no need of sending M samples, since M is aligned to A*/
        if(al->list_len)
        {
            base_p_hwdata = (HW_DATA_UNION *)(al->tail->p_data);
            for (i = 0; i < BMM150smpl_alignbuf_len; ++i)
            {
                if (p_BMM150smpl_alignbuf[i]->timestamp <= base_p_hwdata->timestamp)
                {
                    ml->list_add_rear(p_BMM150smpl_alignbuf[i]);
                    mag_cnt_tosend++;
                }
            }

            BMM150smpl_alignbuf_len = BMM150smpl_alignbuf_len - mag_cnt_tosend;
            memmove(&p_BMM150smpl_alignbuf[0], &p_BMM150smpl_alignbuf[mag_cnt_tosend],
                    BMM150smpl_alignbuf_len * sizeof(HW_DATA_UNION *));
        }

    }

    return;
}


static uint32_t MDOF_hw_deliver_sensordata(BstSensor *bstsensor)
{
    int32_t ret;
    int32_t i;
    uint32_t j;
    char *raw_data_buffer = NULL;
    struct pollfd poll_fds[4];
    ssize_t read_size;
    uint64_t over_run = 0;

    if(ACC_CHIP_BMI160 == accl_chip)
    {
        poll_fds[0].fd = accl_iio_fd;
        poll_fds[0].events = POLLIN;
    }

    if(GYR_CHIP_BMI160 == gyro_chip)
    {
        poll_fds[1].fd = gyro_iio_fd;
        poll_fds[1].events = POLLIN;
    }

    if(MAG_CHIP_BMI160 == magn_chip)
    {
        poll_fds[2].fd = magn_iio_fd;
        poll_fds[2].events = POLLIN;
    }

    poll_fds[3].fd = poll_timer_fd;
    poll_fds[3].events = POLLIN;

    ret = poll(poll_fds, ARRAY_ELEMENTS(poll_fds), -1);
    if (ret <= 0)
    {
        PERR("poll in error: ret=%d", ret);
        return 0;
    }

    for (j = 0; j < ARRAY_ELEMENTS(poll_fds); j++)
    {
        if (POLLIN != poll_fds[j].revents)
        {
            continue;
        }

        switch (j)
        {
            case 0:
                raw_data_buffer = (char *) malloc(hwdata_unit_toread * accl_scan_size);
                if (NULL == raw_data_buffer)
                {
                    PERR("malloc fail");
                    return 0;
                }

                read_size = read(poll_fds[j].fd, raw_data_buffer, hwdata_unit_toread * accl_scan_size);
                if (ret < 0)
                {
                    PERR("read raw data fail\n");
                    free(raw_data_buffer);
                    return 0;
                }

                for (i = 0; i < read_size / accl_scan_size; i++)
                {
                    ap_hw_process_IIO_frame(FIFO_HEAD_A, raw_data_buffer + accl_scan_size * i,
                            bstsensor->tmplist_hwcntl_acclraw);
                }

                free(raw_data_buffer);

                break;

            case 1:
                raw_data_buffer = (char *) malloc(hwdata_unit_toread * gyro_scan_size);
                if (NULL == raw_data_buffer)
                {
                    PERR("malloc fail");
                    return 0;
                }

                read_size = read(poll_fds[j].fd, raw_data_buffer, hwdata_unit_toread * gyro_scan_size);
                if (ret < 0)
                {
                    PERR("read raw data fail\n");
                    free(raw_data_buffer);
                    return 0;
                }

                for (i = 0; i < read_size / gyro_scan_size; i++)
                {
                    ap_hw_process_IIO_frame(FIFO_HEAD_G, raw_data_buffer + gyro_scan_size * i,
                            bstsensor->tmplist_hwcntl_gyroraw);
                }

                free(raw_data_buffer);

                break;

            case 2:
                raw_data_buffer = (char *) malloc(hwdata_unit_toread * magn_scan_size);
                if (NULL == raw_data_buffer)
                {
                    PERR("malloc fail");
                    return 0;
                }

                read_size = read(poll_fds[j].fd, raw_data_buffer, hwdata_unit_toread * magn_scan_size);
                if (ret < 0)
                {
                    PERR("read raw data fail\n");
                    free(raw_data_buffer);
                    return 0;
                }

                for (i = 0; i < read_size / magn_scan_size; i++)
                {
                    ap_hw_process_IIO_frame(FIFO_HEAD_M, raw_data_buffer + magn_scan_size * i,
                            bstsensor->tmplist_hwcntl_magnraw);
                }

                free(raw_data_buffer);

                break;

            case 3:
                ret = read(poll_timer_fd, &over_run, sizeof(uint64_t));
                if(sizeof(uint64_t) == ret){
                    if(ACC_CHIP_BMA2x2 == accl_chip && GYR_CHIP_BMG160 == gyro_chip)
                    {
                        ap_hw_poll(bstsensor->tmplist_hwcntl_acclraw,
                                bstsensor->tmplist_hwcntl_gyroraw,
                                bstsensor->tmplist_hwcntl_magnraw);
                    }else if(ACC_CHIP_BMI160 == accl_chip && GYR_CHIP_BMI160 == gyro_chip)
                    {
                        if(MAG_CHIP_BMM150 == magn_chip){
                            ap_hw_poll_bmm150(bstsensor->tmplist_hwcntl_magnraw);
                        }else if(MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip){
                            ap_hw_poll_akm099xx(bstsensor->tmplist_hwcntl_magnraw);
                        }else if(MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip){
                            ap_hw_poll_yas5xx(bstsensor->tmplist_hwcntl_magnraw);
                        }
                    }
                }

                break;
        }

    }

#if 1
    if (bstsensor->tmplist_hwcntl_acclraw->list_len
            + bstsensor->tmplist_hwcntl_gyroraw->list_len
            + bstsensor->tmplist_hwcntl_magnraw->list_len)
    {
        if(ACC_CHIP_BMI160 == accl_chip && GYR_CHIP_BMI160 == gyro_chip &&
                (MAG_CHIP_BMM150 == magn_chip || MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip ||
                        MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip))
        {
            /** BMI160_IIO plus timer_fd. We need a sync here between A/G and M */
            if(0 == (bstsensor->tmplist_hwcntl_acclraw->list_len + bstsensor->tmplist_hwcntl_gyroraw->list_len) &&
                    (is_acc_open || is_gyr_open))
            {
                return 0;
            }

            syncTM_pollMAG_BMI160(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw, bstsensor->tmplist_hwcntl_magnraw);
            //dump_samples(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw, bstsensor->tmplist_hwcntl_magnraw);

        }else if(ACC_CHIP_BMA2x2 == accl_chip && GYR_CHIP_BMG160 == gyro_chip && MAG_CHIP_BMM150 == magn_chip)
        {
            syncTM_pollMAG_BMI055(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw, bstsensor->tmplist_hwcntl_magnraw);
            //dump_samples(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw, bstsensor->tmplist_hwcntl_magnraw);
        }

        pthread_mutex_lock(&(bstsensor->shmem_hwcntl.mutex));

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_acclraw);
        if(ret){
            PWARN("list mount fail");
        }

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_gyroraw);
        if(ret){
            PWARN("list mount fail");
        }

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_magnraw);
        if(ret){
            PWARN("list mount fail");
        }

        pthread_cond_signal(&(bstsensor->shmem_hwcntl.cond));
        pthread_mutex_unlock(&(bstsensor->shmem_hwcntl.mutex));
    }
#endif
    return 0;

}


static void syncTM_BMI055(BstSimpleList *al, BstSimpleList *gl)
{
    uint32_t i;
    uint32_t loop;
    uint32_t rm_cnt;
    HW_DATA_UNION *base_p_hwdata = NULL;
    uint32_t gyr_cnt_tosend = 0;

    /* align G to A*/
    if(gl->list_len && al->list_len)
    {
        /*read list to align buffer*/
        if(al->list_len > ALIGN_BUF_MAXLEN)
        {
            PERR("al->list_len = %u, while ALIGN_BUF_MAXLEN = %u", al->list_len, ALIGN_BUF_MAXLEN);
            loop = al->list_len - ALIGN_BUF_MAXLEN;
            for (i = 0; i < loop; ++i)
            {
                al->list_get_headdata((void **)&base_p_hwdata);
                free(base_p_hwdata);
            }
        }

        loop = al->list_len;
        for (i = 0; i < loop; ++i)
        {
            al->list_get_headdata((void **)(&p_BMA2x2smpl_alignbuf[i]));
        }
        BMA2x2smpl_alignbuf_len = loop;


        if(gl->list_len > ALIGN_BUF_MAXLEN)
        {
            PERR("gl->list_len = %u, while ALIGN_BUF_MAXLEN = %u", gl->list_len, ALIGN_BUF_MAXLEN);
            loop = gl->list_len - ALIGN_BUF_MAXLEN;
            for (i = 0; i < loop; ++i)
            {
                gl->list_get_headdata((void **)&base_p_hwdata);
                free(base_p_hwdata);
            }
        }


        if(BMG160smpl_alignbuf_len + gl->list_len > ALIGN_BUF_MAXLEN)
        {
            rm_cnt = BMG160smpl_alignbuf_len + gl->list_len - ALIGN_BUF_MAXLEN;
            for (i = 0; i < rm_cnt; ++i) {
                free(p_BMG160smpl_alignbuf[i]);
            }
            BMG160smpl_alignbuf_len = BMG160smpl_alignbuf_len - rm_cnt;
            memmove(&p_BMG160smpl_alignbuf[0], &p_BMG160smpl_alignbuf[rm_cnt],
                    BMG160smpl_alignbuf_len * sizeof(HW_DATA_UNION *));
        }

        loop = gl->list_len;
        for (i = 0; i < loop; ++i)
        {
            gl->list_get_headdata((void **)(&p_BMG160smpl_alignbuf[BMG160smpl_alignbuf_len]));
            BMG160smpl_alignbuf_len++;
        }


        /*assign tm*/
        gyr_cnt_tosend = 0;

        if(BMA2x2smpl_alignbuf_len >= BMG160smpl_alignbuf_len)
        {
            for (i = 0; i < BMG160smpl_alignbuf_len; ++i)
            {
                p_BMG160smpl_alignbuf[BMG160smpl_alignbuf_len-1-i]->timestamp =
                        p_BMA2x2smpl_alignbuf[BMA2x2smpl_alignbuf_len-1-i]->timestamp;
            }
            gyr_cnt_tosend = BMG160smpl_alignbuf_len;
        }else
        {
            for (i = 0; i < BMA2x2smpl_alignbuf_len; ++i)
            {
                p_BMG160smpl_alignbuf[i]->timestamp = p_BMA2x2smpl_alignbuf[i]->timestamp;
            }
            gyr_cnt_tosend = BMA2x2smpl_alignbuf_len;
        }

        /*deliver aligned samples and buffer the left for the next alignment*/
        for (i = 0; i < BMA2x2smpl_alignbuf_len; ++i) {
            al->list_add_rear(p_BMA2x2smpl_alignbuf[i]);
        }
        BMA2x2smpl_alignbuf_len = 0;

        for (i = 0; i < gyr_cnt_tosend; ++i) {
            gl->list_add_rear(p_BMG160smpl_alignbuf[i]);
        }
        BMG160smpl_alignbuf_len = BMG160smpl_alignbuf_len - gyr_cnt_tosend;
        memmove(&p_BMG160smpl_alignbuf[0], &p_BMG160smpl_alignbuf[gyr_cnt_tosend],
                BMG160smpl_alignbuf_len * sizeof(HW_DATA_UNION *));
        if(BMG160smpl_alignbuf_len > HIST_GYR_MAX)
        {
            /*BMG can produce more samples than BMA. Then drop some history samples(usually one sample need to remove)*/
            rm_cnt = BMG160smpl_alignbuf_len - HIST_GYR_MAX;
            for (i = 0; i < rm_cnt; ++i) {
                free(p_BMG160smpl_alignbuf[i]);
            }
            BMG160smpl_alignbuf_len = BMG160smpl_alignbuf_len - rm_cnt;
            memmove(&p_BMG160smpl_alignbuf[0], &p_BMG160smpl_alignbuf[rm_cnt],
                    BMG160smpl_alignbuf_len * sizeof(HW_DATA_UNION *));
        }
    }

    return;
}


static uint32_t IMU_hw_deliver_sensordata(BstSensor *bstsensor)
{
    int32_t ret;
    int32_t i;
    uint32_t j;
    char *raw_data_buffer = NULL;
    struct pollfd poll_fds[3];
    ssize_t read_size;
    uint64_t over_run = 0;

    if(ACC_CHIP_BMI160 == accl_chip)
    {
        poll_fds[0].fd = accl_iio_fd;
        poll_fds[0].events = POLLIN;
    }

    if(GYR_CHIP_BMI160 == gyro_chip)
    {
        poll_fds[1].fd = gyro_iio_fd;
        poll_fds[1].events = POLLIN;
    }

    poll_fds[2].fd = poll_timer_fd;
    poll_fds[2].events = POLLIN;

    ret = poll(poll_fds, ARRAY_ELEMENTS(poll_fds), -1);
    if (ret <= 0)
    {
        PERR("poll in error: ret=%d", ret);
        return 0;
    }

    for (j = 0; j < ARRAY_ELEMENTS(poll_fds); j++)
    {
        if (POLLIN != poll_fds[j].revents)
        {
            continue;
        }

        switch (j)
        {
            case 0:
                raw_data_buffer = (char *) malloc(hwdata_unit_toread * accl_scan_size);
                if (NULL == raw_data_buffer)
                {
                    PERR("malloc fail");
                    return 0;
                }

                read_size = read(poll_fds[j].fd, raw_data_buffer, hwdata_unit_toread * accl_scan_size);
                if (ret < 0)
                {
                    PERR("read raw data fail\n");
                    free(raw_data_buffer);
                    return 0;
                }

                for (i = 0; i < read_size / accl_scan_size; i++)
                {
                    ap_hw_process_IIO_frame(FIFO_HEAD_A, raw_data_buffer + accl_scan_size * i,
                            bstsensor->tmplist_hwcntl_acclraw);
                }

                free(raw_data_buffer);

                break;

            case 1:
                raw_data_buffer = (char *) malloc(hwdata_unit_toread * gyro_scan_size);
                if (NULL == raw_data_buffer)
                {
                    PERR("malloc fail");
                    return 0;
                }

                read_size = read(poll_fds[j].fd, raw_data_buffer, hwdata_unit_toread * gyro_scan_size);
                if (ret < 0)
                {
                    PERR("read raw data fail\n");
                    free(raw_data_buffer);
                    return 0;
                }

                for (i = 0; i < read_size / gyro_scan_size; i++)
                {
                    ap_hw_process_IIO_frame(FIFO_HEAD_G, raw_data_buffer + gyro_scan_size * i,
                            bstsensor->tmplist_hwcntl_gyroraw);
                }

                free(raw_data_buffer);

                break;

            case 2:
                ret = read(poll_timer_fd, &over_run, sizeof(uint64_t));
                if(sizeof(uint64_t) == ret){
                    ap_hw_poll(bstsensor->tmplist_hwcntl_acclraw,
                            bstsensor->tmplist_hwcntl_gyroraw,
                            bstsensor->tmplist_hwcntl_magnraw);
                }

                break;
        }

    }

#if 1
    if (bstsensor->tmplist_hwcntl_acclraw->list_len
            + bstsensor->tmplist_hwcntl_gyroraw->list_len)
    {
        if(ACC_CHIP_BMA2x2 == accl_chip && GYR_CHIP_BMG160 == gyro_chip)
        {
            syncTM_BMI055(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw);
            //dump_samples(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw, bstsensor->tmplist_hwcntl_magnraw);
        }

        pthread_mutex_lock(&(bstsensor->shmem_hwcntl.mutex));

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_acclraw);
        if(ret){
            PWARN("list mount fail");
        }

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_gyroraw);
        if(ret){
            PWARN("list mount fail");
        }

        pthread_cond_signal(&(bstsensor->shmem_hwcntl.cond));
        pthread_mutex_unlock(&(bstsensor->shmem_hwcntl.mutex));
    }
#endif
    return 0;

}


static uint32_t ACCONLY_hw_deliver_sensordata(BstSensor *bstsensor)
{
    int32_t ret;
    int32_t i;
    uint32_t j;
    char *raw_data_buffer = NULL;
    struct pollfd poll_fds[1];
    ssize_t read_size;
    uint64_t over_run = 0;

    if(ACC_CHIP_BMI160 == accl_chip)
    {
        poll_fds[0].fd = accl_iio_fd;
        poll_fds[0].events = POLLIN;
    }else if(ACC_CHIP_BMA2x2 == accl_chip)
    {
        poll_fds[0].fd = poll_timer_fd;
        poll_fds[0].events = POLLIN;
    }

    ret = poll(poll_fds, ARRAY_ELEMENTS(poll_fds), -1);
    if (ret <= 0)
    {
        PERR("poll in error: ret=%d", ret);
        return 0;
    }

    for (j = 0; j < ARRAY_ELEMENTS(poll_fds); j++)
    {
        if (POLLIN != poll_fds[j].revents)
        {
            continue;
        }

        switch (j)
        {
            case 0:
                if(ACC_CHIP_BMI160 == accl_chip)
                {
                    raw_data_buffer = (char *) malloc(hwdata_unit_toread * accl_scan_size);
                    if (NULL == raw_data_buffer)
                    {
                        PERR("malloc fail");
                        return 0;
                    }

                    read_size = read(poll_fds[j].fd, raw_data_buffer, hwdata_unit_toread * accl_scan_size);
                    if (ret < 0)
                    {
                        PERR("read raw data fail\n");
                        free(raw_data_buffer);
                        return 0;
                    }

                    for (i = 0; i < read_size / accl_scan_size; i++)
                    {
                        ap_hw_process_IIO_frame(FIFO_HEAD_A, raw_data_buffer + accl_scan_size * i,
                                bstsensor->tmplist_hwcntl_acclraw);
                    }

                    free(raw_data_buffer);
                }else if (ACC_CHIP_BMA2x2 == accl_chip)
                {
                    ret = read(poll_timer_fd, &over_run, sizeof(uint64_t));
                    if(sizeof(uint64_t) == ret){
                        ap_hw_poll(bstsensor->tmplist_hwcntl_acclraw,
                                bstsensor->tmplist_hwcntl_gyroraw,
                                bstsensor->tmplist_hwcntl_magnraw);
                    }
                }

                break;
        }

    }

#if 1
    if (bstsensor->tmplist_hwcntl_acclraw->list_len)
    {
        pthread_mutex_lock(&(bstsensor->shmem_hwcntl.mutex));

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_acclraw);
        if(ret){
            PWARN("list mount fail");
        }

        pthread_cond_signal(&(bstsensor->shmem_hwcntl.cond));
        pthread_mutex_unlock(&(bstsensor->shmem_hwcntl.mutex));
    }
#endif
    return 0;

}


/*align Magnetic signal's tm, polled acc tm should be reliable ahead*/
static void syncTM_pollMAG_pollBMA2x2(BstSimpleList *al, BstSimpleList *ml)
{
    HW_DATA_UNION *MAG_p_hwdata = NULL;
    HW_DATA_UNION *base_p_hwdata = NULL;

    if(0 == al->list_len){
        /*acc not opened, nothing to do*/;
        return;
    }

    if(0 == ml->list_len){
        return;
    }else if(ml->list_len > 1)
    {
        PWARN("more than one! ml->list_len = %d", ml->list_len);
        while(1 != ml->list_len){
            ml->list_get_headdata((void **)&MAG_p_hwdata);
            free(MAG_p_hwdata);
        }
    }

    /*polled BMA2x2 tm is acccurate, so it's sufficient to align MAG samples with last A samples*/
    MAG_p_hwdata = (HW_DATA_UNION *)(ml->head->p_data);
    base_p_hwdata = (HW_DATA_UNION *)(al->tail->p_data);
    MAG_p_hwdata->timestamp = base_p_hwdata->timestamp;

    return;
}

static uint32_t COMPASS_M4G_hw_deliver_sensordata(BstSensor *bstsensor)
{
    int32_t ret;
    uint32_t j;
    struct pollfd poll_fds[1];
    uint64_t over_run = 0;

    poll_fds[0].fd = poll_timer_fd;
    poll_fds[0].events = POLLIN;

    ret = poll(poll_fds, ARRAY_ELEMENTS(poll_fds), -1);
    if (ret <= 0)
    {
        PERR("poll in error: ret=%d", ret);
        return 0;
    }

    for (j = 0; j < ARRAY_ELEMENTS(poll_fds); j++)
    {
        if (POLLIN != poll_fds[j].revents)
        {
            continue;
        }

        switch (j)
        {
            case 0:
                ret = read(poll_timer_fd, &over_run, sizeof(uint64_t));
                if(sizeof(uint64_t) == ret){
                    ap_hw_poll(bstsensor->tmplist_hwcntl_acclraw,
                            bstsensor->tmplist_hwcntl_gyroraw,
                            bstsensor->tmplist_hwcntl_magnraw);
                }
                break;
        }
    }

#if 1
    if (bstsensor->tmplist_hwcntl_acclraw->list_len
            + bstsensor->tmplist_hwcntl_magnraw->list_len)
    {
        //dump_samples(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw, bstsensor->tmplist_hwcntl_magnraw);
        syncTM_pollMAG_pollBMA2x2(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_magnraw);

        pthread_mutex_lock(&(bstsensor->shmem_hwcntl.mutex));

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_acclraw);
        if(ret){
            PWARN("list mount fail");
        }

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_magnraw);
        if(ret){
            PWARN("list mount fail");
        }

        pthread_cond_signal(&(bstsensor->shmem_hwcntl.cond));
        pthread_mutex_unlock(&(bstsensor->shmem_hwcntl.mutex));
    }
#endif
    return 0;

}



static void ap_show_ver()
{
    const char* accl_chip_name = NULL;
    const char* gyro_chip_name = NULL;
    const char* magn_chip_name = NULL;
    const char* solution_name = NULL;
    char data_log_buf[256] = { 0 };

    switch (accl_chip) {
        case ACC_CHIP_BMI160:
            accl_chip_name = "BMI160";
            break;
        case ACC_CHIP_BMA2x2:
            accl_chip_name = "BMA2x2";
            break;
    }

    switch (gyro_chip) {
        case GYR_CHIP_BMI160:
            gyro_chip_name = "BMI160";
            break;
        case GYR_CHIP_BMG160:
            gyro_chip_name = "BMG160";
            break;
    }

    switch (magn_chip) {
        case MAG_CHIP_BMI160:
            magn_chip_name = "BMI160_Aux";
            break;
        case MAG_CHIP_BMM150:
            magn_chip_name = "BMM150";
            break;
        case MAG_CHIP_AKM09911:
            magn_chip_name = "AKM09911";
            break;
        case MAG_CHIP_AKM09912:
            magn_chip_name = "AKM09912";
            break;
        case MAG_CHIP_YAS532:
            magn_chip_name = "YAS532";
            break;
        case MAG_CHIP_YAS537:
            magn_chip_name = "YAS537";
            break;
    }

    switch (solution_type) {
        case SOLUTION_MDOF:
            solution_name = "MDOF";
            break;
        case SOLUTION_ECOMPASS:
            solution_name = "ECOMPASS";
            gyro_chip_name = NULL;
            break;
        case SOLUTION_IMU:
            solution_name = "IMU";
            magn_chip_name = NULL;
            break;
        case SOLUTION_M4G:
            solution_name = "M4G";
            gyro_chip_name = NULL;
            break;
        case SOLUTION_ACC:
            solution_name = "ACC";
            magn_chip_name = NULL;
            gyro_chip_name = NULL;
            break;
        default:
            solution_name = "Unknown";
            accl_chip_name = NULL;
            magn_chip_name = NULL;
            gyro_chip_name = NULL;
            break;
    }


    PNOTE("\n **************************\n \
* HAL version: %d.%d.%d.%d\n \
* build time: %s, %s\n \
* solution type: %s\n \
* accl_chip: %s\n \
* magn_chip: %s\n \
* gyro_chip: %s\n \
**************************",
            HAL_ver[0], HAL_ver[1], HAL_ver[2], HAL_ver[3],
            __DATE__, __TIME__, solution_name, accl_chip_name, magn_chip_name, gyro_chip_name);

    if(bsx_datalog){
        sprintf(data_log_buf, "<component>HAL version: %d.%d.%d.%d</component>\n",
                HAL_ver[0], HAL_ver[1], HAL_ver[2], HAL_ver[3]);
        bsx_datalog_algo(data_log_buf);
        sprintf(data_log_buf, "<component>build time: %s, %s</component>\n",
                __DATE__, __TIME__);
        bsx_datalog_algo(data_log_buf);
        sprintf(data_log_buf, "<component>solution type: %s</component>\n\n", solution_name);
        bsx_datalog_algo(data_log_buf);

        if(accl_chip_name){
            sprintf(data_log_buf, "<device name=\"%s\" bus=\"IIC\">\n", accl_chip_name);
            bsx_datalog_algo(data_log_buf);
            sprintf(data_log_buf, "</device>\n");
            bsx_datalog_algo(data_log_buf);
        }

        if(magn_chip_name){
            sprintf(data_log_buf, "<device name=\"%s\" bus=\"\" >\n", magn_chip_name);
            bsx_datalog_algo(data_log_buf);
            sprintf(data_log_buf, "</device>\n");
            bsx_datalog_algo(data_log_buf);
        }

        if(gyro_chip_name){
            sprintf(data_log_buf, "<device name=\"%s\" bus=\"IIC\">\n", gyro_chip_name);
            bsx_datalog_algo(data_log_buf);
            sprintf(data_log_buf, "</device>\n\n");
            bsx_datalog_algo(data_log_buf);
        }
    }

    return;
}

static void driver_show_ver(const char* base_path)
{
    char path[64] = {0};
    char driver_info[64] = {0};
    FILE *fp;
    int ret;

    if(NULL == base_path)
    {
        PWARN("base_path is invalid");
        return;
    }

    sprintf(path, "%s/driver_version", base_path);

    fp = fopen(path, "r");
    if (NULL == fp)
    {
        PWARN("read driver version failed");
        return;
    }

    ret = fread(driver_info, 1, sizeof(driver_info), fp);
    fclose(fp);
    if(ret <= 0)
    {
        PERR("fread fail, errno = %d(%s)", errno, strerror(errno));
    }

    PNOTE("\n **************************\n \
* %s \
*****************************", driver_info);

    return;
}

static const char *iio_dir = "/sys/bus/iio/devices/";

static int32_t ap_hwcntl_init_ACC()
{
    int32_t ret = 0;
    int32_t accl_dev_num;
    char accl_dev_dir_name[128];
    char accl_buf_dir_name[128];
    char accl_buffer_access[128];
    const char *accl_device_name = NULL;

    if(ACC_CHIP_BMI160 == accl_chip)
    {
        accl_device_name = "bmi160_accl";

        /* Find the device requested */
        accl_dev_num = get_IIOnum_by_name(accl_device_name, iio_dir);
        if (accl_dev_num < 0)
        {
            PERR("Failed to find the %s\n", accl_device_name);
            return -ENODEV;
        }

        PDEBUG("iio device number being used is acc %d\n", accl_dev_num);
        sprintf(accl_dev_dir_name, "%siio:device%d", iio_dir, accl_dev_num);

        strcpy(iio_dev0_dir_name, accl_dev_dir_name);

        /*driver version only can be accessed in ACC attributes */
        driver_show_ver(accl_dev_dir_name);

        /* Construct the directory name for the associated buffer.*/
        sprintf(accl_buf_dir_name, "%siio:device%d/buffer", iio_dir, accl_dev_num);

        /* Setup ring buffer parameters */
        ret = wr_sysfs_oneint("length", accl_buf_dir_name, hwdata_unit_toread);
        if (ret < 0)
        {
            PERR("wr_sysfs_oneint() fail, ret = %d", ret);
            return 0;
        }

        ret = wr_sysfs_oneint("enable", accl_buf_dir_name, 1);
        if (ret < 0)
        {
            PERR("wr_sysfs_oneint() fail, ret = %d", ret);
            return 0;
        }

        accl_scan_size = 16;

        sprintf(accl_buffer_access, "/dev/iio:device%d", accl_dev_num);
        PDEBUG("accl_buffer_access: %s\n", accl_buffer_access);

        /* Attempt to open non blocking the access dev */
        accl_iio_fd = open(accl_buffer_access, O_RDONLY | O_NONBLOCK);
        if (accl_iio_fd == -1)
        {
            PERR("Failed to open %s\n", accl_buffer_access);
        }

        /*set BMI160 RANGE INT and WM*/
        ret = 0;

        switch(accl_range){
            case ACC_CHIP_RANGCONF_4G:
                ret += wr_sysfs_oneint("acc_range", iio_dev0_dir_name, BMI160_ACCEL_RANGE_4G);
                BMI160_acc_resl *= 2;
                break;
            case ACC_CHIP_RANGCONF_8G:
                ret += wr_sysfs_oneint("acc_range", iio_dev0_dir_name, BMI160_ACCEL_RANGE_8G);
                BMI160_acc_resl *= 4;
                break;
            case ACC_CHIP_RANGCONF_16G:
                ret += wr_sysfs_oneint("acc_range", iio_dev0_dir_name, BMI160_ACCEL_RANGE_16G);
                BMI160_acc_resl *= 8;
                break;
            default:
                break;
        }

        ret += wr_sysfs_twoint("enable_int", iio_dev0_dir_name, 13, 1);
        ret += wr_sysfs_oneint("fifo_watermark", iio_dev0_dir_name, default_watermark);
        if (ret < 0)
        {
            PERR("write_sysfs() fail, ret = %d", ret);
            return ret;
        }
    }else if(ACC_CHIP_BMA2x2 == accl_chip)
    {
        accl_device_name = "bma2x2";

        open_input_by_name(accl_device_name, &acc_input_fd, &acc_input_num);
        if (-1 == acc_input_fd)
        {
            PERR("Failed to open input event\n");
            return -ENODEV;
        }
        close(acc_input_fd);

        PDEBUG("acc input_num = %d", acc_input_num);
        sprintf(acc_input_dir_name, "/sys/class/input/input%d", acc_input_num);

        driver_show_ver(acc_input_dir_name);

        switch(accl_range){
            case ACC_CHIP_RANGCONF_4G:
                ret += wr_sysfs_oneint("range", acc_input_dir_name, BMA2X2_RANGE_4G);
                BMA255_acc_resl *= 2;
                break;
            case ACC_CHIP_RANGCONF_8G:
                ret += wr_sysfs_oneint("range", acc_input_dir_name, BMA2X2_RANGE_8G);
                BMA255_acc_resl *= 4;
                break;
            case ACC_CHIP_RANGCONF_16G:
                ret += wr_sysfs_oneint("range", acc_input_dir_name, BMA2X2_RANGE_16G);
                BMA255_acc_resl *= 8;
                break;
            default:
                break;
        }
    }

    return 0;
}


static int32_t ap_hwcntl_init_GYRO()
{
    int32_t ret;
    int32_t gyro_dev_num;
    char gyro_dev_dir_name[128];
    char gyro_buf_dir_name[128];
    char gyro_buffer_access[128];
    const char *gyro_device_name = NULL;

    if(GYR_CHIP_BMI160 == gyro_chip)
    {
        gyro_device_name = "bmi160_gyro";

        /* Find the device requested */
        gyro_dev_num = get_IIOnum_by_name(gyro_device_name, iio_dir);
        if (gyro_dev_num < 0)
        {
            PERR("Failed to find the %s\n", gyro_device_name);
            return -ENODEV;
        }

        PDEBUG("iio device number being used is gyr %d\n", gyro_dev_num);
        sprintf(gyro_dev_dir_name, "%siio:device%d", iio_dir, gyro_dev_num);

        /* Construct the directory name for the associated buffer.*/
        sprintf(gyro_buf_dir_name, "%siio:device%d/buffer", iio_dir, gyro_dev_num);

        /* Setup ring buffer parameters */
        ret = wr_sysfs_oneint("length", gyro_buf_dir_name, hwdata_unit_toread);
        if (ret < 0)
        {
            PERR("wr_sysfs_oneint() fail");
            return 0;
        }

        ret = wr_sysfs_oneint("enable", gyro_buf_dir_name, 1);
        if (ret < 0)
        {
            PERR("wr_sysfs_oneint() fail");
            return 0;
        }

        gyro_scan_size = 16;

        sprintf(gyro_buffer_access, "/dev/iio:device%d", gyro_dev_num);
        PDEBUG("gyro_buffer_access: %s\n", gyro_buffer_access);

        /* Attempt to open non blocking the access dev */
        gyro_iio_fd = open(gyro_buffer_access, O_RDONLY | O_NONBLOCK);
        if (gyro_iio_fd == -1)
        { /*If it isn't there make the node */
            PERR("Failed to open %s\n", gyro_buffer_access);
        }
    }else if(GYR_CHIP_BMG160 ==  gyro_chip)
    {
        gyro_device_name = "bmg160";

        open_input_by_name(gyro_device_name, &gyr_input_fd, &gyr_input_num);
        if (-1 == gyr_input_fd)
        {
            PERR("Failed to open input event\n");
            return -ENODEV;
        }
        close(gyr_input_fd);

        PDEBUG("gyr input_num = %d", gyr_input_num);
        sprintf(gyr_input_dir_name, "/sys/class/input/input%d", gyr_input_num);

    }

    return 0;

}


static int32_t ap_hwcntl_init_MAGN()
{
    int32_t ret;
    int32_t magn_dev_num;
    char magn_dev_dir_name[128];
    char magn_buf_dir_name[128];
    char magn_buffer_access[128];
    const char *magn_device_name = NULL;

    /* Find the device requested */
    if(MAG_CHIP_BMI160 == magn_chip)
    {
        magn_device_name = "bmi160_magn";

        magn_dev_num = get_IIOnum_by_name(magn_device_name, iio_dir);
        if (magn_dev_num < 0)
        {
            PERR("Failed to find the %s\n", magn_device_name);
            return -ENODEV;
        }

        PDEBUG("iio device number being used is mag %d\n", magn_dev_num);

        sprintf(magn_dev_dir_name, "%siio:device%d", iio_dir, magn_dev_num);

        /* Construct the directory name for the associated buffer.*/
        sprintf(magn_buf_dir_name, "%siio:device%d/buffer", iio_dir, magn_dev_num);

        /* Setup ring buffer parameters */
        ret = wr_sysfs_oneint("length", magn_buf_dir_name, hwdata_unit_toread);
        if (ret < 0)
        {
            PERR("wr_sysfs_oneint() fail");
            return 0;
        }

        ret = wr_sysfs_oneint("enable", magn_buf_dir_name, 1);
        if (ret < 0)
        {
            PERR("wr_sysfs_oneint() fail");
            return 0;
        }

        magn_scan_size = 16;

        sprintf(magn_buffer_access, "/dev/iio:device%d", magn_dev_num);
        PDEBUG("magn_buffer_access: %s\n", magn_buffer_access);

        /* Attempt to open non blocking the access dev */
        magn_iio_fd = open(magn_buffer_access, O_RDONLY | O_NONBLOCK);
        if (magn_iio_fd == -1)
        { /*If it isn't there make the node */
            PERR("Failed to open %s\n", magn_buffer_access);
        }

    }else if(MAG_CHIP_BMM150 == magn_chip || MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip ||
            MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip)
    {
        if(MAG_CHIP_BMM150 == magn_chip){
            magn_device_name = "bmm050";
        }else if(MAG_CHIP_AKM09912 == magn_chip){
            magn_device_name = "akm09912";
        }else if(MAG_CHIP_AKM09911 == magn_chip){
            magn_device_name = "akm09911";
        }else if(MAG_CHIP_YAS537 ==  magn_chip){
            magn_device_name = "yas537";
        }else if( MAG_CHIP_YAS532 ==  magn_chip){
            magn_device_name = "yas532";
        }

        open_input_by_name(magn_device_name, &magn_input_fd, &magn_input_num);
        if (-1 == magn_input_fd)
        {
            PERR("Failed to open input event\n");
            return -ENODEV;
        }
        close(magn_input_fd);

        PDEBUG("magn_input_num = %d", magn_input_num);
        sprintf(mag_input_dir_name, "/sys/class/input/input%d", magn_input_num);
    }

    return 0;

}


int32_t hwcntl_init(BstSensor *bstsensor)
{
    int32_t ret = 0;
    struct itimerspec timerspec;

    ap_show_ver();

    bstsensor->pfun_get_sensorlist = ap_get_sensorlist;
    bstsensor->pfun_activate = ap_activate;
    bstsensor->pfun_batch = ap_batch;
    bstsensor->pfun_flush = ap_flush;
    if(SOLUTION_MDOF == solution_type)
    {
        bstsensor->pfun_hw_deliver_sensordata = MDOF_hw_deliver_sensordata;
        ret = ap_hwcntl_init_ACC();
        ret = ap_hwcntl_init_GYRO();
        ret = ap_hwcntl_init_MAGN();
    }else if(SOLUTION_ECOMPASS == solution_type || SOLUTION_M4G == solution_type)
    {
        bstsensor->pfun_hw_deliver_sensordata = COMPASS_M4G_hw_deliver_sensordata;
        ret = ap_hwcntl_init_ACC();
        ret = ap_hwcntl_init_MAGN();
    }else if(SOLUTION_IMU == solution_type)
    {
        bstsensor->pfun_hw_deliver_sensordata = IMU_hw_deliver_sensordata;
        ret = ap_hwcntl_init_ACC();
        ret = ap_hwcntl_init_GYRO();
    }else if(SOLUTION_ACC == solution_type)
    {
        bstsensor->pfun_hw_deliver_sensordata = ACCONLY_hw_deliver_sensordata;
        ret = ap_hwcntl_init_ACC();
    }else
    {
        PERR("Unkown solution type: %d", solution_type);
    }

    if(ACC_CHIP_BMA2x2 == accl_chip || GYR_CHIP_BMG160 == gyro_chip ||
            (MAG_CHIP_BMM150 == magn_chip || MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip ||
                    MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip))
    {
        poll_timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
        if(-1 == poll_timer_fd)
        {
            PERR("Failed to create timer\n");
            return -ENODEV;
        }
    }

    if(ACC_CHIP_BMA2x2 == accl_chip)
    {
        timerspec.it_value.tv_sec = 0;
        timerspec.it_value.tv_nsec = MIN_TIMER_INTERVAL_ms * 1000000;
        timerspec.it_interval.tv_sec = timerspec.it_value.tv_sec;
        timerspec.it_interval.tv_nsec = timerspec.it_value.tv_nsec;
        ret = timerfd_settime(poll_timer_fd, 0, &timerspec, NULL);
    }


    return ret;
}

