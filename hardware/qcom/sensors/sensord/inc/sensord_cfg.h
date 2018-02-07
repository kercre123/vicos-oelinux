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
 * @file         sensord_cfg.h
 * @date         "Tue Jan 26 11:14:07 2016 +0800"
 * @commit       "c6abe62"
 *
 * @brief
 *
 * @detail
 *
 */

#ifndef __SENSORD_CFG_H
#define __SENSORD_CFG_H

extern int g_place_a;
extern int g_place_m;
extern int g_place_g;
extern int solution_type;
extern int accl_chip;
extern int gyro_chip;
extern int magn_chip;
extern int accl_range;
extern int algo_pass;
extern int amsh_intr_pin;
extern int amsh_calibration;
extern int data_log;
extern int bsx_datalog;
extern int trace_level;
extern int trace_to_logcat;
extern long long unsigned int sensors_mask;

#define SOLUTION_MDOF       0
#define SOLUTION_ECOMPASS   1
#define SOLUTION_IMU        2
#define SOLUTION_M4G        3
#define SOLUTION_ACC        4
/* value is multiplexed with AP solutions, no conflicts*/
#define SOLUTION_BMA4xy_android     0
#define SOLUTION_BMA4xy_legacy      1
#define SOLUTION_BMA4xy_default     2

#define ACC_CHIP_BMI160      0
#define ACC_CHIP_BMA2x2      1
#define GYR_CHIP_BMI160      0
#define GYR_CHIP_BMG160      1
#define MAG_CHIP_BMI160      0
#define MAG_CHIP_AKM09912    1
#define MAG_CHIP_BMM150      2
#define MAG_CHIP_AKM09911    3
#define MAG_CHIP_YAS537      4
#define MAG_CHIP_YAS532      5

#define ACC_CHIP_RANGCONF_2G   2
#define ACC_CHIP_RANGCONF_4G   4
#define ACC_CHIP_RANGCONF_8G   8
#define ACC_CHIP_RANGCONF_16G  16

#endif
