/* 
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
 */

/*! @addtogroup bsx
 * @{
 */

#ifndef BSX_PROPERTY_SET_IDENTIFIER_H_
#define BSX_PROPERTY_SET_IDENTIFIER_H_

/*! @brief Provides the enumeration containing property set identifier 
 *
 * @{
 */ 
#define BSX_PROPERTY_SET_ID_FULL   (0)
#define BSX_PROPERTY_SET_ID_MINIMAL   (1)
#define BSX_PROPERTY_SET_ID_TYPICAL   (2)
/** @brief phone related values, e.g. SIC matrix, conversion factors for sensor resolution and range */
#define BSX_PROPERTY_SET_ID_CONDITIONING   (10)
#define BSX_PROPERTY_SET_ID_PREPROCESSING   (20)
#define BSX_PROPERTY_SET_ID_CALIBRATOR_ALL   (31)
/** @brief State of accel calibrator */
#define BSX_PROPERTY_SET_ID_CALIBRATOR_ACCELEROMETER   (32)
/** @brief state of gyro calibrator */
#define BSX_PROPERTY_SET_ID_CALIBRATOR_GYROSCOPE   (33)
/** @brief state of magnetometer */
#define BSX_PROPERTY_SET_ID_CALIBRATOR_MAGNETOMETER   (34)
/** @brief magnetometer related values, e.g. noise and offsets, for sensors such as BMM*, YAS*, AK* */
#define BSX_PROPERTY_SET_ID_MAGNETOMETER   (35)
/** @brief magnetometer related values, e.g. noise and offsets, for sensors such as BMM*, YAS*, AK* */
#define BSX_PROPERTY_SET_ID_MAGNETOMETER_CALIBRATION   (36)
#define BSX_PROPERTY_SET_ID_MAGNETOMETER_SOFT_IRON_EQUALIZATION   (37)
#define BSX_PROPERTY_SET_ID_MAGNETOMETER_INTERFACE   (38)
#define BSX_PROPERTY_SET_ID_DIRECTIONTRACKER_ALL   (41)
#define BSX_PROPERTY_SET_ID_DIRECTIONTRACKER_COMPASS   (42)
#define BSX_PROPERTY_SET_ID_DIRECTIONTRACKER_M4G   (43)
#define BSX_PROPERTY_SET_ID_DIRECTIONTRACKER_IMU   (44)
#define BSX_PROPERTY_SET_ID_DIRECTIONTRACKER_NDOF   (45)
#define BSX_PROPERTY_SET_ID_PREDEFINED_FILTERS_ALL   (51)
#define BSX_PROPERTY_SET_ID_PREDEFINED_FILTERS_COMPASS_HEADING_SENSITIVITY   (52)
#define BSX_PROPERTY_SET_ID_PREDEFINED_FILTERS_NDOF_ORIENTATION_CORRECTION_SPEED   (53)
#define BSX_PROPERTY_SET_ID_PREDEFINED_FILTERS_MAG_CALIB_ACC_SENSITIVITY   (54)
#define BSX_PROPERTY_SET_ID_PREDEFINED_FILTERS_MAG_CALIB_SPEED   (55)
#define BSX_PROPERTY_SET_ID_PREDEFINED_FILTERS_COMPASS_MAG_CALIB_ACC_AUTO_REC_MODE   (56)
#define BSX_PROPERTY_SET_ID_TURNOFF   (36)
#define BSX_PROPERTY_SET_ID_MIXED   (61)
#define BSX_PROPERTYSET_FULL                           BSX_PROPERTY_SET_ID_FULL
#define BSX_PROPERTYSET_CALIBRATOR_ACCELEROMETER       BSX_PROPERTY_SET_ID_CALIBRATOR_ACCELEROMETER
#define BSX_PROPERTYSET_CALIBRATOR_MAGNETOMETER        BSX_PROPERTY_SET_ID_CALIBRATOR_MAGNETOMETER
#define BSX_PROPERTYSET_CALIBRATOR_GYROSCOPE           BSX_PROPERTY_SET_ID_CALIBRATOR_GYROSCOPE
/*!
 * @}
 */ 

#endif /* BSX_PROPERTY_SET_IDENTIFIER_H_ */

/*! @}
 */
