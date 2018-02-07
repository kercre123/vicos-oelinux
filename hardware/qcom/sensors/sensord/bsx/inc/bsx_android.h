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

/*!@addtogroup bsx_integrationsupport
 * @{ */

#ifndef __BSX_ANDROID_H__
#define __BSX_ANDROID_H__

/*!
 * @brief Contains the android specific definitions to use with bsx_interface.h
 */

#ifdef __cplusplus
extern "C"
{
#endif

#include "bsx_datatypes.h"
#include "bsx_user_def.h"

/*! @brief Static table to map android sensor identifiers to the BSX specific virtual sensor identifiers */
extern const bsx_u16_t bsx_map_from_android_sensor_id[];

#ifndef ANDROID_SENSORS_INTERFACE_H

/*! @defgroup BSX Android specific definitions
 *  @brief includes android specific definitions to be used with bsx_interface.h
 *  @{
 */

#define SENSOR_TYPE_ACCELEROMETER                    (1U)   /**< accelerometer sensor*/
#define SENSOR_TYPE_GEOMAGNETIC_FIELD                (2U)   /**< geomagnetic field sensor*/
#define SENSOR_TYPE_MAGNETIC_FIELD                   SENSOR_TYPE_GEOMAGNETIC_FIELD  /**< magnetic field sensor*/
#define SENSOR_TYPE_ORIENTATION                      (3U)   /**< orientation sensor*/
#define SENSOR_TYPE_GYROSCOPE                        (4U)   /**< gyroscope sensor*/
#define SENSOR_TYPE_LIGHT                            (5U)   /**< light sensor*/
#define SENSOR_TYPE_PRESSURE                         (6U)   /**< pressure sensor*/
#define SENSOR_TYPE_TEMPERATURE                      (7U)   /**< temperature sensor*/
#define SENSOR_TYPE_PROXIMITY                        (8U)   /**< proximity sensor*/
#define SENSOR_TYPE_GRAVITY                          (9U)   /**< gravity sensor*/
#define SENSOR_TYPE_LINEAR_ACCELERATION             (10U)   /**< linear acceleration sensor*/
#define SENSOR_TYPE_ROTATION_VECTOR                 (11U)   /**< rotation vector sensor*/
#define SENSOR_TYPE_RELATIVE_HUMIDITY               (12U)   /**< relative humidity sensor*/
#define SENSOR_TYPE_AMBIENT_TEMPERATURE             (13U)   /**< ambient temperature sensor*/
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED     (14U)   /**< magnetic field uncalibrated sensor*/
#define SENSOR_TYPE_GAME_ROTATION_VECTOR            (15U)   /**< game rotation vector sensor*/
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED          (16U)   /**< gyroscope uncalibrated sensor*/
#define SENSOR_TYPE_SIGNIFICANT_MOTION              (17U)   /**< significant motion sensor*/
#define SENSOR_TYPE_STEP_DETECTOR                   (18U)   /**< step detector sensor*/
#define SENSOR_TYPE_STEP_COUNTER                    (19U)   /**< step counter sensor*/
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR     (20U)   /**< geomagnetic rotation vector sensor*/
#define SENSOR_TYPE_HEART_RATE                      (21U)   /**< heart rate sensor*/
#define SENSOR_TYPE_TILT_DETECTOR                   (22U)   /**< tilt detector sensor*/
#define SENSOR_TYPE_WAKE_GESTURE                    (23U)   /**< wake gesture sensor*/
#define SENSOR_TYPE_GLANCE_GESTURE                  (24U)   /**< glance gesture sensor*/
#define SENSOR_TYPE_PICK_UP_GESTURE                 (25U)   /**< pickup gesture sensor*/

/*! @} */

#endif // #ifndef ANDROID_SENSORS_INTERFACE_H

/*! @brief Offset for the identifier range of standard signals */
#define BSX_SENSOR_ID_OFFSET_STANDARD                        (0U)
/*! @brief Range for identifiers of standard signals */
#define BSX_SENSOR_ID_RANGE_STANDARD                          (32U)

/*! \brief Sensor identifiers linked to Android virtual sensor IDs
 */
typedef enum sensor_identifier_type
{
    BSX_SENSOR_ID_STANDARD_START                = BSX_SENSOR_ID_OFFSET_STANDARD, /**< identifier of sensor with standard, i.e. non-wakeup output signals */
    BSX_SENSOR_ID_ACCELEROMETER                 = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_ACCELEROMETER, /**< equalized (corrected) acceleration signal */
    BSX_SENSOR_ID_MAGNETIC_FIELD                = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_MAGNETIC_FIELD, /**< equalized (corrected) magnetic field signal */
    BSX_SENSOR_ID_ORIENTATION                   = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_ORIENTATION, /**< orientation signal providing Euler angles*/
    BSX_SENSOR_ID_GYROSCOPE                     = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_GYROSCOPE, /**< equalized (corrected) angular rate signal */
    BSX_SENSOR_ID_LIGHT                         = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_LIGHT, /**< light signal (not supported by motion fusion algorithms) */
    BSX_SENSOR_ID_PRESSURE                      = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_PRESSURE, /**< pressure signal */
    BSX_SENSOR_ID_TEMPERATURE                   = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_TEMPERATURE, /**< temperature signal */
    BSX_SENSOR_ID_PROXIMITY                     = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_PROXIMITY, /**< SENSOR_TYPE__PROXIMITY  */
    BSX_SENSOR_ID_GRAVITY                       = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_GRAVITY, /**< SENSOR_TYPE__GRAVITY  */
    BSX_SENSOR_ID_LINEAR_ACCELERATION           = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_LINEAR_ACCELERATION, /**< SENSOR_TYPE__LINEAR_ACCELERATION  */
    BSX_SENSOR_ID_ROTATION_VECTOR               = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_ROTATION_VECTOR, /**< SENSOR_TYPE__ROTATION_VECTOR  */
    BSX_SENSOR_ID_RELATIVE_HUMIDITY             = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_RELATIVE_HUMIDITY, /**< SENSOR_TYPE__RELATIVE_HUMIDITY  */
    BSX_SENSOR_ID_AMBIENT_TEMPERATURE           = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_AMBIENT_TEMPERATURE, /**< SENSOR_TYPE__AMBIENT_TEMPERATURE  */
    BSX_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED   = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED, /**< SENSOR_TYPE__MAGNETIC_FIELD_UNCALIBRATED  */
    BSX_SENSOR_ID_GAME_ROTATION_VECTOR          = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_GAME_ROTATION_VECTOR, /**< SENSOR_TYPE__GAME_ROTATION_VECTOR  */
    BSX_SENSOR_ID_GYROSCOPE_UNCALIBRATED        = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, /**< SENSOR_TYPE__GYROSCOPE_UNCALIBRATED  */
    BSX_SENSOR_ID_SIGNIFICANT_MOTION            = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_SIGNIFICANT_MOTION, /**< SENSOR_TYPE__SIGNIFICANT_MOTION (not used) */
    BSX_SENSOR_ID_STEP_DETECTOR                 = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_STEP_DETECTOR, /**< SENSOR_TYPE__STEP_DETECTOR  */
    BSX_SENSOR_ID_STEP_COUNTER                  = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_STEP_COUNTER, /**< SENSOR_TYPE__STEP_COUNTER  */
    BSX_SENSOR_ID_GEOMAGNETIC_ROTATION_VECTOR   = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR, /**< SENSOR_TYPE__GEOMAGNETIC_ROTATION_VECTOR  */
    BSX_SENSOR_ID_HEART_RATE                    = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_HEART_RATE, /**< SENSOR_TYPE__HEART_RATE  */
    BSX_SENSOR_ID_TILT_DETECTOR                 = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_TILT_DETECTOR, /**< SENSOR_TYPE__TILT_DETECTOR (not used) */
    BSX_SENSOR_ID_WAKE_GESTURE                  = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_WAKE_GESTURE, /**< SENSOR_TYPE__WAKE_GESTURE (not used) */
    BSX_SENSOR_ID_GLANCE_GESTURE                = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_GLANCE_GESTURE, /**< SENSOR_TYPE__GLANCE_GESTURE (not used) */
    BSX_SENSOR_ID_PICK_UP_GESTURE               = BSX_SENSOR_ID_STANDARD_START + SENSOR_TYPE_PICK_UP_GESTURE, /**< SENSOR_TYPE__PICK_UP_GESTURE (not used) */
    BSX_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED_OFFSET, /**< offset of SENSOR_TYPE__MAGNETIC_FIELD_UNCALIBRATED */
    BSX_SENSOR_ID_GYROSCOPE_UNCALIBRATED_OFFSET, /**< offset of SENSOR_TYPE__GYROSCOPE_UNCALIBRATED */
    BSX_SENSOR_ID_ACCELEROMETER_UNCALIBRATED_OFFSET, /**< offset of SENSOR_TYPE__GYROSCOPE_UNCALIBRATED */
    BSX_SENSOR_ID_RAW_ACCELEROMETER,
    BSX_SENSOR_ID_ACTIVITY,

    BSX_SENSOR_ID_WAKEUP_START                  = BSX_SENSOR_ID_RANGE_STANDARD,
    BSX_SENSOR_ID_ACCELEROMETER_WAKEUP          = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_ACCELEROMETER, /**< SENSOR_TYPE__ACCELEROMETER_WAKEUP  */
    BSX_SENSOR_ID_MAGNETIC_FIELD_WAKEUP         = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_MAGNETIC_FIELD, /**< SENSOR_TYPE__MAGNETIC_FIELD_WAKEUP */
    BSX_SENSOR_ID_ORIENTATION_WAKEUP            = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_ORIENTATION, /**< SENSOR_TYPE__ORIENTATION_WAKEUP  */
    BSX_SENSOR_ID_GYROSCOPE_WAKEUP              = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_GYROSCOPE, /**< SENSOR_TYPE__GYROSCOPE_WAKEUP  */
    BSX_SENSOR_ID_LIGHT_WAKEUP                  = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_LIGHT, /**< SENSOR_TYPE__LIGHT_WAKEUP  */
    BSX_SENSOR_ID_PRESSURE_WAKEUP               = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_PRESSURE, /**< SENSOR_TYPE__PRESSURE_WAKEUP  */
    BSX_SENSOR_ID_TEMPERATURE_WAKEUP            = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_TEMPERATURE, /**< SENSOR_TYPE__TEMPERATURE_WAKEUP  */
    BSX_SENSOR_ID_PROXIMITY_WAKEUP              = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_PROXIMITY, /**< SENSOR_TYPE__PROXIMITY_WAKEUP  */
    BSX_SENSOR_ID_GRAVITY_WAKEUP                = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_GRAVITY, /**< SENSOR_TYPE__GRAVITY_WAKEUP  */
    BSX_SENSOR_ID_LINEAR_ACCELERATION_WAKEUP    = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_LINEAR_ACCELERATION, /**< SENSOR_TYPE__LINEAR_ACCELERATION_WAKEUP  */
    BSX_SENSOR_ID_ROTATION_VECTOR_WAKEUP        = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_ROTATION_VECTOR, /**< SENSOR_TYPE__ROTATION_VECTOR_WAKEUP  */
    BSX_SENSOR_ID_RELATIVE_HUMIDITY_WAKEUP      = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_RELATIVE_HUMIDITY, /**< SENSOR_TYPE__RELATIVE_HUMIDITY_WAKEUP  */
    BSX_SENSOR_ID_AMBIENT_TEMPERATURE_WAKEUP    = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_AMBIENT_TEMPERATURE, /**< SENSOR_TYPE__AMBIENT_TEMPERATURE_WAKEUP  */
    BSX_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED_WAKEUP = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED, /**< SENSOR_TYPE__MAGNETIC_FIELD_UNCALIBRATED_WAKEUP  */
    BSX_SENSOR_ID_GAME_ROTATION_VECTOR_WAKEUP   = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_GAME_ROTATION_VECTOR, /**< SENSOR_TYPE__GAME_ROTATION_VECTOR_WAKEUP  */
    BSX_SENSOR_ID_GYROSCOPE_UNCALIBRATED_WAKEUP = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_GYROSCOPE_UNCALIBRATED, /**< SENSOR_TYPE__GYROSCOPE_UNCALIBRATED_WAKEUP  */
    BSX_SENSOR_ID_SIGNIFICANT_MOTION_WAKEUP     = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_SIGNIFICANT_MOTION, /**< SENSOR_TYPE__SIGNIFICANT_MOTION_WAKEUP */
    BSX_SENSOR_ID_STEP_DETECTOR_WAKEUP          = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_STEP_DETECTOR, /**< SENSOR_TYPE__STEP_DETECTOR_WAKEUP  */
    BSX_SENSOR_ID_STEP_COUNTER_WAKEUP           = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_STEP_COUNTER, /**< SENSOR_TYPE__STEP_COUNTER_WAKEUP  */
    BSX_SENSOR_ID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_GEOMAGNETIC_ROTATION_VECTOR, /**< SENSOR_TYPE__GEOMAGNETIC_ROTATION_VECTOR_WAKEUP  */
    BSX_SENSOR_ID_HEART_RATE_WAKEUP             = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_HEART_RATE, /**< SENSOR_TYPE__HEART_RATE_WAKEUP  */
    BSX_SENSOR_ID_TILT_DETECTOR_WAKEUP          = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_TILT_DETECTOR, /**< SENSOR_TYPE__TILT_DETECTOR_WAKEUP */
    BSX_SENSOR_ID_WAKE_GESTURE_WAKEUP           = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_WAKE_GESTURE, /**< SENSOR_TYPE__WAKE_GESTURE_WAKEUP */
    BSX_SENSOR_ID_GLANCE_GESTURE_WAKEUP         = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_GLANCE_GESTURE, /**< SENSOR_TYPE__GLANCE_GESTURE_WAKEUP */
    BSX_SENSOR_ID_PICK_UP_GESTURE_WAKEUP        = BSX_SENSOR_ID_WAKEUP_START + BSX_SENSOR_ID_PICK_UP_GESTURE, /**< SENSOR_TYPE__PICK_UP_GESTURE_WAKEUP */
    BSX_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED_OFFSET_WAKEUP, /**< offset of SENSOR_TYPE__MAGNETIC_FIELD_UNCALIBRATED */
    BSX_SENSOR_ID_GYROSCOPE_UNCALIBRATED_OFFSET_WAKEUP, /**< offset of SENSOR_TYPE__GYROSCOPE_UNCALIBRATED */
    BSX_SENSOR_ID_RAW_ACCELEROMETER_WAKEUP,

    BSX_SENSOR_ID_MAX,
    BSX_SENSOR_ID_INVALID = 255
} sensor_identifier_t;

#ifdef __cplusplus
}
#endif

#endif  /* __BSX_ANDROID_H__ */
/*! @}*/
