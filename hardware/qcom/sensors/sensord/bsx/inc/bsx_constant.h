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

/*!@addtogroup bsx
 * @{*/

#ifndef __BSX_CONSTANT_H__
#define __BSX_CONSTANT_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*! @brief Physical constant for the average gravitational force on Earth */
#define BSX_CONSTANT_GRAVITY_STANDARD                   (9.80665f)
/*! @brief Mathematical constant for \f$ \pi \f$ */
#define BSX_CONSTANT_PI                                 (3.141592653589793f)
#define BSX_CONSTANT_PI_OVER_TWO                        (1.570796326794897f)   //!< \f$ pi/2 \f$
#define BSX_CONSTANT_PI_TIMES_TWO                       (6.283185307179586f)   //!< \f$ 2 \cdot pi \f$
#define BSX_CONSTANT_SQRT_TWO_OVER_TWO                  (0.7071067811865476f) //!< \f$ 0.5\cdot \sqrt(2)\f$
#define BSX_CONSTANT_UNIT_SCALING_RADIAN2DEGREE         (57.295779513082320876798154814105f)  //!< \f$ 180/pi \f$
#define BSX_CONSTANT_UNIT_SCALING_DEGREE2RADION         (0.01745329251994329576923690768489f)   //!< \f$ pi/180 \f$


/** @name Special values for sample rates
 *
 * @{
 */
/** @brief Special value for "disabled" signal that cannot or shall not provide any signal
 *
 * Sample rate value to define the operation mode "disabled". Not a valid sample interval for operation. */
#define BSX_SAMPLE_RATE_DISABLED       (float)(UINT16_MAX)
/** @brief Largest possible sample rate */
#define BSX_SAMPLE_RATE_MAX            (float)(UINT16_MAX - 16U)
/** @brief smallest possible sample rate */
#define BSX_SAMPLE_RATE_MIN           (1.0f)
/** @brief Sample rate value to define non-continuity of outputs of a module or a signal.
 *
 * Non-continuously sampled signals are called events. */
#define BSX_SAMPLE_RATE_EVENT         (0.0f)
/** @} */


/*! @brief conversion factors among internal number representation to physical number representation
 *
 * @note The constants provided within the following group shall be used only when the
 *       configuration applied to the fusion library does apply scaling to output values!
 *
 * @{
 */
#define BSX_CONSTANT_UNIT_SCALING_ACC_OUTPUT2G          (0.001f) //!< internal unit [mg] to [g]
#define BSX_CONSTANT_UNIT_SCALING_ACC_OUTPUT2MPS2       (BSX_CONSTANT_GRAVITY_STANDARD/1000.0f) //!< internal unit [mg] to [m/s^2]
#define BSX_CONSTANT_UNIT_SCALING_ACC_G2OUTPUT          (1000.0f) //!< [g] to internal unit [mg]
#define BSX_CONSTANT_UNIT_SCALING_ACC_MPS22OUTPUT       (1000.0f/BSX_CONSTANT_GRAVITY_STANDARD) //!< [m/s^2] to internal unit [mg]
#define BSX_CONSTANT_UNIT_SCALING_GYRO_OUTPUT2DEGPS     (0.06103515625f) //!< internal unit [0.061 deg/s] to [deg/s]; note: 2000/2^15 = 0.061
#define BSX_CONSTANT_UNIT_SCALING_GYRO_OUTPUT2RADPS     (0.001065264436031695f) //!< internal unit [0.0011 rad/s] to [deg/s]; note: 2000/2^15*pi/180 = 0.0011
#define BSX_CONSTANT_UNIT_SCALING_GYRO_DEGPS2OUTPUT     (16.384f) //!< [deg/s] to internal unit [0.061 deg/s]; note: 2^1/2000 = 16.384
#define BSX_CONSTANT_UNIT_SCALING_GYRO_RADPS2OUTPUT     (938.7340515423410f) //!< internal unit [0.0011 rad/s] to [deg/s]; note: 2000/2^15*pi/180 = 0.0011
#define BSX_CONSTANT_UNIT_SCALING_MAG_OUTPUT2UTESLA     (0.1f) //!< internal unit [0.1 uT] to [uT]
#define BSX_CONSTANT_UNIT_SCALING_MAG_UTESLA2OUTPUT     (10.0f) //!< [uT] to internal unit [0.1 uT]
/*! @} */

#ifdef __cplusplus
}
#endif

#endif /* __BSX_CONSTANT_H__ */

/*! @}*/
