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

#ifndef BSX_RETURN_VALUE_IDENTIFIER_H_
#define BSX_RETURN_VALUE_IDENTIFIER_H_

/*! @brief Provides the enumeration containing return value identifier 
 *
 * @{
 */ 
/** @brief Normal operation was successful without any warning or additional information. */
#define BSX_OK   (0)

/** @brief Invalid input of doSteps. */
#define BSX_E_DOSTEPS_INVALIDINPUT   (-1)

/** @brief Value limits are exceeded. */
#define BSX_E_DOSTEPS_VALUELIMITS   (-2)

/** @brief Requested virtual sensor output is disabled. */
#define BSX_E_GET_DISABLED   (-3)

/** @brief No data is returned by get. */
#define BSX_E_GET_NODATA   (-4)

/** @brief Difference of timestamps between subsequent doSteps calls is out of range. */
#define BSX_E_DOSTEPS_TSINTRADIFFOUTOFRANGE   (-5)

/** @brief Input with same physical sensor identifier is given multiple times. */
#define BSX_E_DOSTEPS_DUPLICATEINPUT   (-6)

/** @brief Difference of timestamps for multiples sensors at same doStep call is out of range. */
#define BSX_E_DOSTEPS_TSINTERDIFFOUTOFRANGE   (-7)

/** @brief No outputs are returned by doSteps because no results processed. */
#define BSX_I_DOSTEPS_NOOUTPUTSRETURNED   (1)

/** @brief No outputs are returned by doSteps because no memory provided. */
#define BSX_I_DOSTEPS_NOOUTPUTSRETURNABLE   (2)

/** @brief More outputs can be returned but not enough memory provided. */
#define BSX_W_DOSTEPS_EXCESSOUTPUTS   (3)

/** @brief Invalid sample rate was given (sample rate for event based input, or event enabled for sample rate based). */
#define BSX_E_SU_WRONGDATARATE   (-10)

/** @brief Maximum number of iterations for update subscription reached. */
#define BSX_E_SU_ITERATIONSEXCEEDED   (-11)

/** @brief Sample rate is out of sample rate limits. */
#define BSX_E_SU_SAMPLERATELIMITS   (-12)

/** @brief Duplicate usage of virtual sensor in updateSubscription. */
#define BSX_E_SU_DUPLICATEGATE   (-13)

/** @brief Invalid sample rate was given to updateSubscription. */
#define BSX_E_SU_INVALIDSAMPLERATE   (-14)

/** @brief Number of configured virtual sensor exceeds number of outputs.  */
#define BSX_E_SU_GATECOUNTEXCEEDSARRAY   (-15)

/** @brief Unknown identifier for virtual sensor given to updateSubscription. */
#define BSX_W_SU_UNKNOWNOUTPUTGATE   (10)

/** @brief No output was given to updateSubscription. */
#define BSX_W_SU_NOOUTPUTGATE   (11)

/** @brief Subscribed output gates have been returned. */
#define BSX_I_SU_SUBSCRIBEDOUTPUTGATES   (12)

/** @brief Operation aborted.  */
#define BSX_E_ABORT   (-252)

/** @brief Library is in invalid state. Operation is not allowed. */
#define BSX_E_INVALIDSTATE   (-253)

/** @brief Fatal error. */
#define BSX_E_FATAL   (-254)

/** @brief Set-up failed. (deprecated) */
#define BSX_E_SETUP_FAIL   (-255)

/** @brief Length of section exceeds work buffer. */
#define BSX_E_PARSE_SECTIONEXCEEDSWORKBUFFER   (-32)

/** @brief Configuration failed. */
#define BSX_E_CONFIG_FAIL   (-33)

/** @brief Version of serialization does not match version of library. */
#define BSX_E_CONFIG_VERSIONMISMATCH   (-34)

/** @brief Feature set of serialization does not match features set of library. */
#define BSX_E_CONFIG_FEATUREMISMATCH   (-35)

/** @brief CRC of serialization is invalid. */
#define BSX_E_CONFIG_CRCMISMATCH   (-36)

/** @brief Serialization is empty. */
#define BSX_E_CONFIG_EMPTY   (-37)

/** @brief Insufficient work buffer for serialization. */
#define BSX_E_CONFIG_INSUFFICIENTWORKBUFFER   (-38)

/** @brief Invalid number of sections in serialization. */
#define BSX_E_CONFIG_INVALIDSECTIONCOUNT   (-39)

/** @brief Invalid length of serialization. */
#define BSX_E_CONFIG_INVALIDSTRINGSIZE   (-40)

/** @brief Insufficient buffer for serialization. */
#define BSX_E_CONFIG_INSUFFICIENTBUFFER   (-41)

/** @brief Serialized configuration/state is too large. */
#define BSX_E_CONFIG_STRINGTOOLARGE   (-42)

/** @brief Invalid channel identifier used in setting of configuration or state. */
#define BSX_E_SET_INVALIDCHANNELIDENTIFIER   (-100)

/** @brief Invalid property identifier used in setting of configuration or state. */
#define BSX_E_SET_INVALIDPROPERTY   (-101)

/** @brief Invalid function identifier. (deprecated) */
#define BSX_E_INVALIDFUNCTION   (-102)

/** @brief Invalid value. */
#define BSX_E_SET_INVALIDVALUE   (-103)

/** @brief Invalid length of serialization. */
#define BSX_E_SET_INVALIDLENGTH   (-104)

/** @brief Insufficient buffer for serialization. */
#define BSX_E_DUMP_INSUFFICIENTBUFFER   (-200)

/*!
 * @}
 */ 

#endif /* BSX_RETURN_VALUE_IDENTIFIER_H_ */

/*! @}
 */
