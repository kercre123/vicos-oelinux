//=============================================================================
// FILE: loc_srv_hal_to_mcm_type_conversions.h
//
// DESCRIPTION:
// Type conversions from HAL LOC data structures to MCM LOC data structures
//
// Copyright (c) 2013-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
// Qualcomm Technologies Proprietary and Confidential.
//=============================================================================

#ifndef LOC_SRV_HAL_TO_MCM_TYPE_CONVERSIONS_H
#define LOC_SRV_HAL_TO_MCM_TYPE_CONVERSIONS_H

#include "gps_extended_c.h"
#include "mcm_loc_v01.h"

#ifdef __cplusplus
extern "C" {
#endif

mcm_gps_position_mode_t_v01* loc_srv_conv_gps_position_mode(
                           LocGpsPositionMode *from,
                           mcm_gps_position_mode_t_v01 *to);

mcm_gps_position_recurrence_t_v01* loc_srv_conv_gps_position_recurrence(
                                 LocGpsPositionRecurrence *from,
                                 mcm_gps_position_recurrence_t_v01 *to);

mcm_gps_status_value_t_v01* loc_srv_conv_gps_status_value(LocGpsStatusValue *from,
                              mcm_gps_status_value_t_v01 *to);
mcm_gps_location_flag_t_v01* loc_srv_conv_gps_location_flags(
                                LocGpsLocationFlags *from,
                                mcm_gps_location_flag_t_v01 *to);

mcm_gps_aiding_data_t_v01* loc_srv_conv_gps_aiding_data(LocGpsAidingData *from,
                             mcm_gps_aiding_data_t_v01 *to);

mcm_agps_t_v01* loc_srv_conv_agps_type(AGpsExtType *from,
                                        mcm_agps_t_v01 *to);

mcm_agps_bearer_t_v01* loc_srv_conv_agps_bearer(AGpsBearerType *from,
                                               mcm_agps_bearer_t_v01 *to);

mcm_gps_ni_t_v01* loc_srv_conv_gps_ni(LocGpsNiType *from,
                                       mcm_gps_ni_t_v01 *to);

mcm_gps_ni_notify_flags_t_v01* loc_srv_conv_gps_ni_notify_flags(
                                 LocGpsNiNotifyFlags *from,
                                 mcm_gps_ni_notify_flags_t_v01 *to);

mcm_gps_user_response_t_v01* loc_srv_conv_gps_user_response(
                               LocGpsUserResponseType *from,
                               mcm_gps_user_response_t_v01 *to);
mcm_gps_ni_encoding_t_v01* loc_srv_conv_gps_ni_encoding(
                             LocGpsNiEncodingType *from,
                             mcm_gps_ni_encoding_t_v01 *to);

mcm_agps_status_value_t_v01* loc_srv_conv_agps_status_value(
                               LocAGpsStatusValue *from,
                               mcm_agps_status_value_t_v01 *to);

mcm_gps_location_t_v01* loc_srv_conv_gps_location(LocGpsLocation *from,
                                                mcm_gps_location_t_v01 *to);

mcm_gps_status_t_v01* loc_srv_conv_gps_status(LocGpsStatus *from,
                                                  mcm_gps_status_t_v01 *to);

mcm_gps_sv_info_t_v01* loc_srv_conv_gps_sv_info(LocGpsSvInfo *from,
                                               mcm_gps_sv_info_t_v01 *to);

mcm_gps_sv_status_t_v01* loc_srv_conv_gps_sv_status(LocGpsSvStatus *from,
                                               mcm_gps_sv_status_t_v01 *to);

mcm_agps_status_t_v01* loc_srv_conv_agps_status(AGpsExtStatus *from,
                                               mcm_agps_status_t_v01 *to);

mcm_gps_ni_notification_t_v01* loc_srv_conv_gps_ni_notification(
                                 LocGpsNiNotification *from,
                                 mcm_gps_ni_notification_t_v01 *to);

mcm_gps_capabilities_t_v01* loc_srv_conv_gps_capabilities(uint32_t *from,
                              mcm_gps_capabilities_t_v01 *to);

mcm_gps_position_source_t_v01* loc_srv_conv_gps_position_source(uint16_t *from,
                                 mcm_gps_position_source_t_v01 *to);

#ifdef __cplusplus
}
#endif

#endif // LOC_SRV_HAL_TO_MCM_TYPE_CONVERSIONS_H

