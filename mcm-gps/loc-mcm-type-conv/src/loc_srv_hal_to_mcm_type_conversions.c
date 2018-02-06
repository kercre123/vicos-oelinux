//=============================================================================
// FILE: loc_srv_hal_to_mcm_type_conversions.c
//
// DESCRIPTION:
// Type conversions from HAL LOC data structures to MCM LOC data structures
//
// Copyright (c) 2013-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
// Qualcomm Technologies Proprietary and Confidential.
//=============================================================================

#include <string.h>
#include "mcm_loc_v01.h"
#include "loc_srv_hal_to_mcm_type_conversions.h"

mcm_gps_position_mode_t_v01* loc_srv_conv_gps_position_mode(LocGpsPositionMode *from,
                           mcm_gps_position_mode_t_v01 *to) {
    *to = (mcm_gps_position_mode_t_v01)*from;
    return to;
}

mcm_gps_position_recurrence_t_v01* loc_srv_conv_gps_position_recurrence(LocGpsPositionRecurrence *from,
                                 mcm_gps_position_recurrence_t_v01 *to) {
    *to = (mcm_gps_position_recurrence_t_v01)*from;
    return to;
}

mcm_gps_status_value_t_v01* loc_srv_conv_gps_status_value(LocGpsStatusValue *from,
                              mcm_gps_status_value_t_v01 *to) {
    *to = (mcm_gps_status_value_t_v01)*from;
    return to;
}
mcm_gps_location_flag_t_v01* loc_srv_conv_gps_location_flags(LocGpsLocationFlags *from,
                                mcm_gps_location_flag_t_v01 *to) {
    *to = (mcm_gps_location_flag_t_v01)*from;
    return to;
}

mcm_gps_aiding_data_t_v01* loc_srv_conv_gps_aiding_data(LocGpsAidingData *from,
                             mcm_gps_aiding_data_t_v01 *to) {
    *to = (mcm_gps_aiding_data_t_v01)*from;
    return to;
}

mcm_agps_t_v01* loc_srv_conv_agps_type(AGpsExtType *from, mcm_agps_t_v01 *to) {
    *to = (mcm_agps_t_v01)*from;
    return to;
}

mcm_agps_bearer_t_v01* loc_srv_conv_agps_bearer(AGpsBearerType *from,
                         mcm_agps_bearer_t_v01 *to) {
    *to = (mcm_agps_bearer_t_v01)*from;
    return to;
}

mcm_gps_ni_t_v01* loc_srv_conv_gps_ni(LocGpsNiType *from, mcm_gps_ni_t_v01 *to) {
    *to = (mcm_gps_ni_t_v01)*from;
    return to;
}

mcm_gps_ni_notify_flags_t_v01* loc_srv_conv_gps_ni_notify_flags(LocGpsNiNotifyFlags *from,
                                 mcm_gps_ni_notify_flags_t_v01 *to) {
    *to = (mcm_gps_ni_notify_flags_t_v01)*from;
    return to;
}
mcm_gps_user_response_t_v01* loc_srv_conv_gps_user_response(LocGpsUserResponseType *from,
                               mcm_gps_user_response_t_v01 *to) {
    *to = (mcm_gps_user_response_t_v01)*from;
    return to;
}
mcm_gps_ni_encoding_t_v01* loc_srv_conv_gps_ni_encoding(LocGpsNiEncodingType *from,
                             mcm_gps_ni_encoding_t_v01 *to) {
    *to = (mcm_gps_ni_encoding_t_v01)*from;
    return to;
}

mcm_agps_status_value_t_v01* loc_srv_conv_agps_status_value(LocAGpsStatusValue *from,
                               mcm_agps_status_value_t_v01 *to) {
    *to = (mcm_agps_status_value_t_v01)*from;
    return to;
}

mcm_gps_location_t_v01* loc_srv_conv_gps_location(LocGpsLocation *from, mcm_gps_location_t_v01 *to) {

    to->size = (uint32_t)sizeof(mcm_gps_location_t_v01);
    loc_srv_conv_gps_location_flags(&(from->flags), &(to->flags));
    to->position_source = MCM_LOC_ULP_LOCATION_IS_FROM_GNSS_V01;
    to->latitude = from->latitude;
    to->longitude = from->longitude;
    to->altitude = from->altitude;
    to->speed = from->speed;
    to->bearing = from->bearing;
    to->accuracy = from->accuracy;
    to->timestamp = from->timestamp;
    to->raw_data_len = 0;
    memset(to->raw_data, 0xFF, MCM_LOC_GPS_RAW_DATA_MAX_SIZE_CONST_V01);
    to->is_indoor = 0;
    to->floor_number = 0;
    to->map_url[0] = '/0';
    memset(to->map_index, 0xFF, MCM_LOC_GPS_LOCATION_MAP_INDEX_SIZE_CONST_V01);
    return to;
}


mcm_gps_status_t_v01* loc_srv_conv_gps_status(LocGpsStatus *from, mcm_gps_status_t_v01 *to) {

    to->size = (uint32_t)sizeof(mcm_gps_status_t_v01);
    loc_srv_conv_gps_status_value(&(from->status), &(to->status));
    return to;
}

mcm_gps_sv_info_t_v01* loc_srv_conv_gps_sv_info(LocGpsSvInfo *from, mcm_gps_sv_info_t_v01 *to) {

    to->size = (uint32_t)sizeof(mcm_gps_sv_info_t_v01);
    to->prn = from->prn;
    to->snr = from->snr;
    to->elevation = from->elevation;
    to->azimuth = from->azimuth;
    return to;
}

mcm_gps_sv_status_t_v01* loc_srv_conv_gps_sv_status(LocGpsSvStatus *from, mcm_gps_sv_status_t_v01 *to) {

    int sv_list_index = 0;
    to->size = (uint32_t)sizeof(mcm_gps_sv_status_t_v01);
    to->num_svs = from->num_svs;
    for(; sv_list_index<MCM_LOC_GPS_MAX_SVS_CONST_V01; sv_list_index++) {
        loc_srv_conv_gps_sv_info(&(from->sv_list[sv_list_index]),
                            &(to->sv_list[sv_list_index]));
    }
    to->ephemeris_mask = from->ephemeris_mask;
    to->almanac_mask = from->almanac_mask;
    to->used_in_fix_mask = from->used_in_fix_mask;
    return to;
}

mcm_agps_status_t_v01* loc_srv_conv_agps_status(AGpsExtStatus *from, mcm_agps_status_t_v01 *to) {

    int index = 0;
    to->size = (uint32_t)sizeof(mcm_agps_status_t_v01);
    loc_srv_conv_agps_type(&(from->type), &(to->type));
    loc_srv_conv_agps_status_value(&(from->status), &(to->status));
    to->ipv4_addr = from->ipv4_addr;
    memcpy((void*)to->ssid, (const void*)from->ssid,
            MCM_LOC_GPS_SSID_BUF_SIZE_CONST_V01 + 1);
    memcpy((void*)to->password, (const void*)from->password,
           MCM_LOC_GPS_SSID_BUF_SIZE_CONST_V01 + 1);
    return to;
}

mcm_gps_ni_notification_t_v01* loc_srv_conv_gps_ni_notification(LocGpsNiNotification *from,
                                 mcm_gps_ni_notification_t_v01 *to) {

    to->size = (uint32_t)sizeof(mcm_gps_ni_notification_t_v01);
    to->notification_id = from->notification_id;
    loc_srv_conv_gps_ni(&(from->ni_type), &(to->ni_type));
    loc_srv_conv_gps_ni_notify_flags(&(from->notify_flags), &(to->notify_flags));
    to->timeout = from->timeout;
    loc_srv_conv_gps_user_response(&(from->default_response),
                              &(to->default_response));
    memcpy((void*)to->requestor_id, (const void *)from->requestor_id,
           MCM_LOC_GPS_NI_SHORT_STRING_MAXLEN_CONST_V01 + 1);
    memcpy((void*)to->text, (const void *)from->text,
           MCM_LOC_GPS_NI_LONG_STRING_MAXLEN_CONST_V01 + 1);
    loc_srv_conv_gps_ni_encoding(&(from->requestor_id_encoding),
                            &(to->requestor_id_encoding));
    loc_srv_conv_gps_ni_encoding(&(from->text_encoding), &(to->text_encoding));

    memcpy((void*)to->extras, (const void *)from->extras,
           MCM_LOC_GPS_NI_LONG_STRING_MAXLEN_CONST_V01 + 1);
    return to;
}

mcm_gps_capabilities_t_v01* loc_srv_conv_gps_capabilities(uint32_t *from,
                              mcm_gps_capabilities_t_v01 *to) {

    *to = (mcm_gps_capabilities_t_v01)*from;
    return to;
}

mcm_gps_position_source_t_v01* loc_srv_conv_gps_position_source(uint16_t *from,
                                 mcm_gps_position_source_t_v01 *to) {
    *to = (mcm_gps_position_source_t_v01)*from;
    return to;
}
