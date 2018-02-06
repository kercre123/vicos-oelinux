//=============================================================================
// FILE: loc_srv_mcm_to_hal_type_conversions.c
//
// DESCRIPTION:
// Type conversions from MCM LOC data structures to HAL LOC data structures
//
// Copyright (c) 2013-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
// Qualcomm Technologies Proprietary and Confidential.
//=============================================================================

#include <string.h>
#include "gps_extended_c.h"
#include "mcm_loc_v01.h"
#include "loc_srv_mcm_to_hal_type_conversions.h"

LocGpsPositionMode* loc_srv_conv_mcm_gps_position_mode(mcm_gps_position_mode_t_v01 *from,
                                       LocGpsPositionMode *to) {

    *to = (LocGpsPositionMode)*from;
    return to;
}

LocGpsPositionRecurrence* loc_srv_conv_mcm_gps_position_recurrence(
                                  mcm_gps_position_recurrence_t_v01 *from,
                                  LocGpsPositionRecurrence *to) {
    *to = (LocGpsPositionRecurrence)*from;
    return to;
}

LocGpsStatusValue* loc_srv_conv_mcm_gps_status_value(
                                     mcm_gps_status_value_t_v01 *from,
                                     LocGpsStatusValue *to) {

    *to = (LocGpsStatusValue)*from;
    return to;
}

LocGpsLocationFlags* loc_srv_conv_mcm_gps_location_flags(
                                  mcm_gps_location_flag_t_v01 *from,
                                  LocGpsLocationFlags *to) {

    *to = (LocGpsLocationFlags)*from;
    return to;
}

LocGpsAidingData* loc_srv_conv_mcm_gps_aiding_data(mcm_gps_aiding_data_t_v01 *from,
                                     LocGpsAidingData *to) {

    *to = (LocGpsAidingData)*from;
    return to;
}

AGpsExtType* loc_srv_conv_mcm_agps_type(mcm_agps_t_v01 *from, AGpsExtType *to) {

    *to = (AGpsExtType)*from;
    return to;
}

AGpsBearerType* loc_srv_conv_mcm_agps_bearer(mcm_agps_bearer_t_v01 *from,
                                 AGpsBearerType *to) {

    *to = (AGpsBearerType)*from;
    return to;
}

LocGpsNiType* loc_srv_conv_mcm_gps_ni(mcm_gps_ni_t_v01 *from, LocGpsNiType *to) {

    *to = (LocGpsNiType)*from;
    return to;
}

LocGpsNiNotifyFlags* loc_srv_conv_mcm_gps_ni_notify_flags(mcm_gps_ni_notify_flags_t_v01 *from,
                                         LocGpsNiNotifyFlags *to) {
    *to = (LocGpsNiNotifyFlags)*from;
    return to;
}

LocGpsUserResponseType* loc_srv_conv_mcm_gps_user_response(mcm_gps_user_response_t_v01 *from,
                                       LocGpsUserResponseType *to) {

    *to = (LocGpsUserResponseType)*from;
    return to;
}

LocGpsNiEncodingType* loc_srv_conv_mcm_gps_ni_encoding(mcm_gps_ni_encoding_t_v01 *from,
                                     LocGpsNiEncodingType *to) {
    *to = (LocGpsNiEncodingType)*from;
    return to;
}

LocAGpsStatusValue* loc_srv_conv_mcm_agps_status_value(mcm_agps_status_value_t_v01 *from,
                                       LocAGpsStatusValue *to) {

    *to = (LocAGpsStatusValue)*from;
    return to;
}

LocGpsLocation* loc_srv_conv_mcm_gps_location(mcm_gps_location_t_v01 *from,
                                  LocGpsLocation *to) {

    to->size = (size_t)sizeof(LocGpsLocation);
    loc_srv_conv_mcm_gps_location_flags(&(from->flags), &(to->flags));
    to->latitude = from->latitude;
    to->longitude = from->longitude;
    to->altitude = from->altitude;
    to->speed = from->speed;
    to->bearing = from->bearing;
    to->accuracy = from->accuracy;
    to->timestamp = from->timestamp;
    return to;
}

LocGpsStatus* loc_srv_conv_mcm_gps_status(mcm_gps_status_t_v01 *from, LocGpsStatus *to) {

    to->size = (size_t)sizeof(LocGpsStatus);
    loc_srv_conv_mcm_gps_status_value(&(from->status), &(to->status));
    return to;
}

LocGpsSvInfo* loc_srv_conv_mcm_gps_sv_info(mcm_gps_sv_info_t_v01 *from,
                                 LocGpsSvInfo  *to) {

    to->size = (size_t)sizeof(LocGpsSvInfo);
    to->prn = from->prn;
    to->snr = from->snr;
    to->elevation = from->elevation;
    to->azimuth = from->azimuth;
    return to;
}

LocGpsSvStatus* loc_srv_conv_mcm_gps_sv_status(mcm_gps_sv_status_t_v01 *from,
                                   LocGpsSvStatus *to) {
    int sv_list_index = 0;
    to->size = (size_t)sizeof(LocGpsSvStatus);
    to->num_svs = from->num_svs;
    for(; sv_list_index < LOC_GPS_MAX_SVS; ++sv_list_index) {
        loc_srv_conv_mcm_gps_sv_info(&(from->sv_list[sv_list_index]),
                            &(to->sv_list[sv_list_index]));
    }
    to->ephemeris_mask = from->ephemeris_mask;
    to->almanac_mask = from->almanac_mask;
    to->used_in_fix_mask = from->used_in_fix_mask;
    return to;
}

LocAGpsStatus* loc_srv_conv_mcm_agps_status(mcm_agps_status_t_v01 *from, AGpsExtStatus *to) {

    int index = 0;
    to->size = (size_t)sizeof(AGpsExtStatus);
    loc_srv_conv_mcm_agps_type(&(from->type), &(to->type));
    loc_srv_conv_mcm_agps_status_value(&(from->status), &(to->status));
    to->ipv4_addr = from->ipv4_addr;
    memcpy((void*)to->ssid, (const void*)from->ssid,
            SSID_BUF_SIZE);
    memcpy((void*)to->password, (const void*)from->password,
           SSID_BUF_SIZE);
    return to;
}

LocGpsNiNotification* loc_srv_conv_mcm_gps_ni_notification(mcm_gps_ni_notification_t_v01 *from,
                                         LocGpsNiNotification *to) {

    to->size = (size_t)sizeof(LocGpsNiNotification);
    to->notification_id = from->notification_id;
    loc_srv_conv_mcm_gps_ni(&(from->ni_type), &(to->ni_type));
    loc_srv_conv_mcm_gps_ni_notify_flags(&(from->notify_flags), &(to->notify_flags));
    to->timeout = from->timeout;
    loc_srv_conv_mcm_gps_user_response(&(from->default_response),
                                      &(to->default_response));
    memcpy((void*)to->requestor_id, (const void *)from->requestor_id,
           LOC_GPS_NI_SHORT_STRING_MAXLEN);
    memcpy((void*)to->text, (const void *)from->text,
           LOC_GPS_NI_LONG_STRING_MAXLEN);
    loc_srv_conv_mcm_gps_ni_encoding(&(from->requestor_id_encoding),
                            &(to->requestor_id_encoding));
    loc_srv_conv_mcm_gps_ni_encoding(&(from->text_encoding), &(to->text_encoding));

    memcpy((void*)to->extras, (const void *)from->extras,
           LOC_GPS_NI_LONG_STRING_MAXLEN);
    return to;
}

uint32_t* loc_srv_conv_mcm_gps_capabilities(mcm_gps_capabilities_t_v01 *from,
                                      uint32_t *to) {

    *to = (uint32_t)*from;
    return to;
}

uint16_t* loc_srv_conv_mcm_gps_position_source(mcm_gps_position_source_t_v01 *from,
                                      uint16_t *to) {

    *to = (uint16_t)*from;
    return to;
}


