/* Copyright (c) 2013-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef LOCATION_CALLBACKS_GARDEN_H
#define LOCATION_CALLBACKS_GARDEN_H

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef TEST_ULP
#include "ulp_service.h"
#endif

void test_garden_location_cb (LocGpsLocation * location);
void test_garden_status_cb (LocGpsStatus * status);
void test_garden_sv_status_cb (LocGpsSvStatus * sv_info);
void test_garden_nmea_cb (LocGpsUtcTime timestamp, const char *nmea, int length);
void test_garden_set_capabilities_cb (uint32_t capabilities);
void test_garden_xtra_time_req_cb ();
void test_garden_xtra_report_server_cb(const char* server1, const char* server2, const char* server3);
void test_garden_xtra_download_req_cb ();
void test_agps_status_cb (AGpsExtStatus * status);
void test_garden_ni_notify_cb(LocGpsNiNotification *notification);

void test_garden_acquire_wakelock_cb ();
void test_garden_release_wakelock_cb ();
pthread_t test_garden_create_thread_cb (const char *name, void (*start) (void *),
                                     void *arg);
#ifdef TEST_ULP
void test_ulp_request_phone_context_cb(UlpPhoneContextRequest *req);
void test_ulp_network_location_request_cb(UlpNetworkRequestPos* req);
#endif /* TEST_ULP */

#ifdef __cplusplus
}
#endif

#endif //LOCATION_CALLBACKS_GARDEN_H
