/* Copyright (c) 2013-2014, Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#include <gps_extended_c.h>
#include <platform_lib_log_util.h>
#include <loc_vzw.h>

void nmea_callback (LocGpsUtcTime timestamp, const char *nmea, int length)
{
    LOC_LOGI ("test_gps_nmea_cb:");
}

void sv_status_callback(LocGpsSvStatus * sv_info, GpsDop* dop)
{
    LOC_LOGI ("test_gps_sv_status_cb:");
}

void status_callback(LocGpsStatus * status)
{
    LOC_LOGI ("test_gps_status_cb: %d", status->status);
}

void location_callback (LocGpsLocation * location, VzwGpsLocationExt* locExt)
{
    LOC_LOGI ("test_gps_location_cb: lat: %f, lon %f, acc %f, time %llu",
                 location->latitude, location->longitude, location->accuracy,
                 (long long) location->timestamp);
}
