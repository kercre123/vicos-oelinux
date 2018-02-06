/*****************************************************************************
  Copyright (c) 2016 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
*****************************************************************************/

#include <loc_gps.h>
#include "FlpLocationClient.h"
#include "GeoFencer.h"
#include "GpsGeofenceCb.h"
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <inttypes.h>
#include <map>
#include <errno.h>

#define LOG_TAG "LocSvc_test"

#define LOC_FLP_BATCH_WAKEUP_ON_FIFO_FULL        0x0000001
#define LOC_FLP_BATCH_CALLBACK_ON_LOCATION_FIX   0x0000002

static FlpLocationClient* sFlpLocationClient = NULL;
static GeoFencer* sGeoFencer = NULL;

static const LocGpsInterface* sGpsInterface = NULL;
static const LocAGpsInterface* sAGpsInterface = NULL;

typedef struct {
    double latitude;
    double longitude;
    double radius_m;
} TestGeofenceData;

typedef std::map<int32_t, TestGeofenceData> TestGeoFencesMap;
static TestGeoFencesMap sGpsGeoFencesRunning;
static TestGeoFencesMap sGpsGeoFencesPaused;

enum GeoFenceCommands {
    ADD_COMMAND = 0,
    PAUSE_COMMAND = 1,
    RESUME_COMMAND = 2,
    MODIFY_COMMAND = 3,
    REMOVE_COMMAND = 4,
};

bool gGpsGeofenceAddCallbackCalled = false;
bool gGpsGeofenceRemoveCallbackCalled = false;
bool gGpsGeofencePauseCallbackCalled = false;
bool gGpsGeofenceResumeCallbackCalled = false;
pthread_cond_t gCondGeofence = PTHREAD_COND_INITIALIZER;
pthread_mutex_t gMutexGeofence = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t gCondLocation = PTHREAD_COND_INITIALIZER;
pthread_mutex_t gMutexLocation = PTHREAD_MUTEX_INITIALIZER;

typedef void (*ThreadStart) (void *);
ThreadStart g_pfnThreadStart;

extern "C" const LocGpsInterface *loc_eng_gps_get_hardware_interface();

// Command Line arguments
bool gGpsVerifyTestInProgress = false;
int gStopOnMinSvs = 0; // part of conditions that cause test to stop. Minimum number of SVs
float gStopOnMinSnr = 0; // part of conditions that casue test to stop. Minimum number of SNR
bool gFlpBatchingVerify = false;

void *my_thread_fn (void *arg)
{
    g_pfnThreadStart (arg);
    return NULL;
}

pthread_t create_thread_callback(const char *name, void (*start) (void *), void *arg)
{
    pthread_t thread_id;
    printf("%s ", name);
    LOC_LOGW("%s ", name);
    g_pfnThreadStart = start;

    if (0 > pthread_create (&thread_id, NULL, my_thread_fn, arg)) {
        printf("error creating thread\n");
        LOC_LOGW("error creating thread");
    } else {
        printf("created thread\n");
        LOC_LOGW("created thread");
    }

    return thread_id;
}

static void gps_loc_location_callback(LocGpsLocation* location)
{
    printf("EVENT LOCATION_CALLBACK\n");
    LOC_LOGW("EVENT LOCATION_CALLBACK");
    printf("LAT: %8.4f LON: %9.4f ACC: %8.2f TIME: %13lld\n",
          location->latitude, location->longitude, location->accuracy, location->timestamp);
    LOC_LOGW("LAT: %8.4f LON: %9.4f ACC: %8.2f TIME: %13lld",
          location->latitude, location->longitude, location->accuracy, location->timestamp);

    if (gGpsVerifyTestInProgress)
    {
        printf("GPS VERIFY TEST SUCCESSFUL\n");
        LOC_LOGW("GPS VERIFY TEST SUCCESSFUL");
        pthread_mutex_lock(&gMutexLocation);
        pthread_cond_signal(&gCondLocation);
        pthread_mutex_unlock(&gMutexLocation);
    }

}

static void gps_loc_status_callback(LocGpsStatus* status)
{
    printf("EVENT GPS STATUS_CALLBACK status=%d\n", status->status);
    LOC_LOGW("EVENT GPS STATUS_CALLBACK status=%d", status->status);
}

static void gps_loc_sv_status_callback(LocGpsSvStatus* sv_info)
{
    printf("EVENT GPS SV_STATUS_CALLBACK Svs=%d\n", sv_info->num_svs);
    LOC_LOGW("EVENT GPS SV_STATUS_CALLBACK Svs=%d", sv_info->num_svs);

    if (gGpsVerifyTestInProgress) {
        for(int i=0;i<sv_info->num_svs; ++i)
        {
            printf("%02d : PRN: %04d, SNR: %09.4f, ELE: %09.4f, AZI: %09.4f\n",
                          i+1,sv_info->sv_list[i].prn,
                              sv_info->sv_list[i].snr,
                              sv_info->sv_list[i].elevation,
                              sv_info->sv_list[i].azimuth);
            LOC_LOGW("%02d : PRN: %04d, SNR: %09.4f, ELE: %09.4f, AZI: %09.4f",
                          i+1,sv_info->sv_list[i].prn,
                              sv_info->sv_list[i].snr,
                              sv_info->sv_list[i].elevation,
                              sv_info->sv_list[i].azimuth);
        }
        printf("Ephemeris Mask : 0x%X, ",sv_info->ephemeris_mask);
        LOC_LOGW("Ephemeris Mask : 0x%X",sv_info->ephemeris_mask);
        printf("Almanac Mask: 0x%X, ",sv_info->almanac_mask);
        LOC_LOGW("Almanac Mask: 0x%X",sv_info->almanac_mask);
        printf("Used in Fix Mask: 0x%X:\n",sv_info->used_in_fix_mask);
        LOC_LOGW("Used in Fix Mask: 0x%X:",sv_info->used_in_fix_mask);

        // if the minimum number of SVs with minimum number of SNR
        // has been satisfied then stop session
        if (gStopOnMinSvs > 0 && gStopOnMinSnr > 0) {
            if (gStopOnMinSvs <= sv_info->num_svs) {
                int minCnSvCount = 0;
                // count number of SVs that meet the min SNR
                for(int i=0; i<sv_info->num_svs; ++i) {
                    if (sv_info->sv_list[i].snr >= gStopOnMinSnr) {
                        minCnSvCount++;
                    }
                }
                if (minCnSvCount >= gStopOnMinSvs){
                    printf("GPS VERIFY TEST SUCCESSFUL, as %d SVs are seen with at least a SNR of %f\n",
                              gStopOnMinSvs, gStopOnMinSnr);
                    LOC_LOGW("GPS VERIFY TEST SUCCESSFUL, as %d SVs are seen with at least a SNR of %f",
                              gStopOnMinSvs, gStopOnMinSnr);
                    pthread_mutex_lock(&gMutexLocation);
                    pthread_cond_signal(&gCondLocation);
                    pthread_mutex_unlock(&gMutexLocation);
                }
            }
        }
    }

}

static void gps_loc_nmea_callback(LocGpsUtcTime timestamp, const char* nmea, int length)
{
    //printf("EVENT NMEA_CALLBACK\n");
    //LOC_LOGW("EVENT NMEA_CALLBACK");
}

static void gps_loc_set_capabilities_callback(uint32_t capabilities)
{
    printf("EVENT GPS SET_CAPABILITIES_CALLBACK\n");
    LOC_LOGW("EVENT GPS SET_CAPABILITIES_CALLBACK");
}

static void gps_loc_acquire_wakelock_callback()
{
    printf("EVENT GPS ACQUIRE_WAKELOCK_CALLBACK\n");
    LOC_LOGW("EVENT GPS ACQUIRE_WAKELOCK_CALLBACK");
}

static void gps_loc_release_wakelock_callback()
{
    printf("EVENT GPS RELEASE_WAKELOCK_CALLBACK\n");
    LOC_LOGW("EVENT GPS RELEASE_WAKELOCK_CALLBACK");
}

static void gps_loc_request_utc_time_callback()
{
    printf("EVENT GPS REQUEST_UTC_TIME_CALLBACK\n");
    LOC_LOGW("EVENT GPS REQUEST_UTC_TIME_CALLBACK");
}

static void agps_loc_status_callback(LocAGpsStatus* status)
{
    printf("EVENT AGPS STATUS %d\n", status->status);
    LOC_LOGW("EVENT AGPS STATUS %d", status->status);
}

LocGpsCallbacks sGpsCallbacks = {
    sizeof(LocGpsCallbacks),
    gps_loc_location_callback,
    gps_loc_status_callback,
    gps_loc_sv_status_callback,
    gps_loc_nmea_callback,
    gps_loc_set_capabilities_callback,
    gps_loc_acquire_wakelock_callback,
    gps_loc_release_wakelock_callback,
    create_thread_callback,
    gps_loc_request_utc_time_callback
};

LocAGpsCallbacks sAGpsCallbacks = {
    agps_loc_status_callback,
    create_thread_callback,
};

static void flp_loc_location_callback(int32_t num_locations,
    FlpExtLocation** locations,
    LocReportType reportTrigger) {

    printf("EVENT FLP LOCATION num_locations=%d\n", num_locations);
    LOC_LOGW("EVENT FLP LOCATION num_locations=%d", num_locations);

    if (num_locations <= 0)
        return;

    printf(" flags | latitude | longitude | altitude | accuracy | speed | bearing |     time      | sources \n");
    LOC_LOGW(" flags | latitude | longitude | altitude | accuracy | speed | bearing |     time      | sources ");

    for (int i=0; i<num_locations; ++i) {
        printf("%6d | %8.4f | %9.4f | %8.2f | %8.2f | %5.2f | %7.2f | %13lld | %d\n",
              locations[i]->flags, locations[i]->latitude, locations[i]->longitude, locations[i]->altitude,
              locations[i]->accuracy, locations[i]->speed, locations[i]->bearing, locations[i]->timestamp,
              locations[i]->sources_used);
        LOC_LOGW("%6d | %8.4f | %9.4f | %8.2f | %8.2f | %5.2f | %7.2f | %13lld | %d",
              locations[i]->flags, locations[i]->latitude, locations[i]->longitude, locations[i]->altitude,
              locations[i]->accuracy, locations[i]->speed, locations[i]->bearing, locations[i]->timestamp,
              locations[i]->sources_used);
    }
}

static void flp_loc_acquire_wakelock() {

    printf("EVENT FLP ACQUIRE_WAKELOCK\n");
    LOC_LOGW("EVENT FLP ACQUIRE_WAKELOCK");
}

static void flp_loc_release_wakelock() {

    printf("EVENT FLP RELEASE_WAKELOCK\n");
    LOC_LOGW("EVENT FLP RELEASE_WAKELOCK");
}

static int flp_loc_set_thread_event(FlpExtThreadEvent event) {

    printf("EVENT FLP SET THREAD EVENT\n");
    LOC_LOGW("EVENT FLP SET THREAD EVENT");
    return FLP_SUCCESS;
}

static void flp_loc_capabilities_callback(int capabilities) {
    printf("EVENT FLP_CAPABILITIES = %d ", capabilities);
    LOC_LOGW("EVENT FLP_CAPABILITIES = %d ", capabilities);
    if (capabilities & LOC_CAPABILITY_GNSS) {
        printf("GNSS ");
        LOC_LOGW("GNSS ");
    }
    if (capabilities & LOC_CAPABILITY_WIFI) {
        printf("WIFI ");
        LOC_LOGW("WIFI ");
    }
    if (capabilities & LOC_CAPABILITY_CELL) {
        printf("CELL ");
        LOC_LOGW("CELL ");
    }
    printf("\n");

}

static void flp_loc_status_callback(int32_t status) {
    if (LOC_FLP_STATUS_LOCATION_AVAILABLE == status) {
        printf("EVENT FLP_STATUS = AVAILABLE\n");
        LOC_LOGW("EVENT FLP_STATUS = AVAILABLE");
    } else if (LOC_FLP_STATUS_LOCATION_UNAVAILABLE == status) {
        printf("EVENT FLP_STATUS = AVAILABLE\n");
        LOC_LOGW("EVENT FLP_STATUS = AVAILABLE");
    } else {
        printf("EVENT FLP_STATUS = INVALID STATUS!\n");
        LOC_LOGE("EVENT FLP_STATUS = INVALID_STATUS!");
    }
}

static void flp_loc_max_power_allocated_callback(double power_mW) {
    printf("flp_loc_max_power_allocated_callback!\n");
    LOC_LOGE("flp_loc_max_power_allocated_callback!");
}

static void geofence_transition_callback(int32_t geofence_id, LocGpsLocation* location,
                                         int32_t transition, LocGpsUtcTime timestamp) {
    printf("EVENT GPS GEOFENCE TRANSITION id=%d transition=%d lat=%8.2f long=%8.2f accuracy=%8.2f speed=%8.2f bearing=%8.2f time=%lld\n",
              geofence_id, transition, location->latitude, location->longitude, location->accuracy, location->speed, location->bearing, timestamp);
    LOC_LOGW("EVENT GPS GEOFENCE TRANSITION id=%d transition=%d lat=%8.2f long=%8.2f accuracy=%8.2f speed=%8.2f bearing=%8.2f time=%lld",
              geofence_id, transition, location->latitude, location->longitude, location->accuracy, location->speed, location->bearing, timestamp);

}

static void geofence_status_callback(int32_t status, LocGpsLocation* last_location) {

    printf("EVENT GPS GEOFENCE STATUS status=%d\n", status);
    LOC_LOGW("EVENT GPS GEOFENCE STATUS status=%d", status);
}

static void geofence_add_callback(int32_t geofence_id, int32_t status) {

    printf("EVENT GPS GEOFENCE ADD geofence_id=%d status=%d\n", geofence_id, status);
    LOC_LOGW("EVENT GPS GEOFENCE ADD geofence_id=%d status=%d", geofence_id, status);

    pthread_mutex_lock(&gMutexGeofence);
    gGpsGeofenceAddCallbackCalled = true;
    pthread_cond_broadcast(&gCondGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

static void geofence_remove_callback(int32_t geofence_id, int32_t status) {

    printf("EVENT GPS GEOFENCE REMOVE geofence_id=%d status=%d\n", geofence_id, status);
    LOC_LOGW("EVENT GPS GEOFENCE REMOVE geofence_id=%d status=%d", geofence_id, status);

    pthread_mutex_lock(&gMutexGeofence);
    gGpsGeofenceRemoveCallbackCalled = true;
    pthread_cond_broadcast(&gCondGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

static void geofence_pause_callback(int32_t geofence_id, int32_t status) {

    printf("EVENT GPS GEOFENCE PAUSE geofence_id=%d status=%d\n", geofence_id, status);
    LOC_LOGW("EVENT GPS GEOFENCE PAUSE geofence_id=%d status=%d", geofence_id, status);

    pthread_mutex_lock(&gMutexGeofence);
    gGpsGeofencePauseCallbackCalled = true;
    pthread_cond_broadcast(&gCondGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

static void geofence_resume_callback(int32_t geofence_id, int32_t status) {

    printf("EVENT GPS GEOFENCE RESUME geofence_id=%d status=%d\n", geofence_id, status);
    LOC_LOGW("EVENT GPS GEOFENCE RESUME geofence_id=%d status=%d", geofence_id, status);

    pthread_mutex_lock(&gMutexGeofence);
    gGpsGeofenceResumeCallbackCalled = true;
    pthread_cond_broadcast(&gCondGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

LocGpsGeofenceCallbacks sGpsGeofenceCallbacks = {
    geofence_transition_callback,
    geofence_status_callback,
    geofence_add_callback,
    geofence_remove_callback,
    geofence_pause_callback,
    geofence_resume_callback,
    create_thread_callback
};

FlpExtCallbacks sFlpCallbacks = {
    sizeof(FlpExtCallbacks),
    flp_loc_location_callback,
    flp_loc_acquire_wakelock,
    flp_loc_release_wakelock,
    flp_loc_set_thread_event,
    flp_loc_capabilities_callback,
    flp_loc_status_callback,
    flp_loc_max_power_allocated_callback
};

int init()
{
    int ret;

    sGpsInterface = loc_eng_gps_get_hardware_interface();

    if (sGpsInterface) {
        ret = sGpsInterface->init(&sGpsCallbacks);
        if (ret) {
            printf("sGpsInterface->init returned: %i\n", ret);
            LOC_LOGE("sGpsInterface->init returned: %i", ret);
        }
        sAGpsInterface =
            (const LocAGpsInterface*)sGpsInterface->get_extension(LOC_AGPS_INTERFACE);

        if (sAGpsInterface) {
            sAGpsInterface->init(&sAGpsCallbacks);
        }
    }

    if (sGeoFencer == NULL) {
        GpsGeofenceCb* gpsCallBacks = new GpsGeofenceCb(&sGpsGeofenceCallbacks);
        sGeoFencer = new GeoFencer(gpsCallBacks, create_thread_callback);
    }

    if (sFlpLocationClient == NULL) {
        sFlpLocationClient = FlpLocationClient::createInstance();
    }

    if (sFlpLocationClient) {
        ret = sFlpLocationClient->flp_init(&sFlpCallbacks);
    }
    else {
        printf("sFlpLocationClient is null\n");
        LOC_LOGE("sFlpLocationClient is null");
    }

    return ret;
}

float randomFloat(float low, float high)
{
    return (low + (float)rand() / ((float)RAND_MAX / (high - low)));
}

int randomInt(int low, int high)
{
    return(low + (rand() % (high - low + 1)));
}

void addRandomGeofence()
{
    int32_t geofence_id;
    int monitor_transitions;
    int responsiveness_ms;
    TestGeofenceData tGeofenceData;
    TestGeoFencesMap::iterator it;
    TestGeoFencesMap& runningMap(sGpsGeoFencesRunning);

    do {
        geofence_id = randomInt(0, 4096);
        it = runningMap.find(geofence_id);
    } while (it != runningMap.end());

    tGeofenceData.latitude = randomFloat(-90, 90);
    tGeofenceData.longitude = randomFloat(-180, 180);
    tGeofenceData.radius_m = randomFloat(1, 1000);
    monitor_transitions = LOC_GPS_GEOFENCE_ENTERED |
        LOC_GPS_GEOFENCE_EXITED |
        LOC_GPS_GEOFENCE_UNCERTAIN;
        responsiveness_ms = randomInt(1, 0x40000);

    runningMap[geofence_id] = tGeofenceData;

    printf("\nADD GPS GEOFENCE -> id=%2u | lat=%8.2f | lon=%8.2f | rad=%2.2f | resp=%d(ms)\n",
           geofence_id, tGeofenceData.latitude,
           tGeofenceData.longitude, tGeofenceData.radius_m,
            responsiveness_ms);
    LOC_LOGW("ADD GPS GEOFENCE -> id=%2u | lat=%8.2f | lon=%8.2f | rad=%2.2f | resp=%d(ms)",
        geofence_id, tGeofenceData.latitude,
        tGeofenceData.longitude, tGeofenceData.radius_m,
        responsiveness_ms);

    gGpsGeofenceAddCallbackCalled = false;
    if (sGeoFencer) {
        sGeoFencer->addCommand(geofence_id,
            tGeofenceData.latitude,
            tGeofenceData.longitude,
            tGeofenceData.radius_m,
            LOC_GPS_GEOFENCE_UNCERTAIN,
            monitor_transitions,
            responsiveness_ms,
            0,
            0,
            0,
            0);
    }
    else {
        printf("sGeoFencer is NULL in addRandomGeofence");
        LOC_LOGE("sGeoFencer is NULL in addRandomGeofence");
        gGpsGeofenceAddCallbackCalled = true;
        return;
    }
    pthread_mutex_lock(&gMutexGeofence);
    printf("Waiting on Callback...\n");
    while (gGpsGeofenceAddCallbackCalled == false)
        pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

void removeRandomGeofence()
{
    TestGeoFencesMap& runningMap(sGpsGeoFencesRunning);
    TestGeoFencesMap& pausedMap(sGpsGeoFencesPaused);

    bool removeRunningGeofence = true;
    if (!runningMap.empty() && !pausedMap.empty())
    {
        removeRunningGeofence = randomInt(0, 1);
    }
    else if (!runningMap.empty())
    {
        removeRunningGeofence = true;
    }
    else if (!pausedMap.empty())
    {
        removeRunningGeofence = false;
    }
    else
    {
        return;
    }

    if (removeRunningGeofence)
    {
        int index = randomInt(0, runningMap.size() - 1);
        TestGeoFencesMap::iterator it = runningMap.begin();

        for (int i = 0; i < index; ++i)
            it++;

        int32_t geofence_id = it->first;

        runningMap.erase(it);

        printf("REMOVE-RUNNING GPS GEOFENCE -> %5u \n", geofence_id);
        LOC_LOGW("REMOVE-RUNNING GPS GEOFENCE -> %5u ", geofence_id);

        gGpsGeofenceRemoveCallbackCalled = false;
        if (sGeoFencer) {
            sGeoFencer->removeCommand(geofence_id);
        }
        else {
            printf("sGeoFencer is NULL in removeRandomGeofence");
            LOC_LOGE("sGeoFencer is NULL in removeRandomGeofence");
            gGpsGeofenceRemoveCallbackCalled = true;
            return;
        }
        pthread_mutex_lock(&gMutexGeofence);
        printf("Waiting on Callback...\n");
        while (gGpsGeofenceRemoveCallbackCalled == false)
            pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
        pthread_mutex_unlock(&gMutexGeofence);
    }
    else
    {
        int index = randomInt(0, pausedMap.size() - 1);
        TestGeoFencesMap::iterator it = pausedMap.begin();

        for (int i = 0; i < index; ++i)
            it++;

        int32_t geofence_id = it->first;

        pausedMap.erase(it);

        printf("REMOVE-PAUSED GPS GEOFENCE -> %5u \n", geofence_id);
        LOC_LOGW("REMOVE-PAUSED GPS GEOFENCE -> %5u ", geofence_id);

        gGpsGeofenceRemoveCallbackCalled = false;
        if (sGeoFencer) {
            sGeoFencer->removeCommand(geofence_id);
        }
        else {
            printf("sGeoFencer is NULL in removeRandomGeofence");
            LOC_LOGE("sGeoFencer is NULL in removeRandomGeofence");
            gGpsGeofenceRemoveCallbackCalled = true;
            return;
        }
        pthread_mutex_lock(&gMutexGeofence);
        printf("Waiting on Callback...\n");
        while (gGpsGeofenceRemoveCallbackCalled == false)
            pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
        pthread_mutex_unlock(&gMutexGeofence);
    }
}

void pauseRandomGeofence()
{
    TestGeoFencesMap& runningMap(sGpsGeoFencesRunning);
    TestGeoFencesMap& pausedMap(sGpsGeoFencesPaused);

    if (!runningMap.empty())
    {
        int index = randomInt(0, runningMap.size() - 1);
        TestGeoFencesMap::iterator it = runningMap.begin();

        for (int i = 0; i < index; ++i)
            it++;

        int32_t geofence_id = it->first;

        pausedMap[it->first] = it->second;
        runningMap.erase(it);

        printf("PAUSE GPS GEOFENCE -> %5u \n", geofence_id);
        LOC_LOGW("PAUSE GPS GEOFENCE -> %5u ", geofence_id);

        gGpsGeofencePauseCallbackCalled = false;
        if (sGeoFencer) {
            sGeoFencer->pauseCommand(geofence_id);
        }
        else {
            printf("sGeoFencer is NULL in pauseRandomGeofence");
            LOC_LOGE("sGeoFencer is NULL in pauseRandomGeofence");
            gGpsGeofencePauseCallbackCalled = true;
            return;
        }
        pthread_mutex_lock(&gMutexGeofence);
        printf("Waiting on Callback...\n");
        while (gGpsGeofencePauseCallbackCalled == false)
            pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
        pthread_mutex_unlock(&gMutexGeofence);
    }
}

void resumeRandomGeofence()
{
    TestGeoFencesMap& runningMap(sGpsGeoFencesRunning);
    TestGeoFencesMap& pausedMap(sGpsGeoFencesPaused);

    if (!pausedMap.empty())
    {
        int index = randomInt(0, pausedMap.size() - 1);
        TestGeoFencesMap::iterator it = pausedMap.begin();

        for (int i = 0; i < index; ++i)
            it++;

        int32_t geofence_id = it->first;

        int monitor_transitions;
        monitor_transitions = LOC_GPS_GEOFENCE_ENTERED |
            LOC_GPS_GEOFENCE_EXITED |
            LOC_GPS_GEOFENCE_UNCERTAIN;

        runningMap[it->first] = it->second;
        pausedMap.erase(it);

        printf("RESUME GPS GEOFENCE -> %5u \n", geofence_id);
        LOC_LOGW("RESUME GPS GEOFENCE -> %5u ", geofence_id);

        gGpsGeofenceResumeCallbackCalled = false;
        if (sGeoFencer) {
            sGeoFencer->resumeCommand(geofence_id, monitor_transitions);
        }
        else {
            printf("sGeoFencer is NULL in resumeRandomGeofence");
            LOC_LOGE("sGeoFencer is NULL in resumeRandomGeofence");
            gGpsGeofenceResumeCallbackCalled = true;
            return;
        }
        pthread_mutex_lock(&gMutexGeofence);
        printf("Waiting on Callback...\n");
        while (gGpsGeofenceResumeCallbackCalled == false)
            pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
        pthread_mutex_unlock(&gMutexGeofence);
    }
}

void modifyRandomGeofence()
{
    bool modifyRunningGeofence = true;
    if (!sGpsGeoFencesRunning.empty() && !sGpsGeoFencesPaused.empty())
    {
        modifyRunningGeofence = randomInt(0, 1);
    }
    else if (!sGpsGeoFencesRunning.empty())
    {
        modifyRunningGeofence = true;
    }
    else if (!sGpsGeoFencesPaused.empty())
    {
        modifyRunningGeofence = false;
    }
    else
    {
        return;
    }

    TestGeoFencesMap::iterator it;
    int index;
    if (modifyRunningGeofence)
    {
        index = randomInt(0, sGpsGeoFencesRunning.size() - 1);
        it = sGpsGeoFencesRunning.begin();
    }
    else
    {
        index = randomInt(0, sGpsGeoFencesPaused.size() - 1);
        it = sGpsGeoFencesPaused.begin();
    }

    for (int i = 0; i < index; ++i)
        it++;

    int32_t geofence_id = it->first;
    GeofenceExtOptions options;

    memset(&options, 0, sizeof(GeofenceExtOptions));
    options.last_transition = LOC_GPS_GEOFENCE_UNCERTAIN;
    options.monitor_transitions = LOC_GPS_GEOFENCE_ENTERED |
        LOC_GPS_GEOFENCE_EXITED |
        LOC_GPS_GEOFENCE_UNCERTAIN;
    options.notification_responsivenes_ms = randomInt(1, 0x40000);

    printf("MODIFY-RUNNING FLP GEOFENCE -> %5u | %d \n", geofence_id, options.notification_responsivenes_ms);
    LOC_LOGW("MODIFY-RUNNING FLP GEOFENCE -> %5u | %d ", geofence_id, options.notification_responsivenes_ms);

    if (sGeoFencer) {
        sGeoFencer->modifyCommand(geofence_id, &options);
    }
    else {
        printf("sGeoFencer is NULL in modifyRandomGeofence");
        LOC_LOGE("sGeoFencer is NULL in modifyRandomGeofence");
    }
}

void dump()
{
    printf("GPS  | afwId | latitude  | longitude |  radius  | paused \n");
    LOC_LOGW("GPS  | afwId | latitude  | longitude |  radius  | paused ");

    for (TestGeoFencesMap::iterator it = sGpsGeoFencesRunning.begin(); it != sGpsGeoFencesRunning.end(); it++)
    {
        uint32_t afwId = it->first;
        TestGeofenceData data(it->second);
        printf("     | %5u | %8.4f  | %9.4f | %4.4f | %d \n",
            afwId, data.latitude,
            data.longitude, data.radius_m, 0);
        LOC_LOGW("     | %5u | %8.4f  | %9.4f | %4.4f | %d ",
            afwId, data.latitude,
            data.longitude, data.radius_m, 0);
    }

    for (TestGeoFencesMap::iterator it = sGpsGeoFencesPaused.begin(); it != sGpsGeoFencesPaused.end(); it++)
    {
        uint32_t afwId = it->first;
        TestGeofenceData data = it->second;
        printf("     | %5u | %8.4f  | %9.4f | %4.4f | %d \n",
            afwId, data.latitude,
            data.longitude, data.radius_m, 1);
        LOC_LOGW("     | %5u | %8.4f  | %9.4f | %4.4f | %d ",
            afwId, data.latitude,
            data.longitude, data.radius_m, 1);
    }
}

void get_batch_size()
{
    int batchSize = 0;

    printf("\ncalling get_batch_size...");

    if (sFlpLocationClient) {
        batchSize = sFlpLocationClient->flp_get_batch_size();
    }
    else {
        printf("sFlpLocationClient is null\n");
        LOC_LOGE("sFlpLocationClient is null");
    }

    printf("\nget_batch_size returned %i", batchSize);
}

void start_batching()
{
    char buf[16], *p;
    FlpExtBatchOptions options;
    options.flags = 0;
    options.period_ns = 0;
    options.distance_ms = 0;

    printf ("\nLOC_FLP_BATCH_WAKEUP_ON_FIFO_FULL (y/n) :");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (p[0] == 'y')
        options.flags |= LOC_FLP_BATCH_WAKEUP_ON_FIFO_FULL;

    printf ("\nLOC_FLP_BATCH_CALLBACK_ON_LOCATION_FIX (y/n) :");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (p[0] == 'y')
        options.flags |= LOC_FLP_BATCH_CALLBACK_ON_LOCATION_FIX;

    printf ("\nEnter period in seconds:");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    int period = atoi(p);
    printf ("\nseconds entered = %i:", period);
    options.period_ns = (int64_t)period*1000000000;

    printf ("\nEnter Smallest Displacement in meters (default 0):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atof(p) != 0)
        options.distance_ms = atof(p);

    printf("\ncalling start_batching with flags=%i period_ns=%lld smallest_displacement=%f...",
               options.flags, options.period_ns, options.distance_ms);

    int ret = 0;

    if (sFlpLocationClient) {
        ret = sFlpLocationClient->flp_start_session(1, &options);
    }
    else {
        printf("sFlpLocationClient is null\n");
        LOC_LOGE("sFlpLocationClient is null");
    }

    printf("\nstart_batching returned %i", ret);
}

void update_batching()
{
    char buf[16], *p;
    FlpExtBatchOptions options;
    memset(&options, 0, sizeof(options));

    printf ("\nLOC_FLP_BATCH_WAKEUP_ON_FIFO_FULL (y/n) :");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (p[0] == 'y')
        options.flags |= LOC_FLP_BATCH_WAKEUP_ON_FIFO_FULL;

    printf ("\nLOC_FLP_BATCH_CALLBACK_ON_LOCATION_FIX (y/n) :");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (p[0] == 'y')
        options.flags |= LOC_FLP_BATCH_CALLBACK_ON_LOCATION_FIX;

    printf ("\nEnter period in seconds:");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    int period = atoi(p);
    printf ("\nseconds entered = %i:", period);
    options.period_ns = (int64_t)period*1000000000;

    printf("\ncalling update_batching with flags=%i period_ns=%lld ...",
    options.flags, options.period_ns);
    int ret = 0;

    if (sFlpLocationClient) {
        ret = sFlpLocationClient->flp_update_session(1, &options);
    }
    else {
        printf("sFlpLocationClient is null\n");
        LOC_LOGE("sFlpLocationClient is null");
    }
    printf("\nstart_batching returned %i", ret);
}

void stop_batching()
{
    printf("\ncalling stop_batching...");
    int ret = 0;

    if (sFlpLocationClient) {
        ret = sFlpLocationClient->flp_stop_session(1);
    }
    else {
        printf("sFlpLocationClient is null\n");
        LOC_LOGE("sFlpLocationClient is null");
    }
    printf("\nstop_batching returned %i", ret);
}

void get_batched_location()
{
    char buf[16], *p;
    printf ("\nEnter number of locations:");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    int n = atoi(p);
    printf("\ncalling get_batched_location..n=%i.\n", n);

    if (sFlpLocationClient) {
        sFlpLocationClient->flp_get_batched_location(n);
    }
    else {
        printf("sFlpLocationClient is null\n");
        LOC_LOGE("sFlpLocationClient is null");
    }
    printf("\nget_batch_size returned");
}

void batching_clean_up()
{
    printf("\ncalling cleanup...");

    if (sFlpLocationClient) {
        sFlpLocationClient->flp_cleanup();
    }
    else {
        printf("sFlpLocationClient is null\n");
        LOC_LOGE("sFlpLocationClient is null");
    }

    printf("\ncleanup returned");
}

#include <ctime>

void inject_location()
{
    FlpExtLocation location;
    memset(&location, 0, sizeof(location));
    location.size = sizeof(FlpExtLocation);
    location.flags = 31;
    location.latitude = 32.896535;
    location.longitude = -117.201025;
    location.accuracy = 50;
    location.speed = 0;
    location.bearing = 0;
    location.timestamp = std::time(0);

    printf("\ncalling inject_location time=%lld...", location.timestamp);

    if (sFlpLocationClient) {
        sFlpLocationClient->flp_inject_location(&location);
    }
    else {
        printf("sFlpLocationClient is null\n");
        LOC_LOGE("sFlpLocationClient is null");
    }

    printf("\ninject_location returned");
}

void geofence_stress_test_2()
{
    uint32_t testCount = 50;

    char buf[16], *p;
    printf("\nEnter number of times to add/remove 50 geofences (default 50):");
    fflush(stdout);
    p = fgets(buf, 16, stdin);
    if (p == NULL) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atoi(p) != 0)
        testCount = atoi(p);

    while (testCount-- > 0)
    {
        for (uint32_t i = 0; i < 50; ++i)
            addRandomGeofence();

        for (uint32_t i = 0; i < 50; ++i)
            removeRandomGeofence();
    }

    printf("CLEARING ALL GEOFENCES\n");
    LOC_LOGW("CLEARING ALL GEOFENCES");
    sGpsGeoFencesRunning.clear();
    sGpsGeoFencesPaused.clear();
}

void geofence_stress_test()
{
    uint32_t testCount = 50;
    uint32_t geofencesToStartWith = 50;

    char buf[16], *p;
    printf("\nEnter number of initial geofences to add (default 50):");
    fflush(stdout);
    p = fgets(buf, 16, stdin);
    if (p == NULL) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atoi(p) != 0)
        geofencesToStartWith = atoi(p);
    printf("\nEnter number of random geofences commands to call (default 50):");
    fflush(stdout);
    p = fgets(buf, 16, stdin);
    if (p == NULL) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atoi(p) != 0)
        testCount = atoi(p);

    srand(time(NULL));

    if (geofencesToStartWith > 10000) {
        printf("\nInvalid geofencesToStartWith (max 10000) %u",
                geofencesToStartWith);
        return;
    }
    for (uint32_t i = 0; i < geofencesToStartWith; ++i)
        addRandomGeofence();

    while (testCount-- > 0)
    {
        GeoFenceCommands commandId = (GeoFenceCommands)randomInt(0, 4);
        switch (commandId)
        {
        case ADD_COMMAND:
            printf("ADD_COMMAND\n");
            LOC_LOGW("ADD_COMMAND");
            addRandomGeofence();
            break;

        case PAUSE_COMMAND:
            printf("PAUSE_COMMAND\n");
            LOC_LOGW("PAUSE_COMMAND");
            pauseRandomGeofence();
            break;

        case RESUME_COMMAND:
            printf("RESUME_COMMAND\n");
            LOC_LOGW("RESUME_COMMAND");
            resumeRandomGeofence();
            break;

        case MODIFY_COMMAND:
            printf("MODIFY_COMMAND\n");
            LOC_LOGW("MODIFY_COMMAND");
            modifyRandomGeofence();
            break;

        case REMOVE_COMMAND:
            printf("REMOVE_COMMAND\n");
            LOC_LOGW("REMOVE_COMMAND");
            removeRandomGeofence();
            break;

        }

        dump();
    }

    printf("CLEARING ALL GEOFENCES\n");
    LOC_LOGW("CLEARING ALL GEOFENCES");
    sGpsGeoFencesRunning.clear();
    sGpsGeoFencesPaused.clear();

    printf("DONE\n");
    LOC_LOGW("DONE");
}

void gps_add_geofence()
{
    int32_t geofence_id;
    double latitude;
    double longitude;
    double radius_m;
    int monitor_transitions = LOC_GPS_GEOFENCE_ENTERED |
        LOC_GPS_GEOFENCE_EXITED |
        LOC_GPS_GEOFENCE_UNCERTAIN;

    geofence_id = 1;
    latitude = 32.896535;
    longitude = -117.201025;
    radius_m = 50;
    int responsiveness_ms = 4000;

    char buf[16], *p;
    printf ("\nEnter id (default 1):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atoi(p) != 0)
        geofence_id = atoi(p);

    printf ("\nEnter latitude (default 32.896535):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atof(p) != 0)
        latitude = atof(p);

    printf ("\nEnter longitude (default -117.201025):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atof(p) != 0)
        longitude = atof(p);

    printf ("\nEnter radius (default 50):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atof(p) != 0)
        radius_m = atof(p);

    printf ("\nEnter responsiveness in seconds: (default 4):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    int responsiveness_sec = atoi(p);
    if (responsiveness_sec < 0 ||
            responsiveness_sec > 1000000) {
        printf("Error: responsiveness value out of bound");
        return;
    }
    if (responsiveness_sec != 0)
        responsiveness_ms = responsiveness_sec*1000;

    printf("\nGPS GEOFENCE ADD -> id=%2u | lat=%8.2f | lon=%8.2f | rad=%2.2f | resp=%d(ms)\n",
            geofence_id, latitude,
            longitude, radius_m,
            responsiveness_ms);
    LOC_LOGW("GPS GEOFENCE ADD -> id=%2u | lat=%8.2f | lon=%8.2f | rad=%2.2f | resp=%d(ms)",
            geofence_id, latitude,
            longitude, radius_m,
            responsiveness_ms);

    gGpsGeofenceAddCallbackCalled = false;

    if (sGeoFencer) {
        sGeoFencer->addCommand(geofence_id,
            latitude,
            longitude,
            radius_m,
            LOC_GPS_GEOFENCE_UNCERTAIN, //last_transition
            monitor_transitions, // transition_types
            responsiveness_ms,
            0, //unknown_timer_ms
            0, // confidence
            0, // dwell_time_s
            0); // dwell_time_mask
    }
    else {
        printf("sGeoFencer is NULL in gps_add_geofence");
        LOC_LOGE("sGeoFencer is NULL in gps_add_geofence");
        gGpsGeofenceAddCallbackCalled = true;
        return;
    }

    pthread_mutex_lock(&gMutexGeofence);
    printf("Waiting on Callback...\n");
    while (gGpsGeofenceAddCallbackCalled == false)
        pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

void gps_pause_geofence()
{
    int32_t geofence_id = 1;

    char buf[16], *p;
    printf ("\nEnter id (default 1):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atoi(p) != 0)
        geofence_id = atoi(p);

    printf("\nGPS GEOFENCE PAUSE -> id=%2u \n",
            geofence_id);
    LOC_LOGW("GPS GEOFENCE PAUSE -> id=%2u ",
            geofence_id);

    gGpsGeofencePauseCallbackCalled = false;
    if (sGeoFencer) {
        sGeoFencer->pauseCommand(geofence_id);
    }
    else {
        printf("sGeoFencer is NULL in gps_pause_geofence");
        LOC_LOGE("sGeoFencer is NULL in gps_pause_geofence");
        gGpsGeofencePauseCallbackCalled = true;
        return;
    }
    pthread_mutex_lock(&gMutexGeofence);
    printf("Waiting on Callback...\n");
    while (gGpsGeofencePauseCallbackCalled == false)
        pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

void gps_resume_geofence()
{
    int32_t geofence_id = 1;
    int monitor_transitions = LOC_GPS_GEOFENCE_ENTERED |
                              LOC_GPS_GEOFENCE_EXITED  |
                              LOC_GPS_GEOFENCE_UNCERTAIN;
    char buf[16], *p;
    printf ("\nEnter id (default 1):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atoi(p) != 0)
        geofence_id = atoi(p);

    printf("\nGPS GEOFENCE RESUME -> id=%2u monitor_transitions=%d\n",
            geofence_id, monitor_transitions);
    LOC_LOGW("GPS GEOFENCE RESUME -> id=%2u monitor_transitions=%d",
            geofence_id, monitor_transitions);

    gGpsGeofenceResumeCallbackCalled = false;
    if (sGeoFencer) {
        sGeoFencer->resumeCommand(geofence_id, monitor_transitions);
    }
    else {
        printf("sGeoFencer is NULL in gps_resume_geofence");
        LOC_LOGE("sGeoFencer is NULL in gps_resume_geofence");
        gGpsGeofenceResumeCallbackCalled = true;
        return;
    }

    pthread_mutex_lock(&gMutexGeofence);
    printf("Waiting on Callback...\n");
    while (gGpsGeofenceResumeCallbackCalled == false)
        pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

void gps_remove_geofence()
{
    int32_t geofence_id = 1;

    char buf[16], *p;
    printf ("\nEnter id (default 1):");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (atoi(p) != 0)
        geofence_id = atoi(p);

    printf("\nGPS GEOFENCE REMOVE -> id=%2u \n",
            geofence_id);
    LOC_LOGW("GPS GEOFENCE REMOVE -> id=%2u ",
            geofence_id);

    gGpsGeofenceRemoveCallbackCalled = false;
    if (sGeoFencer) {
        sGeoFencer->removeCommand(geofence_id);
    }
    else {
        printf("sGeoFencer is NULL in gps_remove_geofence");
        LOC_LOGE("sGeoFencer is NULL in gps_remove_geofence");
        gGpsGeofenceRemoveCallbackCalled = true;
        return;
    }
    pthread_mutex_lock(&gMutexGeofence);
    printf("Waiting on Callback...\n");
    while (gGpsGeofenceRemoveCallbackCalled == false)
        pthread_cond_wait(&gCondGeofence, &gMutexGeofence);
    pthread_mutex_unlock(&gMutexGeofence);
}

void gps_start_tracking()
{
    printf("\nSTART TRACKING\n");
    LOC_LOGW("START TRACKING");

    if (sGpsInterface) {
        sGpsInterface->set_position_mode(LOC_GPS_POSITION_MODE_STANDALONE,
                                         LOC_GPS_POSITION_RECURRENCE_PERIODIC,
                                         1000, 89, 10000);
        sGpsInterface->start();
    }

}

void gps_stop_tracking()
{
    printf("\nSTOP TRACKING\n");
    LOC_LOGW("STOP TRACKING");

    if (sGpsInterface) {
        sGpsInterface->stop();
    }
}

void flp_batching_menu()
{
    char buf[16], *p;
    bool exit_loop = false;

    while(!exit_loop)
    {
        printf("\n\n");
        printf ("1: get batch size\n"
            "2: start batching\n"
            "3: update batching\n"
            "4: stop batching\n"
            "5: get batched location\n"
            "6: batching clean up\n"
            "7: inject location\n"
            "b: back\n"
            "q: quit\n"
            "\nEnter Command:");
        fflush (stdout);
        p = fgets (buf, 16, stdin);
        if( p == NULL ) {
            printf("Error: fgets returned NULL !!");
            return;
        }

        switch(p[0])
        {
        case '1':
            get_batch_size();
            break;
        case '2':
            start_batching();
            break;
        case '3':
            update_batching();
            break;
        case '4':
            stop_batching();
            break;
        case '5':
            get_batched_location();
            break;
        case '6':
            batching_clean_up();
            break;
        case '7':
            inject_location();
            break;
        case 'b':
            exit_loop = true;
            break;
        case 'q':
            exit(0);
            break;
        default:
            printf("\ninvalid command\n");
        }
    }

}

void gps_positioning_menu()
{
    char buf[16], *p;
    bool exit_loop = false;

    while(!exit_loop)
    {
        printf("\n\n");
        printf ("1: start tracking\n"
            "2: stop tracking\n"
            "b: back\n"
            "q: quit\n"
            "\nEnter Command:");
        fflush (stdout);
        p = fgets (buf, 16, stdin);
        if( p == NULL ) {
            printf("Error: fgets returned NULL !!");
            return;
        }

        switch(p[0])
        {
        case '1':
            gps_start_tracking();
            break;
        case '2':
            gps_stop_tracking();
            break;
        case 'b':
            exit_loop = true;
            break;
        case 'q':
            exit(0);
            break;
        default:
            printf("\ninvalid command\n");
        }
    }

}

void gps_geofence_menu()
{
    char buf[16], *p;
    bool exit_loop = false;

    while(!exit_loop)
    {
        printf("\n\n");
        printf ("1: add_geofence\n"
            "2: pause_geofence\n"
            "3: resume geofence\n"
            "4: remove geofence\n"
            "b: back\n"
            "q: quit\n"
            "\nEnter Command:");
        fflush (stdout);
        p = fgets (buf, 16, stdin);
        if( p == NULL ) {
            printf("Error: fgets returned NULL !!");
            return;
        }

        switch(p[0])
        {
        case '1':
            gps_add_geofence();
            break;
        case '2':
            gps_pause_geofence();
            break;
        case '3':
            gps_resume_geofence();
            break;
        case '4':
            gps_remove_geofence();
            break;
        case 'b':
            exit_loop = true;
            break;
        case 'q':
            exit(0);
        default:
            printf("\ninvalid command\n");
        }
    }

}

void gps_verify_test_menu()
{
    char buf[16], *p;
    gStopOnMinSvs = 0;
    gStopOnMinSnr = 0.0;
    gGpsVerifyTestInProgress = true;

    printf ("\nComplete test with SVs SNR instead of position? (y/n) (default n) :");
    fflush (stdout);
    p = fgets (buf, 16, stdin);
    if( p == NULL ) {
        printf("Error: fgets returned NULL !!");
        return;
    }
    if (p[0] == 'y')
    {
        printf ("\nEnter minimum SVs needed to complete test? (default 1):");
        fflush (stdout);
        p = fgets (buf, 16, stdin);
        if( p == NULL ) {
            printf("Error: fgets returned NULL !!");
            return;
        }
        gStopOnMinSvs = 1;
        if (atoi(p) != 0)
            gStopOnMinSvs = atoi(p);
        printf ("\nEnter minimum SNR for SVs to complete test (default 28):");
        fflush (stdout);
        p = fgets (buf, 16, stdin);
        if( p == NULL ) {
            printf("Error: fgets returned NULL !!");
            return;
        }
        gStopOnMinSnr = 28;
        if (atof(p) != 0)
            gStopOnMinSnr = atof(p);
    }

}

void gps_verify_test()
{
    printf("GPS VERIFY TEST START...\n");
    LOC_LOGW("GPS VERIFY TEST START");
    printf("\ncalling gps_start_tracking with StopOnMinSvs=%i StopOnMinSnr=%f...\n",
              gStopOnMinSvs, gStopOnMinSnr);
    gps_start_tracking();

    pthread_mutex_lock(&gMutexLocation);
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 300;
    int ret = pthread_cond_timedwait(&gCondLocation, &gMutexLocation, &ts);
    pthread_mutex_unlock(&gMutexLocation);
    if (ret == ETIMEDOUT) {
        printf("GPS VERIFY TEST FAILED...Timeout\n");
        LOC_LOGW("GPS VERIFY TEST FAILED...Timeout");
    }
    gps_stop_tracking();
}

void flp_batching_verify(int fStopOnSec)
{
    int ret;
    FlpExtBatchOptions options;
    memset(&options, 0, sizeof(FlpExtBatchOptions));
    options.flags |= LOC_FLP_BATCH_WAKEUP_ON_FIFO_FULL;
    options.period_ns = (int64_t)5*1000000000; // set period=5s
    options.distance_ms = 0;

    sFlpLocationClient->flp_start_session(1, &options);
    pthread_mutex_lock(&gMutexLocation);
    if(fStopOnSec) { // -F option with nonzero argument, flp session waits fStopOnSec seconds here
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += fStopOnSec;
        ret = pthread_cond_timedwait(&gCondLocation, &gMutexLocation, &ts);
    }
    else // -F option with argument 0, flp session waits here and runs permanently
        ret = pthread_cond_wait(&gCondLocation, &gMutexLocation);
    pthread_mutex_unlock(&gMutexLocation);
    if(fStopOnSec && ret == ETIMEDOUT) { //after fStopOnSec seconds, flp session runs out of time
        printf("FLP BATCHING VERIFY STOPPED...Timeout\n");
        LOC_LOGW("FLP BATCHING VERIFY STOPPED...Timeout\n");
    }
}

int main (int argc, char *argv[])
{
     if (argc <= 1) {
         printf("Optional command line arguments to start gps verify test without menu:\n");
         printf("    -T:  1 to start gps verify test (ex: location_hal_test -T 1)\n");
         printf("    -A:  Minimum number of SVs seen in combination with -B option\n");
         printf("         to determine when to stop the test without actually getting\n");
         printf("         a position report to save test time\n");
         printf("    -B:  Minimum SNR for each SV seen in -A option to determine\n");
         printf("         when to stop the test  without actually getting a position\n");
         printf("         report to save test time\n");
         printf("    -F:  Enable flp batching verify, with argument num to set the running\n");
         printf("         time in seconds, and 0 means the flp session runs permanently\n");
         printf("         (e.g., location_hal_test -F num)\n\n");
    }
    int result = 0;
    int opt;
    int flpStopOnSec = 0;
    extern char *optarg;

    while ((opt = getopt (argc, argv, "t:T:a:A:b:B:f:F:")) != -1) {
        switch (opt) {
        case 't':
        case 'T':
            gGpsVerifyTestInProgress = (atoi(optarg) == 1);
            printf("GPS Verify Test:%d\n", gGpsVerifyTestInProgress);
            break;
        case 'a':
        case 'A':
            gStopOnMinSvs = atoi(optarg);
            printf("Stop on Minimum Svs: %d\n",gStopOnMinSvs);
            break;
        case 'b':
        case 'B':
            gStopOnMinSnr = atof(optarg);
            printf("Stop on Minimum SNR: %f\n",gStopOnMinSnr);
            break;
        case 'f':
        case 'F':
            flpStopOnSec = atoi(optarg);
            gFlpBatchingVerify = true;
            printf("FLP batching verify: %d\n", gFlpBatchingVerify);
            break;
        }
    }

    int ret_val = init();
    if (ret_val) {
        printf("init failed!\n");
        LOC_LOGW("init failed!");
        return 1;
    }

    if (gGpsVerifyTestInProgress)
    {
        gps_verify_test();
        return 0;
    }

    if(gFlpBatchingVerify)
    {
        if(sFlpLocationClient)
        {
            flp_batching_verify(flpStopOnSec);
            sFlpLocationClient->flp_stop_session(1);
            sFlpLocationClient->flp_cleanup();
        }
        else
        {
            printf("sFlpLocationClient is null, and cannot start flp batching\n");
            LOC_LOGE("sFlpLocationClient is null, and cannot start flp batching");
        }
        return 0;
    }

    char buf[16], *p;
    bool exit_loop = false;

    while(!exit_loop)
    {
        usleep(100000);
        printf("\n\n"
                "1: GPS Verify Test\n"
                "2: GPS Positioning\n"
                "3: Geofence\n"
                "4: FLP Batching\n"
                "5: Geofence Stress Test 1\n"
                "6: Geofence Stress Test 2\n"
                "q: quit\n"
                "\nEnter Command:");
        fflush (stdout);
        p = fgets (buf, 16, stdin);
        if( p == NULL ) {
            printf("Error: fgets returned NULL !!");
            return -1;
        }

        switch(p[0])
        {
        case '1':
            gps_verify_test_menu();
            gps_verify_test();
            break;
        case '2':
            gps_positioning_menu();
            break;
        case '3':
            gps_geofence_menu();
            break;
        case '4':
            flp_batching_menu();
            break;
        case '5':
            geofence_stress_test();
            break;
        case '6':
            geofence_stress_test_2();
            break;
        case 'q':
            exit_loop = true;
            break;
        default:
            printf("\ninvalid command\n");
        }
    }

    return 0;
}

