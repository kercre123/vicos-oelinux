/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2011-2015 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <gps_extended.h>
#include "platform_lib_log_util.h"
#include <stdarg.h>
#include <pthread.h>
#include <sys/time.h>
#ifdef __ANDROID__
#include <android/log.h>
#include <private/android_filesystem_config.h>
#include "android_runtime/AndroidRuntime.h"
#include <linux/android_alarm.h>
#include "qmi_client_instance_defs.h"
#include <cutils/klog.h>
#include <linux/android_alarm.h>
#endif
#include "garden_app_session_tracker.h"

#ifdef USE_GLIB
#include "IIzatManager.h"
#include "OSFramework.h"
#include "PowerEvtHandler.h"
#endif /* USE_GLIB */
#include "loc_vzw.h"
#include "loc_cfg.h"
#include "test_android_gps.h"
#include "ulp_service.h"
#include "loc_extended.h"
#include "loc_target.h"
#include <time.h>
#include "qmi_client_instance_defs.h"
#include "EventObserver.h"
#include "FlpLocationClient.h"

#define LOC_FLP_BATCH_CALLBACK_ON_LOCATION_FIX   0x0000002

#define TRUE     1
#define FALSE    0
#define LOG_TAG  "afw-simu"

#ifdef USE_GLIB
#include <glib.h>
#define strlcpy g_strlcpy
#endif /* USE_GLIB */

typedef enum
{
    ACTION_NONE,
    ACTION_QUIT,
    ACTION_OPEN_ATL,
    ACTION_OPEN_ATL_HALCLIENT,
    ACTION_CLOSE_ATL,
    ACTION_CLOSE_ATL_HALCLIENT,
    ACTION_FAIL_ATL,
    ACTION_FAIL_ATL_HALCLIENT,
    ACTION_NI_NOTIFY,
    ACTION_XTRA_DATA,
    ACTION_XTRA_TIME,
    ACTION_NLP_RESPONSE,
    ACTION_PHONE_CONTEXT_UPDATE,
    ACTION_EXT_CLIENT_INIT
} test_thread_action_e_type;

pthread_mutex_t test_thread_mutex;
pthread_cond_t test_thread_cond;

pthread_mutex_t location_conn_mutex;
pthread_cond_t location_conn_cond;

pthread_mutex_t ext_client_init_mutex;
pthread_cond_t ext_client_init_cond;

pthread_mutex_t close_atl_handles_mutex;
pthread_cond_t close_atl_handles_cond;

pthread_mutex_t networkpos_req_mutex;
pthread_cond_t networkpos_req_cond;

pthread_mutex_t phone_context_mutex;
pthread_cond_t phone_context_cond;

pthread_mutex_t ulp_location_mutex;
pthread_cond_t ulp_location_cond;

pthread_mutex_t session_status_mutex;
pthread_cond_t session_status_cond;

pthread_mutex_t wait_count_mutex;
pthread_cond_t wait_count_cond;

pthread_mutex_t wait_atlcb_mutex;
pthread_cond_t wait_atlcb_cond;

test_thread_action_e_type test_thread_action;
AGpsExtType test_thread_agps_ext_type;

const LocGpsInterface *pGpsInterface = NULL;
const LocAGpsInterface* pAGpsInterface = NULL;
const LocGpsGeofencingInterface* pGpsGeoFencingInterface = NULL;
const LocGnssConfigurationInterface* pGnssConfigurationInterface = NULL;
LocAGpsRilInterface *pAgpsRilInterface = NULL;
LocGpsNiNotification sNotification;
FlpLocationClient* sFlpLocationClient = NULL;
int LEGACY;
#ifdef TEST_ULP
const UlpEngineInterface* pUlpEngineInterface = NULL;
const UlpPhoneContextInterface *pUlpPhoneContextInterface = NULL;
const UlpNetworkInterface* pUlpNetworkInterface = NULL;
#endif

struct timeval TimeAtStartNav = {0, 0};
struct timeval TimeAtFirstFix = {0, 0};

int g_numAtlHandleOpen = 0;
int g_checkForEngineOff = 0;
int g_checkForXtraDataCallBack = 1;
int g_exitStatus = 0; // Test result unknown

static time_t  startTime, firstFixTime;

//Global Phone context stettings
// global phone context setting
bool    is_gps_enabled = TRUE;
/** is network positioning enabled */
bool    is_network_position_available = TRUE;
/** is wifi turned on */
bool    is_wifi_setting_enabled = TRUE;
/** is battery being currently charged */
bool    is_battery_charging = TRUE;
/** is agps enabled */
bool    is_agps_enabled = TRUE;
/** run tracking session if true*/
static bool    g_run_active_client = FALSE;


/** Agps Command Line options **/
typedef struct agps_command_line_options {
    LocAGpsType agpsType; // Agps type
    const char * apn; // apn
    AGpsBearerType agpsBearerType; // Agps bearer type.
/* values for struct members from here down are initialized by reading gps.conf
   file using -c options to garden */
    unsigned long suplVer;
    char suplHost[256];
    int suplPort;
    char c2kHost[256];
    int c2kPort;
} AgpsCommandLineOptionsType;

// Default Agps command line values
static AgpsCommandLineOptionsType sAgpsCommandLineOptions = {
    LOC_AGPS_TYPE_SUPL,
    "myapn.myapn.com",
    AGPS_APN_BEARER_IPV4,
    0x10000, // SUPL version
    "supl.host.com",
    1234,
    "c2k.pde.com",
    1234
};

/** Structure that holds the command line options given to main **/
typedef struct command_line_options {
    LocGpsPositionRecurrence r; // recurrence type
    int l; // Number of sessions to loop through.
    int t; // time to stop fix in seconds
    int s; // Stacks to test.
    int b; // Legacy TRUE OR FALSE.
    int ulpTestCaseNumber; // Run specified ULP test case number
    int deleteAidingDataMask; // Specifies Hexadecimal mask for deleting aiding data.
    int positionMode; // Specifies Position mode.
    int interval; // Time in milliseconds between fixes.
    int accuracy; // Accuracy in meters
    int responseTime; // Requested time to first fix in milliseconds
    LocGpsLocation location; // Only latitude, longiture and accuracy are used in this structure.
    int networkInitiatedResponse[256]; // To store the response pattern
    int niCount; // Number of back to back Ni tests
    int niResPatCount; // Number of elements in the response pattern
    AgpsCommandLineOptionsType agpsData; // Agps Data
    int isSuccess; // Success, Failure, ...
    bool isNativeAgpsEnabled;
    int rilInterface; // Ril Interface
    char gpsConfPath[256]; // Path to config file
    int disableAutomaticTimeInjection; // Flag to indicate whether to disable or enable automatic time injection
    int niSuplFlag; // Flag to indicate that tests being conducted is ni supl
    int printNmea; // Print nmea string
    int satelliteDetails; // Print detailed info on satellites in view.
    int fixThresholdInSecs; // User specified time to first fix threshold in seconds
    int zppTestCaseNumber; // Run specified ULP test case number
    int enableXtra; // Flag to enable/disable Xtra
    int stopOnMinSvs; // part of conditions that cause test to stop. Minimum number of SVs
    float stopOnMinSnr; // part of conditions that casue test to stop. Minimum number of SNR
    int tracking;    // start tracking session
    bool trackUsingFlp; // start tracking session using HW_FLP
    int keepAlive;   // Keep garden-app alive till Ctrl-C is hit
} CommandLineOptionsType;


/** Default values for position/location */
static LocGpsLocation sPositionDefaultValues = {
    sizeof(LocGpsLocation),
    0,
    32.90285,
    -117.202185,
    0,
    0,
    0,
    10000,
    (LocGpsUtcTime)0,
};


// Default values
static CommandLineOptionsType sOptions = {
    LOC_GPS_POSITION_RECURRENCE_SINGLE,
    1,
    60, // Time to stop fix.
    1, // Android frame work (AFW)AGpsBearerType
    TRUE, // Yes to Legacy
    1, // ULP test case number
    0, // By default don't delete aiding data.
    LOC_GPS_POSITION_MODE_STANDALONE, // Standalone mode.
    1000, // 1000 millis between fixes
    0, // Accuracy
    0, // 1 millisecond?
    sPositionDefaultValues,
    {0}, // Invalid for NI response
    0, // Defaults to 0 back to back NI test
    0, // Number of elements in NI response pattern
    sAgpsCommandLineOptions, // default Agps values
    1, // Agps Success
    FALSE, // isNativeAgpsEnabled
    0, // Test Ril Interface
    {0}, // Default path to config file, will be set in main
    0, // By default do not disable automatic time injection
    0, // NI SUPL flag defaults to zero
    0, // Print NMEA info. By default does not get printed
    0, // SV info. Does not print the detaul by default
    20, // Default value of 20 seconds for time to first fix
    0, // ZPP test case number
    0, // Xtra Enabled By default
    0, // Minimum number of SVs option off by default
    0, // Minimum number of SNR option off by default
    0, //Tracking Session status off by default
    FALSE, // Start GPS tracking session using HW_FLP
    false // Keep garden-app alive till Ctrl-C is pressed
};


static loc_param_s_type cfg_parameter_table[] =
{
  {"SUPL_VER",     &sOptions.agpsData.suplVer,                         NULL, 'n'},
  {"SUPL_HOST",    &sOptions.agpsData.suplHost,                        NULL, 's'},
  {"SUPL_PORT",    &sOptions.agpsData.suplPort,                        NULL, 'n'},
  {"C2K_HOST",     &sOptions.agpsData.c2kHost,                         NULL, 's'},
  {"C2K_PORT",     &sOptions.agpsData.c2kPort,                         NULL, 'n'},
};

#ifdef TEST_ULP
//prototypes
void test_ulp(int ulptestCase);
#endif /* TEST_ULP */
void test_zpp(int zpptestCase);
void run_tracking_session(void);
EventObserver     *gEventObserver = NULL;
garden_app_session_tracker  *gGardenSessionControl = NULL;
void rxSystemEvent(unsigned int systemEvent)
{
    LOC_LOGV("%s: SYSTEM EVENT = %d\n", __func__, systemEvent);
    if (NULL != gGardenSessionControl)
    {
        gGardenSessionControl->process_system_event(systemEvent);
    }
}

static uint64_t getUpTimeSec()
{
    struct timespec ts;

    ts.tv_sec = ts.tv_nsec = 0;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return ts.tv_sec + (ts.tv_nsec / 1000000000LL);
}

#define KPI_FILE_PATH               "sys/kernel/debug/bootkpi/kpi_values"

#ifdef __ANDROID__
void printMarker(char *markerString)
{
    int fd = 0;
    if (NULL != markerString && (fd = ::open(KPI_FILE_PATH, O_RDWR)) > 0) {
        write(fd, markerString, strlen(markerString));
        close(fd);
    }
}
#endif

void mutex_init()
{
    pthread_mutex_init (&test_thread_mutex, NULL);
    pthread_cond_init (&test_thread_cond, NULL);

    pthread_mutex_init (&ext_client_init_mutex, NULL);
    pthread_cond_init (&ext_client_init_cond, NULL);

    pthread_mutex_init (&close_atl_handles_mutex, NULL);
    pthread_cond_init (&close_atl_handles_cond, NULL);

    pthread_mutex_init (&location_conn_mutex, NULL);
    pthread_cond_init (&location_conn_cond, NULL);

    pthread_mutex_init (&networkpos_req_mutex, NULL);
    pthread_cond_init (&networkpos_req_cond, NULL);

    pthread_mutex_init (&phone_context_mutex, NULL);
    pthread_cond_init (&phone_context_cond, NULL);

    pthread_mutex_init (&session_status_mutex, NULL);
    pthread_cond_init (&session_status_cond, NULL);

    pthread_mutex_init (&wait_count_mutex,NULL);
    pthread_cond_init (&wait_count_cond,NULL);

    pthread_mutex_init (&wait_atlcb_mutex,NULL);
    pthread_cond_init (&wait_atlcb_cond,NULL);

    pthread_mutex_init (&ulp_location_mutex, NULL);
    pthread_cond_init (&ulp_location_cond, NULL);

}

void mutex_destroy()
{
    pthread_mutex_destroy (&test_thread_mutex);
    pthread_cond_destroy (&test_thread_cond);

    pthread_mutex_destroy (&ext_client_init_mutex);
    pthread_cond_destroy (&ext_client_init_cond);

    pthread_mutex_destroy (&close_atl_handles_mutex);
    pthread_cond_destroy (&close_atl_handles_cond);

    pthread_mutex_destroy (&location_conn_mutex);
    pthread_cond_destroy (&location_conn_cond);

    pthread_mutex_destroy (&networkpos_req_mutex);
    pthread_cond_destroy (&networkpos_req_cond);

    pthread_mutex_destroy (&phone_context_mutex );
    pthread_cond_destroy (&phone_context_cond);

    pthread_mutex_destroy (&session_status_mutex);
    pthread_cond_destroy (&session_status_cond);

    pthread_mutex_destroy (&wait_count_mutex);
    pthread_cond_destroy (&wait_count_cond);

    pthread_mutex_destroy (&wait_atlcb_mutex);
    pthread_cond_destroy (&wait_atlcb_cond);

    pthread_mutex_destroy (&ulp_location_mutex );
    pthread_cond_destroy (&ulp_location_cond);

}

void garden_print(const char *fmt, ...)
{
    va_list ap;
    char buf[1024];
    va_start(ap, fmt);
    vsnprintf(buf, 1024, fmt, ap);
    va_end(ap);
    fprintf(stderr,"GARDEN: %s\n",buf);
    LOC_LOGV("%s", buf);
}

int timeval_difference (struct timeval *result, struct timeval *x, struct timeval *y)
{
       /* Perform the carry for the later subtraction by updating y. */
       if (x->tv_usec < y->tv_usec) {
         int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
         y->tv_usec -= 1000000 * nsec;
         y->tv_sec += nsec;
       }
       if (x->tv_usec - y->tv_usec > 1000000) {
         int nsec = (x->tv_usec - y->tv_usec) / 1000000;
         y->tv_usec += 1000000 * nsec;
         y->tv_sec -= nsec;
       }

       /* Compute the time remaining to wait.
          tv_usec is certainly positive. */
       result->tv_sec = x->tv_sec - y->tv_sec;
       result->tv_usec = x->tv_usec - y->tv_usec;

       /* Return 1 if result is negative. */
       return x->tv_sec < y->tv_sec;
}

/*  to get over the fact tha Pthread needs a function returning void * */
/*  but Android gps.h declares a fn which returns just a void. */

typedef void (*ThreadStart) (void *);
struct tcreatorData {
    ThreadStart pfnThreadStart;
    void* arg;
};

void *my_thread_fn (void *tcd)
{
    tcreatorData* local_tcd = (tcreatorData*)tcd;
    if (NULL != local_tcd) {
        local_tcd->pfnThreadStart (local_tcd->arg);
        free(local_tcd);
    }

    return NULL;
}

void action_open_atl(test_thread_action_e_type atl_open_type) {

    garden_print ("got LOC_GPS_REQUEST_AGPS_DATA_CONN");
    pthread_mutex_lock (&test_thread_mutex);
    test_thread_action = atl_open_type;
    pthread_cond_signal (&test_thread_cond);
    pthread_mutex_unlock (&test_thread_mutex);
}

void action_close_atl(test_thread_action_e_type atl_close_type) {

    garden_print ("got LOC_GPS_RELEASE_AGPS_DATA_CONN");
    pthread_mutex_lock (&test_thread_mutex);
    test_thread_action = atl_close_type;
    pthread_cond_signal (&test_thread_cond);
    pthread_mutex_unlock (&test_thread_mutex);

}

void action_fail_atl(test_thread_action_e_type atl_fail_type) {

    pthread_mutex_lock (&test_thread_mutex);
    test_thread_action = atl_fail_type;
    pthread_cond_signal (&test_thread_cond);
    pthread_mutex_unlock (&test_thread_mutex);
}

void notify_main_thread_of_atlcb() {
    // Notify main thread everytime callback is invoked
    pthread_mutex_lock (&wait_atlcb_mutex);
    pthread_cond_signal (&wait_atlcb_cond);
    pthread_mutex_unlock (&wait_atlcb_mutex);
}

void decrement_atl_handle_count() {
    // decrement ATL handle count
    if (g_numAtlHandleOpen > 0) {
        g_numAtlHandleOpen--;
    }
    garden_print ("Num ATL handles remaining: %d",g_numAtlHandleOpen);

    // once count is zero, notify cond variable
    if (g_numAtlHandleOpen == 0) {
        pthread_mutex_lock (&close_atl_handles_mutex);
        pthread_cond_signal (&close_atl_handles_cond);
        pthread_mutex_unlock (&close_atl_handles_mutex);
    }
}

/*
 * Helper thread for emulating the Android Framework.
 */
void *test_thread (void *args)
{
    garden_print ("Test Thread Enter");
    int rc = 0;
    pthread_mutex_lock (&test_thread_mutex);
    do {
        pthread_cond_wait (&test_thread_cond, &test_thread_mutex);
        /*
         * got a condition
         */
        garden_print ("test thread unblocked, action = %d",
                     test_thread_action);
        if (test_thread_action == ACTION_QUIT) {
            garden_print ("ACTION_QUIT");
            break;
        }
        switch (test_thread_action) {

        case ACTION_OPEN_ATL:
            garden_print ("ACTION_OPEN_ATL AgpsType: %d", test_thread_agps_ext_type);
            /*
             * sleep(3);
             */
            loc_extended_agps_open(test_thread_agps_ext_type, sOptions.agpsData.apn,
                                   sOptions.agpsData.agpsBearerType);
            g_numAtlHandleOpen++;
            garden_print ("Open Num ATL handles: %d",g_numAtlHandleOpen);
            break;

        case ACTION_OPEN_ATL_HALCLIENT:
            garden_print ("ACTION_OPEN_ATL_HALCLIENT AgpsType: %d", test_thread_agps_ext_type);
            /*
             * sleep(3);
             */
            pAGpsInterface->data_conn_open(sOptions.agpsData.apn);
            g_numAtlHandleOpen++;
            garden_print ("Open Num ATL handles: %d",g_numAtlHandleOpen);
            break;

        case ACTION_CLOSE_ATL:
            garden_print ("ACTION_CLOSE_ATL AgpsType: %d", test_thread_agps_ext_type);
            loc_extended_agps_closed(test_thread_agps_ext_type);
            decrement_atl_handle_count();
            break;

        case ACTION_CLOSE_ATL_HALCLIENT:
            garden_print ("ACTION_CLOSE_ATL_HALCLIENT AgpsType: %d", test_thread_agps_ext_type);
            pAGpsInterface->data_conn_closed();
            decrement_atl_handle_count();
            break;

        case ACTION_FAIL_ATL:
            garden_print ("ACTION_FAIL_ATL AgpsType: %d", test_thread_agps_ext_type);
            loc_extended_agps_open_failed(test_thread_agps_ext_type);
            decrement_atl_handle_count();
            break;

        case ACTION_FAIL_ATL_HALCLIENT:
            garden_print ("ACTION_FAIL_ATL_HALCLIENT AgpsType: %d", test_thread_agps_ext_type);
            pAGpsInterface->data_conn_failed();
            decrement_atl_handle_count();
            break;

        case ACTION_EXT_CLIENT_INIT:
            garden_print ("ACTION_EXT_CLIENT_INIT");
            pthread_mutex_lock (&ext_client_init_mutex);
            pthread_cond_signal (&ext_client_init_cond);
            pthread_mutex_unlock (&ext_client_init_mutex);
            break;

        case ACTION_NI_NOTIFY:
            static int pc = 0;
            pc = pc % sOptions.niResPatCount; // Cycle through pattern
            if(sOptions.networkInitiatedResponse[pc] != 3) // Don't send response if "No response" is in pattern
            {
                loc_extended_ni_respond(sNotification.notification_id,
                                        sOptions.networkInitiatedResponse[pc]);
            }
            pc++;
            break;

#ifdef TEST_ULP
        case ACTION_NLP_RESPONSE:
            garden_print("ACTION_NLP_RESPONSE \n Simulating AFW injection of network location after 5 secs\n");
            sleep(5);
            pthread_mutex_lock (&networkpos_req_mutex);
            if((pUlpNetworkInterface != NULL) && (pUlpNetworkInterface->ulp_send_network_position != NULL)){
              UlpNetworkPositionReport position_report;
              position_report.valid_flag = ULP_NETWORK_POSITION_REPORT_HAS_POSITION;
              position_report.position.latitude = 32.7;
              position_report.position.longitude = -119;
              position_report.position.HEPE = 1000;
              position_report.position.pos_source = ULP_NETWORK_POSITION_SRC_UNKNOWN;
              pUlpNetworkInterface->ulp_send_network_position(&position_report);
            }

            pthread_cond_signal (&networkpos_req_cond);
            pthread_mutex_unlock (&networkpos_req_mutex);
            break;

        case ACTION_PHONE_CONTEXT_UPDATE:
            pthread_mutex_lock (&phone_context_mutex);
            garden_print("ACTION_PHONE_CONTEXT_UPDATE \n Simulating AFW injection of phone context info\n");
            if(pUlpPhoneContextInterface != NULL) {
              UlpPhoneContextSettings settings;
              settings.context_type = ULP_PHONE_CONTEXT_GPS_SETTING |
                                      ULP_PHONE_CONTEXT_NETWORK_POSITION_SETTING |
                                      ULP_PHONE_CONTEXT_WIFI_SETTING |
                                      ULP_PHONE_CONTEXT_AGPS_SETTING;
              settings.is_gps_enabled = is_gps_enabled;
              settings.is_agps_enabled = is_agps_enabled;
              settings.is_network_position_available = is_network_position_available;
              settings.is_wifi_setting_enabled = is_wifi_setting_enabled;
              pUlpPhoneContextInterface->ulp_phone_context_settings_update(&settings);

            }
            pthread_cond_signal (&phone_context_cond);
            pthread_mutex_unlock (&phone_context_mutex);
            break;
#endif
        default:
            break;
        }
        test_thread_action = ACTION_NONE;

    } while (1);
    pthread_mutex_unlock (&test_thread_mutex);

    garden_print ("Test Thread Exit");
    return NULL;
}

void test_gps_location_cb (LocGpsLocation * location)
{
    static int callCount = 1;
    static int ttff_meas = 0;
    struct timeval result = {0,0};
    int64_t ttffSecs = 0;
    if(callCount) {
        gettimeofday(&TimeAtFirstFix,NULL);
    }
    garden_print ("## loc_gps_location_callback ##:");
    garden_print("LAT: %f, LON: %f, ACC: %f, TIME: %llu",
                 location->latitude, location->longitude, location->accuracy,
                 (long long) location->timestamp);

    if(!ttff_meas)
    {
        firstFixTime = getUpTimeSec();
#ifdef __ANDROID__
        printMarker("GPS-First Fix");
#endif
        garden_print("TTFF in Secs:: %ld, StartTime: %ld, FirstFixTime: %ld",
                     (firstFixTime - startTime), startTime, firstFixTime );
        ttff_meas = 1;
    }

    if(callCount) {
        timeval_difference(&result,&TimeAtFirstFix, &TimeAtStartNav);
        ttffSecs = (int64_t)result.tv_sec + (int64_t)result.tv_usec/(int64_t)1000000;
        garden_print("Time to First Fix in Secs:: %ld",ttffSecs);
    }
    //check if time to first fix is within threshold value
    if(ttffSecs <= sOptions.fixThresholdInSecs) {
        g_exitStatus = 1;
    }
    else {
        g_exitStatus = -1;
    }
    pthread_mutex_lock (&location_conn_mutex);
    pthread_cond_signal (&location_conn_cond);
    pthread_mutex_unlock (&location_conn_mutex);
}

void test_ulp_location_cb (UlpLocation * location, GpsLocationExtended* locationExtended, enum loc_sess_status status)
{
    garden_print("======================================================");
    garden_print ("ulp_location_cb :");
    garden_print("LAT: %f, LON: %f, ACC: %f, TIME: %llu, status: %d",
                 location->gpsLocation.latitude, location->gpsLocation.longitude,
                 location->gpsLocation.accuracy,(long long) location->gpsLocation.timestamp,
                 status);
    garden_print("======================================================");
    pthread_mutex_lock (&ulp_location_mutex);
    pthread_cond_signal (&ulp_location_cond);
    pthread_mutex_unlock (&ulp_location_mutex);
}

void test_ulp_gnss_status_cb (const LocGpsStatusValue status) {
}

void test_ulp_gnss_nmea_cb (const UlpNmea *pNmeaStr) {

    if(pNmeaStr) {
        garden_print ("NMEA : %s", pNmeaStr);
    }
}

void test_ulp_gnss_svinfo_cb (const LocGnssSvStatus *gnssSvStatus,
                              const GpsLocationExtended* locationExtended,
                              uint16_t source) {

    if(gnssSvStatus && locationExtended) {
        garden_print ("SVInfo :\n");
    }
}

void test_gps_status_cb (LocGpsStatus * status)
{

    garden_print("## loc_gps_status_callback ##:: GPS Status: %d",status->status);

    if(status->status == LOC_GPS_STATUS_ENGINE_ON)
    {
        g_checkForEngineOff = 1;
    }

    if(status->status == LOC_GPS_STATUS_ENGINE_OFF)
    {
        g_checkForEngineOff = 0;
        pthread_mutex_lock (&session_status_mutex);
        pthread_cond_signal (&session_status_cond);
        pthread_mutex_unlock (&session_status_mutex);
    }
}

void test_gps_sv_status_cb (LocGpsSvStatus * sv_info)
{

    garden_print ("## loc_gps_sv_status_callback ##:: Number of SVs: %d",sv_info->num_svs);
    if(sOptions.satelliteDetails) {
        for(int i=0;i<sv_info->num_svs; ++i)
        {
            garden_print("%02d : PRN: %04d, SNR: %09.4f, ELE: %09.4f, AZI: %09.4f",
                          i+1,sv_info->sv_list[i].prn,sv_info->sv_list[i].snr, sv_info->sv_list[i].elevation,
                          sv_info->sv_list[i].azimuth);
        }
        garden_print("Ephemeris Mask : 0x%X",sv_info->ephemeris_mask);
        garden_print("Almanac Mask: 0x%X",sv_info->almanac_mask);
        garden_print("Used in Fix Mask: 0x%X:",sv_info->used_in_fix_mask);
    }

    // if the minimum number of SVs with minimum number of SNR
    // has been satisfied then stop session
    if (sOptions.stopOnMinSvs && sOptions.stopOnMinSnr) {
        if (sOptions.stopOnMinSvs <= sv_info->num_svs) {
            int minCnSvCount = 0;
            // count number of SVs that meet the min SNR
            for(int i=0; i<sv_info->num_svs; ++i) {
                if (sv_info->sv_list[i].snr >= sOptions.stopOnMinSnr) {
                    minCnSvCount++;
                }
            }
            if (minCnSvCount >= sOptions.stopOnMinSvs){
                garden_print("Stop test, as %d SVs are seen with at least a SNR of %f",
                              sOptions.stopOnMinSvs, sOptions.stopOnMinSnr);
                pthread_mutex_lock (&location_conn_mutex);
                pthread_cond_signal (&location_conn_cond);
                pthread_mutex_unlock (&location_conn_mutex);
            }
        }
    }

}

void test_gps_nmea_cb (LocGpsUtcTime timestamp, const char *nmea, int length)
{
    if(sOptions.printNmea) {
        garden_print ("## loc_gps_nmea_callback ##:: Timestamp:%lld NMEA string length:%d",
                                    timestamp, length);
        if (length > 0)
            garden_print ("## loc_gps_nmea_callback ##:: NMEA String:%s",nmea);

    }
}

void test_gps_set_capabilities_cb (uint32_t capabilities)
{

    garden_print("## loc_gps_set_capabilities ##:");
    garden_print("Capabilities: 0x%x",capabilities);

}

void test_ext_gps_set_capabilities_cb (uint32_t capabilities)
{
    garden_print("## loc_ext_gps_set_capabilities ##:");
    garden_print("Capabilities: 0x%x",capabilities);
    pthread_mutex_lock (&test_thread_mutex);
    test_thread_action = ACTION_EXT_CLIENT_INIT;
    pthread_cond_signal (&test_thread_cond);
    pthread_mutex_unlock (&test_thread_mutex);
}

void test_gps_acquire_wakelock_cb ()
{

    garden_print ("## loc_gps_acquire_wakelock ##:");

}

void test_gps_release_wakelock_cb ()
{

    garden_print ("## loc_gps_release_wakelock ##:");

}

void test_gps_loc_request_utc_time_cb ()
{
    garden_print("## gps_loc_request_utc_time ##:");
}

void test_gnss_loc_set_system_info_cb (const LocGnssSystemInfo* info)
{
    garden_print("## test_gnss_loc_set_system_info_cb ##: %d", info->year_of_hw);
}

void test_gnss_loc_sv_status_cb(LocGnssSvStatus* sv_info)
{

    garden_print("## loc_gnss_sv_status_callback ##:: Number of SVs: %d", sv_info->num_svs);
    if (sOptions.satelliteDetails) {
        for (int i = 0; i<sv_info->num_svs; ++i)
        {
            garden_print("%02d : PRN: %04d, SNR: %09.4f, ELE: %09.4f, AZI: %09.4f"
                "EPH:%u ALM:%u USED:%u",
                i + 1, sv_info->gnss_sv_list[i].svid, sv_info->gnss_sv_list[i].c_n0_dbhz,
                sv_info->gnss_sv_list[i].elevation, sv_info->gnss_sv_list[i].azimuth,
                (sv_info->gnss_sv_list[i].flags & LOC_GNSS_SV_FLAGS_HAS_EPHEMERIS_DATA) ==
                LOC_GNSS_SV_FLAGS_HAS_EPHEMERIS_DATA,
                (sv_info->gnss_sv_list[i].flags & LOC_GNSS_SV_FLAGS_HAS_ALMANAC_DATA) ==
                LOC_GNSS_SV_FLAGS_HAS_ALMANAC_DATA,
                (sv_info->gnss_sv_list[i].flags & LOC_GNSS_SV_FLAGS_USED_IN_FIX) ==
                LOC_GNSS_SV_FLAGS_USED_IN_FIX);
        }
    }

    // if the minimum number of SVs with minimum number of SNR
    // has been satisfied then stop session
    if (sOptions.stopOnMinSvs && sOptions.stopOnMinSnr) {
        if (sOptions.stopOnMinSvs <= sv_info->num_svs) {
            int minCnSvCount = 0;
            // count number of SVs that meet the min SNR
            for (int i = 0; i<sv_info->num_svs; ++i) {
                if (sv_info->gnss_sv_list[i].c_n0_dbhz >= sOptions.stopOnMinSnr) {
                    minCnSvCount++;
                }
            }
            if (minCnSvCount >= sOptions.stopOnMinSvs) {
                garden_print("Stop test, as %d SVs are seen with at least a SNR of %f",
                    sOptions.stopOnMinSvs, sOptions.stopOnMinSnr);
                pthread_mutex_lock(&location_conn_mutex);
                pthread_cond_signal(&location_conn_cond);
                pthread_mutex_unlock(&location_conn_mutex);
            }
        }
    }

}

pthread_t test_gps_create_thread_cb (const char *name, void (*start) (void *),
                                void *arg)
{
    garden_print("## loc_gps_create_thread ##:");
    pthread_t thread_id = -1;
    garden_print ("%s", name);

    tcreatorData* tcd = (tcreatorData*)malloc(sizeof(*tcd));

    if (NULL != tcd) {
        tcd->pfnThreadStart = start;
        tcd->arg = arg;

        if (0 > pthread_create (&thread_id, NULL, my_thread_fn, (void*)tcd)) {
            garden_print ("error creating thread");
            free(tcd);
        } else {
            garden_print ("created thread");
        }
    }


    return thread_id;
}

void test_geofence_transition_callback(int32_t geofence_id,  LocGpsLocation* location,
                                         int32_t transition, LocGpsUtcTime timestamp) {
    garden_print("EVENT GPS GEOFENCE TRANSITION id=%d transition=%d lat=%8.2f long=%8.2f"
                 "accuracy=%8.2f speed=%8.2f bearing=%8.2f time=%lld\n",
                  geofence_id, transition, location->latitude, location->longitude,
                  location->accuracy, location->speed, location->bearing, timestamp);
}

void test_geofence_status_callback(int32_t status, LocGpsLocation* last_location) {
    garden_print("EVENT GPS GEOFENCE STATUS status=%d\n", status);
}

void test_geofence_add_callback(int32_t geofence_id, int32_t status) {
    garden_print("EVENT GPS GEOFENCE ADD geofence_id=%d status=%d\n", geofence_id, status);
}

void test_geofence_remove_callback(int32_t geofence_id, int32_t status) {
    garden_print("EVENT GPS GEOFENCE REMOVE geofence_id=%d status=%d\n", geofence_id, status);
}

void test_geofence_pause_callback(int32_t geofence_id, int32_t status) {
    garden_print("EVENT GPS GEOFENCE PAUSE geofence_id=%d status=%d\n", geofence_id, status);
}

void test_geofence_resume_callback(int32_t geofence_id, int32_t status) {
    garden_print("EVENT GPS GEOFENCE RESUME geofence_id=%d status=%d\n", geofence_id, status);
}

static void flp_loc_location_callback(int32_t num_locations,
    FlpExtLocation** locations,
    LocReportType reportTrigger) {

    garden_print("EVENT FLP LOCATION num_locations=%d\n", num_locations);

    if (num_locations <= 0)
        return;

    garden_print(" flags | latitude | longitude | altitude | accuracy | speed | bearing |     time      | sources \n");

    for (int i=0; i<num_locations; ++i) {
        garden_print("%6d | %8.4f | %9.4f | %8.2f | %8.2f | %5.2f | %7.2f | %13lld | %d\n",
              locations[i]->flags, locations[i]->latitude, locations[i]->longitude,
              locations[i]->altitude, locations[i]->accuracy, locations[i]->speed,
              locations[i]->bearing, locations[i]->timestamp, locations[i]->sources_used);
    }
}

static void flp_loc_acquire_wakelock() {

    garden_print("EVENT FLP ACQUIRE_WAKELOCK\n");
}

static void flp_loc_release_wakelock() {

    garden_print("EVENT FLP RELEASE_WAKELOCK\n");
}

static int flp_loc_set_thread_event(FlpExtThreadEvent event) {

    garden_print("EVENT FLP SET THREAD EVENT\n");
    return FLP_SUCCESS;
}

static void flp_loc_capabilities_callback(int capabilities) {
    garden_print("EVENT FLP_CAPABILITIES = %d ", capabilities);
    if (capabilities & LOC_CAPABILITY_GNSS) {
        garden_print("GNSS ");
    }
    if (capabilities & LOC_CAPABILITY_WIFI) {
        garden_print("WIFI ");
    }
    if (capabilities & LOC_CAPABILITY_CELL) {
        garden_print("CELL ");
    }
    garden_print("\n");

}

static void flp_loc_status_callback(int32_t status) {
    if (LOC_FLP_STATUS_LOCATION_AVAILABLE == status) {
        garden_print("EVENT FLP_STATUS = AVAILABLE\n");
    } else if (LOC_FLP_STATUS_LOCATION_UNAVAILABLE == status) {
        garden_print("EVENT FLP_STATUS = AVAILABLE\n");
    } else {
        garden_print("EVENT FLP_STATUS = INVALID STATUS!\n");
    }
}

static void flp_loc_max_power_allocated_callback(double power_mW) {
    garden_print("flp_loc_max_power_allocated_callback!\n");
}


static void test_ulp_request_phone_context_cb(UlpPhoneContextRequest *req)
{
 garden_print ("test_ulp_request_phone_context with context_type: %x,request_type: %d ",
           req->context_type, req->request_type );

 pthread_mutex_lock (&test_thread_mutex);
 test_thread_action = ACTION_PHONE_CONTEXT_UPDATE;
 pthread_cond_signal (&test_thread_cond);
 pthread_mutex_unlock (&test_thread_mutex);
}

static void test_ulp_network_location_request_cb(UlpNetworkRequestPos* req)
{
    garden_print ("test_ulp_network_location_request_cb with request_type: %d,interval %d, desired_position_source: %d ",
           req->request_type , req->interval_ms , req->desired_position_source );

    if (req->request_type == ULP_NETWORK_POS_STOP_REQUEST )
    {
       garden_print ("received network provider stop request\n");
    }
    else
    {
       sleep (1);
       pthread_mutex_lock (&test_thread_mutex);
       test_thread_action = ACTION_NLP_RESPONSE;
       pthread_cond_signal (&test_thread_cond);
       pthread_mutex_unlock (&test_thread_mutex);
    }
}

void test_loc_agps_status_callback(LocAGpsStatus* status)
{
    garden_print ("## test_loc_agps_status_callback AgpsType:%d ##",status->type);
    test_thread_agps_ext_type = status->type;
    if(sOptions.isSuccess)
    {
        if (status->status == LOC_GPS_REQUEST_AGPS_DATA_CONN) {
            action_open_atl(ACTION_OPEN_ATL_HALCLIENT);
        }
        else if (status->status == LOC_GPS_RELEASE_AGPS_DATA_CONN) {
            action_close_atl(ACTION_CLOSE_ATL_HALCLIENT);
        }
    }
    else
    {
        garden_print ("got status %d, sending failure ", status->status);
        action_fail_atl(ACTION_FAIL_ATL_HALCLIENT);
    }

}

void test_agps_status_cb (AGpsExtStatus * status)
{
    garden_print ("## test_agps_status_cb AgpsType:%d ##",status->type);
    test_thread_agps_ext_type = status->type;
    if(sOptions.niSuplFlag) {
        static int niCount = 1;
        if(niCount <= sOptions.niCount) {
            if(sOptions.isSuccess)
            {
                if (status->status == LOC_GPS_REQUEST_AGPS_DATA_CONN) {
                    action_open_atl(ACTION_OPEN_ATL);
                }
                else if (status->status == LOC_GPS_RELEASE_AGPS_DATA_CONN) {
                    action_close_atl(ACTION_CLOSE_ATL);
                }
            }
            else
            {
                garden_print ("got status %d, sending failure ", status->status);
                action_fail_atl(ACTION_FAIL_ATL);
            }
        }
        notify_main_thread_of_atlcb();
        niCount++;
    }
    else
    {
        if(sOptions.isSuccess)
        {
            if (status->status == LOC_GPS_REQUEST_AGPS_DATA_CONN) {
                action_open_atl(ACTION_OPEN_ATL);
            }
            else if (status->status == LOC_GPS_RELEASE_AGPS_DATA_CONN) {
                action_close_atl(ACTION_CLOSE_ATL);
            }
        }
        else
        {
            garden_print ("got status %d, sending failure ", status->status);
            action_fail_atl(ACTION_FAIL_ATL);
        }
    }
}

static void test_gps_ni_notify_cb(LocGpsNiNotification *notification, bool esEnabled)
{

    static int niCount = 1;
    garden_print ("## loc_gps_ni_notify_callback ##:%d",niCount);

    sNotification = *notification;
    garden_print ("ACTION_NI_NOTIFY: notification_id %d, ni_type %d, "
                  "notify_flags %d, timeout %d, default_response %d, "
                  "requestor_id %s, text %s, requestor_id_encoding %d, "
                  "text_encoding %d, extras %s, esEnabled %d\n",
        sNotification.notification_id,
        (int) sNotification.ni_type,
        (int) sNotification.notify_flags,
        sNotification.timeout,
        sNotification.default_response,
        sNotification.requestor_id,
        sNotification.text,
        sNotification.requestor_id_encoding,
        sNotification.text_encoding,
        sNotification.extras,
        esEnabled);


    // Do not notify test thread if callback count exceeds command line option
    // for number of back to back tests
    if(niCount <= sOptions.niCount)
    {
        pthread_mutex_lock (&test_thread_mutex);
        test_thread_action = ACTION_NI_NOTIFY;
        pthread_cond_signal (&test_thread_cond);
        pthread_mutex_unlock (&test_thread_mutex);
    }
    // Notify main thread everytime callback is invoked.
    pthread_mutex_lock (&wait_count_mutex);
    pthread_cond_signal (&wait_count_cond);
    pthread_mutex_unlock (&wait_count_mutex);
    niCount++;
}


LocGpsCallbacks myCallbacks = {
    sizeof (LocGpsCallbacks),
    test_gps_location_cb,
    test_gps_status_cb,
    test_gps_sv_status_cb,
    test_gps_nmea_cb,
    test_gps_set_capabilities_cb,
    test_gps_acquire_wakelock_cb,
    test_gps_release_wakelock_cb,
    test_gps_create_thread_cb,
    test_gps_loc_request_utc_time_cb,

    test_gnss_loc_set_system_info_cb,
    test_gnss_loc_sv_status_cb,
};

LocAGpsCallbacks myAGpsCallbacks = {
    test_loc_agps_status_callback,
    test_gps_create_thread_cb,
};

LocGpsGeofenceCallbacks sGpsGeofenceCallbacks = {
    test_geofence_transition_callback,
    test_geofence_status_callback,
    test_geofence_add_callback,
    test_geofence_remove_callback,
    test_geofence_pause_callback,
    test_geofence_resume_callback,
    test_gps_create_thread_cb
};

#ifdef TEST_ULP
UlpEngineCallbacks sUlpEngineCallbacks = {
    sizeof(UlpEngineCallbacks),
    test_ulp_location_cb,
    test_ulp_gnss_status_cb,
    test_gps_create_thread_cb,
    test_ulp_gnss_nmea_cb,
    test_ulp_gnss_svinfo_cb
};

UlpPhoneContextCallbacks pUlpPhoneContextCallbacks = {
    test_ulp_request_phone_context_cb,
};

UlpNetworkLocationCallbacks pUlpNetworkLocationCallbacks = {
    test_ulp_network_location_request_cb,
};
#endif

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

void set_phone_context()
{
    garden_print("set_phone_context ");
    is_gps_enabled                = TRUE;
    is_network_position_available = TRUE;
    is_wifi_setting_enabled       = TRUE;
    is_battery_charging           = TRUE;
    //Simulate phone context update from AFW
    pthread_mutex_lock (&test_thread_mutex);
    test_thread_action = ACTION_PHONE_CONTEXT_UPDATE;
    pthread_cond_signal (&test_thread_cond);
    pthread_mutex_unlock (&test_thread_mutex);

}

AGpsExtCallbacks myAGpsExtCallbacks = {
    test_agps_status_cb,
    test_gps_create_thread_cb
};

GpsExtCallbacks myGpsExtCallbacks = {
    sizeof(GpsExtCallbacks),
    test_ext_gps_set_capabilities_cb,
    test_gps_acquire_wakelock_cb,
    test_gps_release_wakelock_cb,
    test_gps_create_thread_cb,
    NULL
};

GpsNiExtCallbacks myGpsNiExtCallbacks = {
    test_gps_ni_notify_cb
};

extern "C" const LocGpsInterface *loc_eng_gps_get_hardware_interface ();

/*=============================================================================
  FUNCTION test_main

    Android Framework simulation entry point.

    Includes individual tests for specific features.

    Search the makefile for "TEST_ANDROID_GPS_FLAGS" and uncomment the
    flags that correspond to desired tests.

=============================================================================*/
int test_main ()
{
    int rc = 0;
    uint32_t port = 10001;
    uint32_t targetType;
    pthread_t tid;
    int r;
    test_thread_action = ACTION_NONE;
    test_thread_agps_ext_type = LOC_AGPS_TYPE_INVALID;

    mutex_init();

    pthread_create (&tid, NULL, test_thread, NULL);

    startTime = firstFixTime = 0;
    startTime = getUpTimeSec();

    /* In order to enable native AGPS, we must not provide a status
     * callback */
    if (sOptions.isNativeAgpsEnabled) {
        garden_print("Native AGPS enabled");
        myAGpsExtCallbacks.status_cb = NULL;
    }
    loc_extended_init(&myGpsExtCallbacks);
    // wait for extended client to init
    pthread_mutex_lock (&ext_client_init_mutex);
    pthread_cond_wait (&ext_client_init_cond, &ext_client_init_mutex);
    pthread_mutex_unlock (&ext_client_init_mutex);

    loc_extended_agps_init(&myAGpsExtCallbacks);
    loc_extended_ni_init(&myGpsNiExtCallbacks);

    pGpsInterface = (LocGpsInterface *) loc_eng_gps_get_hardware_interface ();

    if (NULL == pGpsInterface) {
        mutex_destroy();
        garden_print(" Could not get a handle to LocGpsInterface, Cannot proceed ");
        return -1;
    }

    rc = pGpsInterface->init(&myCallbacks);
    garden_print ("initialize GPS interface returned %d", rc);

    pAGpsInterface =
        (const LocAGpsInterface*)pGpsInterface->get_extension(LOC_AGPS_INTERFACE);
    if (pAGpsInterface) {
        if (!sOptions.isNativeAgpsEnabled) {
            pAGpsInterface->init(&myAGpsCallbacks);
        }
        else {
            garden_print("Native AGPS enabled");
        }
    }
    else {
        garden_print("Failed to get LOC_AGPS_INTERFACE");
        return -1;
    }

    pGpsGeoFencingInterface =
        (const LocGpsGeofencingInterface*)pGpsInterface->get_extension(LOC_GPS_GEOFENCING_INTERFACE);
    if (pGpsGeoFencingInterface) {
        pGpsGeoFencingInterface->init(&sGpsGeofenceCallbacks);
    } else {
        garden_print("Failed to get GPS_GEOFENCING_INTERFACE");
    }
    if (sFlpLocationClient == NULL) {
        sFlpLocationClient = FlpLocationClient::createInstance();
    }

    if (sFlpLocationClient) {
        rc = sFlpLocationClient->flp_init(&sFlpCallbacks);
    }
    else {
        garden_print("Failed to create FlpLocationClient");
    }

    pGnssConfigurationInterface = (const LocGnssConfigurationInterface*)
                                    pGpsInterface->get_extension(LOC_GNSS_CONFIGURATION_INTERFACE);
    if (NULL == pGnssConfigurationInterface) {
        garden_print("Failed to get GNSS_CONFIGURATION_INTERFACE");
    }

    if (sOptions.s & 2) {
        loc_ext_init();
    }

    pAgpsRilInterface = (LocAGpsRilInterface *) pGpsInterface->get_extension (LOC_AGPS_RIL_INTERFACE);
    if (NULL == pAgpsRilInterface) {
        //mutex_destroy();
        garden_print("Could not get a handle to AgpsRilInterface...but proceeding");
        //return -1;
    }

    // Initialize AGPS server settings
    if(sOptions.agpsData.agpsType == LOC_AGPS_TYPE_SUPL)
    {
        rc = loc_extended_agps_set_server(sOptions.agpsData.agpsType,
                                          sOptions.agpsData.suplHost,
                                          sOptions.agpsData.suplPort );

    }
    else if(sOptions.agpsData.agpsType == LOC_AGPS_TYPE_C2K)
    {
       rc = loc_extended_agps_set_server(sOptions.agpsData.agpsType,
                                         sOptions.agpsData.c2kHost,
                                         sOptions.agpsData.c2kPort );
    }

    if(sOptions.deleteAidingDataMask != 0){
        pGpsInterface->delete_aiding_data (sOptions.deleteAidingDataMask);
    }

    if(sOptions.location.flags != 0 ) {
        pGpsInterface->inject_location(sOptions.location.latitude, sOptions.location.longitude, sOptions.location.accuracy);
    }
#ifdef USE_GLIB
    IIzatManager * pIzatManager = getIzatManager (OSFramework::getOSFramework());
    if(NULL == pIzatManager){
            garden_print ("getIzatManager returned NULL!!");
    } else {
        pIzatManager->enableProvider(IZAT_STREAM_NETWORK);
    }
    pUlpEngineInterface =
        (const UlpEngineInterface*)ulp_get_extension(ULP_ENGINE_INTERFACE);
   pUlpNetworkInterface =
        (UlpNetworkInterface*)ulp_get_extension(ULP_NETWORK_INTERFACE);
   pUlpPhoneContextInterface =
        (const UlpPhoneContextInterface*)ulp_get_extension(ULP_PHONE_CONTEXT_INTERFACE);
   if((NULL == pUlpEngineInterface)||
        (NULL == pUlpNetworkInterface)||
        (NULL == pUlpPhoneContextInterface)) {
              ALOGE("Error in classInit.ulp_get_extension is null ");
    }
#endif
#ifdef __ANDROID__
    ulp_init(&sUlpEngineCallbacks, &pUlpNetworkLocationCallbacks, &pUlpPhoneContextCallbacks);
    pUlpEngineInterface =
        (const UlpEngineInterface*)ulp_get_extension(ULP_ENGINE_INTERFACE);
    pUlpNetworkInterface =
        (UlpNetworkInterface*)ulp_get_extension(ULP_NETWORK_INTERFACE);
    pUlpPhoneContextInterface =
           (const UlpPhoneContextInterface*)ulp_get_extension(ULP_PHONE_CONTEXT_INTERFACE);
    if((NULL == pUlpEngineInterface)||
       (NULL == pUlpNetworkInterface)||
       (NULL == pUlpPhoneContextInterface)) {
        LOC_LOGE("Error in classInit.ulp_get_extension is null ");
    }

    if(NULL != pUlpEngineInterface)
    {
       pthread_mutex_lock (&ulp_location_mutex);
       pUlpEngineInterface->system_update(ULP_LOC_EVENT_OBSERVER_STOP_EVENT_TX);
       pthread_mutex_unlock (&ulp_location_mutex);
    }

#endif
#if defined (TEST_ULP) && defined (__ANDROID__)
    if((sOptions.ulpTestCaseNumber > 0) && (sOptions.ulpTestCaseNumber < 13)) {
        pthread_mutex_lock (&phone_context_mutex);
        garden_print ("Waiting for Ulp Test to finish...");
        garden_print ("Running ULP test case number: %d",sOptions.ulpTestCaseNumber);

        test_ulp(sOptions.ulpTestCaseNumber);
        pthread_cond_wait (&phone_context_cond, &phone_context_mutex);
        pthread_mutex_unlock (&phone_context_mutex);
        sleep(3);
        garden_print ("Ulp Test - 1 session complted!");
    }

#endif
   if((sOptions.zppTestCaseNumber > 0) && (sOptions.zppTestCaseNumber < 5)) {
        garden_print ("Running ZPP test case number: %d",sOptions.zppTestCaseNumber);
        test_zpp(sOptions.zppTestCaseNumber);
    } else
    {
        garden_print ("Invalid ZPP test case number: %d",sOptions.zppTestCaseNumber);
    }

    if (NULL != pGnssConfigurationInterface &&
            NULL != pGnssConfigurationInterface->configuration_update) {
        // set SUPL_MODE based on position mode.
        // For MSB(1) set 0x1,For MSA(2) 0x3,For standalone(0) 0x0.
        char config_data[20];
        //default to LOC_GPS_POSITION_MODE_STANDALONE
        int supl_mode=0;
        if (sOptions.positionMode == LOC_GPS_POSITION_MODE_MS_BASED) {
            supl_mode=1;
        }
        else if (sOptions.positionMode == LOC_GPS_POSITION_MODE_MS_ASSISTED) {
            supl_mode=3;
        }

        snprintf(config_data, sizeof(config_data), "%s=%x","SUPL_MODE",supl_mode);
        garden_print ("invoke gnss configuration_update: %s",config_data);
        pGnssConfigurationInterface->configuration_update(config_data, strlen(config_data));
    }
    usleep(200000); //Allow ULP initialization to complete before start fix request

    for(int k=0;k<sOptions.l; ++k)
    {
        garden_print("Session %d:",k);
        if (NULL != pGpsInterface) {
            rc = pGpsInterface->set_position_mode (sOptions.positionMode,
                                               sOptions.r , sOptions.interval,
                                               sOptions.accuracy,sOptions.responseTime);
            garden_print ("set_position_mode returned %d", rc);

            gettimeofday(&TimeAtStartNav,NULL);
            rc = pGpsInterface->start ();
            garden_print ("start GPS interface returned %d", rc);
#ifdef USE_GLIB
            //In LE, we have to push the phone settings after first start fix request
            if(k == 0){
                set_phone_context();
            }
#endif
        }

        if (sOptions.s & 2) {
            rc = loc_ext_set_position_mode(sOptions.positionMode,
                                           sOptions.r == LOC_GPS_POSITION_RECURRENCE_SINGLE,
                                           sOptions.interval, sOptions.responseTime, NULL);
            garden_print ("vzw_set_position_mode returned %d", rc);

            rc = loc_ext_start();
            garden_print ("vzw start GPS interface returned %d", rc);
        }

        if (sOptions.r == LOC_GPS_POSITION_RECURRENCE_SINGLE) {
            struct timespec ts;

            garden_print ("Waiting for location callback...");

            pthread_mutex_lock (&location_conn_mutex);
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += sOptions.t;

            pthread_cond_timedwait(&location_conn_cond, &location_conn_mutex, &ts);
            pthread_mutex_unlock (&location_conn_mutex);
        } else {
            garden_print ("sleep for %d seconds", sOptions.t);
            sleep(sOptions.t);
        }

        if( (pAgpsRilInterface != NULL) && (sOptions.rilInterface) ) {
            pAgpsRilInterface->update_network_availability(88, "MY_APN_FOR_RIL_TEST");
            garden_print ("Ril Interface test Done...");
        }


        rc = pGpsInterface->stop ();
        garden_print ("stop GPS interface returned %d", rc);

        if (sOptions.s & 2) {
            rc = loc_ext_stop();
            garden_print ("vzw stop GPS interface returned %d", rc);
        }
    } // End of for loop for sessions

    int niCount = 1;
    for(;niCount<= sOptions.niCount;++niCount)
    {
        //wait for NI back to back test to finish
        garden_print("Waiting for NI Back to back tests to finish...");
        pthread_mutex_lock (&wait_count_mutex);
        pthread_cond_wait(&wait_count_cond, &wait_count_mutex);
        pthread_mutex_unlock (&wait_count_mutex);

        // IF NI SUPL back to back tests are being conducted
        // wait for atl also
        if(sOptions.niSuplFlag) {
            // If NI SUPL tests are being conducted wait for ATL callback
            garden_print("Waiting for ATL Callback...");
            pthread_mutex_lock (&wait_atlcb_mutex);
            pthread_cond_wait(&wait_atlcb_cond, &wait_atlcb_mutex);
            pthread_mutex_unlock (&wait_atlcb_mutex);
        }

    }

    if(g_checkForEngineOff) {
        garden_print ("Waiting for Engine off...");
        pthread_mutex_lock (&session_status_mutex);
        pthread_cond_wait(&session_status_cond, &session_status_mutex);
        pthread_mutex_unlock (&session_status_mutex);
    }
    else {
        garden_print ("Engine is off, g_checkForEngineOff : %d...", g_checkForEngineOff);
    }

    if (g_numAtlHandleOpen) {
        // There are few atl handle's open. wait till they are closed.
        garden_print ("Waiting for %d ATL handles to close...", g_numAtlHandleOpen);
        pthread_mutex_lock (&close_atl_handles_mutex);
        pthread_cond_wait(&close_atl_handles_cond, &close_atl_handles_mutex);
        pthread_mutex_unlock (&close_atl_handles_mutex);

        // wait for last atl handle to close properly
        usleep(100000);
    }

    if (sOptions.trackUsingFlp == TRUE)  {
        garden_print("Start tracking session using HW FLP API");

#ifdef USE_GLIB
        set_phone_context();
#endif

        FlpExtBatchOptions options;
        options.flags = 0;
        options.distance_ms = 0;

        options.flags |= LOC_FLP_BATCH_CALLBACK_ON_LOCATION_FIX;
        options.period_ns = (int64_t)sOptions.interval*1000000;

        if (sFlpLocationClient) {
            sFlpLocationClient->flp_start_session(1, &options);
        }
        else {
            garden_print("sFlpLocationClient is null\n");
        }

        garden_print("sleep for %d secs", sOptions.t);
        sleep(sOptions.t);
    }

    if (sOptions.keepAlive)
    {
        garden_print("Garden App is alive. Hit Ctrl -C to exit");
        sleep(3600);
    }

    /*********
    *
    *   ## IMPORTANT ##
    *   This tracking option will start a tracking session in continous loop
    *   Alwasys keep this as a last test case and add new test cases
    *   before this option
    *
    ******* */
    if(sOptions.tracking == 1)
    {
       g_run_active_client = TRUE;
       run_tracking_session();
    }
    loc_extended_cleanup();
    pGpsInterface->cleanup ();

    if (sOptions.s & 2)
        loc_ext_cleanup();

    garden_print ("cleanup GPS interface returned ");


    pthread_mutex_lock (&test_thread_mutex);
    test_thread_action = ACTION_QUIT;
    pthread_cond_signal (&test_thread_cond);
    pthread_mutex_unlock (&test_thread_mutex);

    garden_print ("wait for pthread_join");

    void *ignored;
    pthread_join (tid, &ignored);
    garden_print ("pthread_join done");

    mutex_destroy();

/* #ifndef UDP_XPORT */
/*   ipc_router_core_deinit(); */
/* #endif */
    garden_print ("GARDEn Tests Finished!");

    return g_exitStatus;
        }

void printHelp(char **arg)
{
     garden_print("usage: %s -r <1|0> -t <xxx> -u <1-12>", arg[0]);
     garden_print("    -r:  RECURRENCE 1:SINGLE; 0:PERIODIC; Defaults: %d", sOptions.r);
     garden_print("    -l:  Number of sessions to loop through. Takes an argument. An argument of 0 means no sessions. Defaults:%d", sOptions.l);
     garden_print("    -t:  User defined length of time to issue stop navigation. Takes an argument. Time in seconds Defaults: %d", sOptions.t);
     garden_print("    -b:  run backwards compatibility tests (legacy mode). Takes and argument 1 for true and 0 for false. Defaults: %d ",(int)sOptions.b);
     garden_print("    -u:  run specified ULPLite test case Defaults: %d", sOptions.ulpTestCaseNumber);
     garden_print("    -s:  stacks to test: 1:afw; 2:vzw; 3:(afw | vzw, default) Defaults: %d",sOptions.s);
     garden_print("    -d:  Delete aiding data: Takes a hexadecimal mask as an argument as given in gps.h Defaults: 0x%X ",sOptions.deleteAidingDataMask);
     garden_print("    -m:  Position Mode. Takes an argument 0:LOC_GPS_POSITION_MODE_STANDALONE, 1:LOC_GPS_POSITION_MODE_MS_BASED, 2:LOC_GPS_POSITION_MODE_MS_ASSISTED Defaults: %d ", sOptions.positionMode);
     garden_print("    -i:  Interval. Takes an argument. Time in milliseconds between fixes Defaults: %d", sOptions.interval);
     garden_print("    -a:  Accuracy. Takes an argument. Accuracy in meters Defaults: %d ", sOptions.accuracy);
     garden_print("    -x:  Response Time. Takes an argument. Requested time to first fix in milliseconds Defaults: %d" , sOptions.responseTime);
     garden_print("    -P:  Inject Position. Takes 3 arguments seperated by a COMMA. Latitude, Longitude, and accuracy Defaults: %f,%f,%d ",sOptions.location.latitude,sOptions.location.longitude,(int)sOptions.location.accuracy);
     garden_print("    -N:  Network Initiated. Takes 2 arguments separated by COMMA. First being the number of back to back NI tests and second being a COMMA separated pattern of  1:Accept, 2:Deny,or 3:No Response Defaults: %d:%d",sOptions.niCount,0);
     for(int i = 0; i<sOptions.niResPatCount; ++i) { garden_print("%12d",sOptions.networkInitiatedResponse[i]); }
     garden_print("    -S:  ATL Success. Takes 3 arguments seperated by a COMMA. AgpsType, Apn, AgpsBearerType Defaults: %d,%s,%d,0x%X,%s,%d,%s,%d ", (int)sOptions.agpsData.agpsType, sOptions.agpsData.apn, (int)sOptions.agpsData.agpsBearerType,(int)sOptions.agpsData.suplVer,sOptions.agpsData.suplHost,sOptions.agpsData.suplPort,sOptions.agpsData.c2kHost,sOptions.agpsData.c2kPort);
     garden_print("    -f:  ATL Failure. Takes 1 argument. AgpsType Defaults: %d ", sOptions.agpsData.agpsType);
     garden_print("    -L:  Native AGPS via LocNetIface. Takes no argument. Defaults: %d", sOptions.isNativeAgpsEnabled);
     garden_print("    -g:  Test Ril Interface. Takes an argument 0 or 1. Defaults: %d",sOptions.rilInterface);
     garden_print("    -c:  gps.conf file. Takes an argument. The argument is the path to gps.conf file. Defaults: %s",sOptions.gpsConfPath);
     garden_print("    -j:  Using this option will disable garden from doing an automatic time injection. Takes an argument 0: Enable, 1: Disable. Defaults: %d",sOptions.disableAutomaticTimeInjection);
     garden_print("    -k:  used in conjunction with -N option to indicate that the test being conducted is an NI SUPL test. Takes no arguments.");
     garden_print("    -n:  Use this option to print nmea string, timestamp and length. Takes no arguments. Defaults:%d",sOptions.printNmea);
     garden_print("    -y:  Use this option to print detailed info on satellites in view. Defaults:%d",sOptions.satelliteDetails);
     garden_print("    -o:  This option, when used, will enforce a check to determine if time to first fix is within a given threshold value. Takes one argument, the threshold value in seconds. Defaults: %d",sOptions.fixThresholdInSecs);
     garden_print("    -q:  This option is used to enable/disable XTRA. Takes one argument. 0:disable, 1:enable. Defaults to %d", sOptions.enableXtra);
     garden_print("    -A:  Minimum number of SVs seen in combination with -B option to determine when to stop the test without actually getting a position report to save test time");
     garden_print("    -B:  Minimum SNR for each SV seen in -A option to determine when to stop the test  without actually getting a position report to save test time");
     garden_print("    -T:  Start a tracking session *WARNING* this tracking session will run until process is alive Ctrl-C to exit");
     garden_print("    -Z:  ZPP Test case number 1 to 5 - Default: 4");
     garden_print("    -F:  Start a tracking session using HW FLP API");
     garden_print("    -K:  Keep garden-app alive. Press Ctrl-C to exit.");
     garden_print("    -h:  print this help");
}


int main (int argc, char *argv[])
{
    int result = 0;
    int opt;
    extern char *optarg;
    char *argPtr;
    char *tokenPtr;
    LEGACY = FALSE;

    // Initialize gps conf path
    strlcpy(sOptions.gpsConfPath, LOC_PATH_GPS_CONF, sizeof(sOptions.gpsConfPath));

    while ((opt = getopt (argc, argv, "r:l:t:u:h:b:s:d:m:i:a:x:P:N:S:f:Lg:D:w:c:j:knye:o:z:q:A:B:T::Z:KF")) != -1) {
        switch (opt) {
        case 'r':
            sOptions.r = atoi(optarg);
            garden_print("Recurrence:%d",sOptions.r);
            break;
        case 'l':
            sOptions.l = atoi(optarg);
            garden_print("Number of Sessions to loop through:%d",sOptions.l);
            break;
        case 't':
            sOptions.t = atoi(optarg);
            garden_print("User defined length of time to issue stop navigation:%d",sOptions.t);
            break;
        case 'b':
            sOptions.b = atoi(optarg);
            garden_print("Run backward compatibility tests:%d",sOptions.b);
            break;
        case 'u':
            sOptions.ulpTestCaseNumber = atoi(optarg);
            garden_print("ulptestCase number: %d \n",sOptions.ulpTestCaseNumber);
            break;
        case 's':
            sOptions.s = atoi(optarg);
            garden_print("Stacks to test:%d",sOptions.s);
            break;
        case 'd':
            sOptions.deleteAidingDataMask = strtoll(optarg,NULL,16);
            garden_print("Delete Aiding Mask:%x",sOptions.deleteAidingDataMask);
            break;
        case 'm':
            sOptions.positionMode = atoi(optarg);
            garden_print("Position Mode:%d",sOptions.positionMode);
            break;
        case 'i':
            sOptions.interval = atoi(optarg);
            garden_print("Interval:%d",sOptions.interval);
            break;
        case 'a':
            sOptions.accuracy = atoi(optarg);
            garden_print("Accuracy:%d",sOptions.accuracy);
            break;
        case 'x':
            sOptions.responseTime = atoi(optarg);
            garden_print("Response Time:%d",sOptions.responseTime);
            break;
        case 'P':
            sOptions.location.flags = 0x0011;
            tokenPtr = strtok_r(optarg, ",", &argPtr);
            if(tokenPtr != NULL) {
                sOptions.location.latitude = atof(tokenPtr);
                tokenPtr = strtok_r(NULL, ",", &argPtr);
                if(tokenPtr != NULL) {
                    sOptions.location.longitude = atof(tokenPtr);
                    tokenPtr = strtok_r(NULL, ",", &argPtr);
                    if(tokenPtr != NULL) {
                        sOptions.location.accuracy = atoi(tokenPtr);
                    }
                }
            }
            garden_print("Inject Position:: flags:%x, lat:%f, lon:%f, acc:%d",sOptions.location.flags,
                    sOptions.location.latitude, sOptions.location.longitude,sOptions.location.accuracy);
            break;
        case 'N':
            // Number of back to back tests
            tokenPtr = strtok_r(optarg, ",", &argPtr);
            if(tokenPtr != NULL) {
                sOptions.niCount = atoi(tokenPtr);
                if(sOptions.niCount > 0)
                {
                   char *ret;
                   while((ret = strtok_r(NULL, ",", &argPtr)) != NULL)
                   {
                      sOptions.networkInitiatedResponse[sOptions.niResPatCount++] = atoi(ret);
                   }
                }
            }
            garden_print("Number of back to back NI tests : %d",sOptions.niCount);
            break;
        case 'S':
            tokenPtr = strtok_r(optarg, ",", &argPtr);
            if(tokenPtr != NULL) {
                sOptions.agpsData.agpsType = (LocAGpsType)atoi(tokenPtr);
                tokenPtr = strtok_r(NULL, ",", &argPtr);
                if(tokenPtr != NULL) {
                    sOptions.agpsData.apn = tokenPtr;
                    tokenPtr = strtok_r(NULL, ",", &argPtr);
                    if(tokenPtr != NULL) {
                        sOptions.agpsData.agpsBearerType = (AGpsBearerType)atoi(tokenPtr);
                    }
                }
            }
            sOptions.isSuccess = 1;
            garden_print("Success:: Agps Type:%d, apn:%s, Agps Bearer Type:%d, success:%d",
            sOptions.agpsData.agpsType, sOptions.agpsData.apn, sOptions.agpsData.agpsBearerType, sOptions.isSuccess);
            break;
        case 'f':
            sOptions.agpsData.agpsType = (LocAGpsType)atoi(optarg);
            sOptions.isSuccess = 0; // Failure
            garden_print("Failure:: Agps type:%d, success:%d",
            sOptions.agpsData.agpsType, sOptions.isSuccess);
            break;
        case 'L':
            sOptions.isNativeAgpsEnabled = true;
            garden_print("Native AGPS via LocNetIface is enabled.");
            break;
        case 'g':
            sOptions.rilInterface = atoi(optarg);
            garden_print("Test Ril Interface:%d",sOptions.rilInterface);
            break;
        case 'c':
            strlcpy(sOptions.gpsConfPath,optarg,256);
            // Initialize by reading the gps.conf file
            UTIL_READ_CONF(sOptions.gpsConfPath, cfg_parameter_table);
            garden_print("Parameters read from the config file :");
            garden_print("**************************************");
            garden_print("SUPL_VER      : 0x%X",sOptions.agpsData.suplVer);
            garden_print("SUPL_HOST     : %s",sOptions.agpsData.suplHost);
            garden_print("SUPL_PORT     : %ld",sOptions.agpsData.suplPort);
            garden_print("C2K_HOST      : %s",sOptions.agpsData.c2kHost);
            garden_print("C2K_PORT      : %ld",sOptions.agpsData.c2kPort);
            garden_print("**************************************");
            break;
        case 'j':
            sOptions.disableAutomaticTimeInjection = 1;
            garden_print("Automatic time injections is disabled");
            break;
        case 'k':
            sOptions.niSuplFlag = 1;
            garden_print("NI SUPL flag:%d",sOptions.niSuplFlag);
            break;
        case 'n':
            sOptions.printNmea = 1;
            garden_print("Print NMEA info:%d",sOptions.printNmea);
            break;
        case 'y':
            sOptions.satelliteDetails = 1;
            garden_print("Print Details Satellites in View info:%d",sOptions.satelliteDetails);
            break;
        case 'o':
            sOptions.fixThresholdInSecs = atoi(optarg);
            garden_print("Time to first fix threshold value in seconds : %d", sOptions.fixThresholdInSecs);
            break;
        case 'z':
            sOptions.zppTestCaseNumber = atoi(optarg);
            garden_print("ZPP testCase: %d \n",sOptions.zppTestCaseNumber);
            break;
        case 'q':
            sOptions.enableXtra = atoi(optarg);
            garden_print("Xtra Enabled: %d",sOptions.enableXtra);
            break;
        case 'A':
            sOptions.stopOnMinSvs = atoi(optarg);
            garden_print("Stop on Minimum Svs: %d",sOptions.stopOnMinSvs);
            break;
        case 'B':
            sOptions.stopOnMinSnr = atof(optarg);
            garden_print("Stop on Minimum SNR: %f",sOptions.stopOnMinSnr);
            break;
        case 'T':
            sOptions.tracking = 1;
            garden_print("Start Tracking Session -- continuous untill processs is alive press Ctrl-C to exit");
            break;
        case 'Z':
            sOptions.zppTestCaseNumber =  atoi(optarg);
            garden_print("ZPP Test number: %d",sOptions.zppTestCaseNumber);
            break;
        case 'F':
            sOptions.trackUsingFlp = TRUE;
            break;
        case 'K':
            sOptions.keepAlive = true;
            garden_print("Keep garden_app alive. Press Ctrl-C to exit");
            break;
        case 'h':
        default:
            printHelp(argv);
            return 0;
        }
    }

    garden_print("Starting GARDEn");
    result = test_main();

    garden_print("Exiting GARDEn");
    return result;
}

#ifdef TEST_ULP
void test_ulp(int ulptestCase)
{
   garden_print ("Starting Ulp test cases\n");
   int rc = 0;

   if(NULL == pUlpEngineInterface){
      garden_print ("In test_ulp. Error.pUlpEngineInterface null\n");
      return;
   }

   switch (ulptestCase)
   {
    // Run test scenario 1
  // GPS tracking session when GPS is enabled
  // Remove GPS tracking session
    case 1:
     {

      UlpLocationCriteria criteria;

      // set up global phone context
      is_gps_enabled                = TRUE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_GNSS;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 5;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);
      rc = pGpsInterface->start ();
      garden_print ("start GPS interface for ulp test returned %d", rc);
      sleep (5);
      criteria.action = ULP_REMOVE_CRITERIA;
      pUlpEngineInterface->update_criteria(criteria);
      rc = pGpsInterface->stop ();
      garden_print ("stop GPS interface for ulp test returned %d", rc);
      sleep (3);
     }
     break;
     // Network tracking session when network is enabled
     // Remove network tracking session
    case 2:
     {
      // set up global phone context
      is_gps_enabled                = TRUE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;

      UlpLocationCriteria criteria;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_HYBRID;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;

      // assume in seconds
      criteria.min_interval = 5;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);
      rc = pGpsInterface->start ();
      sleep (10);

      criteria.action = ULP_REMOVE_CRITERIA;
      pUlpEngineInterface->update_criteria(criteria);
      rc = pGpsInterface->stop ();
      sleep (10);
     }
     break;
     // Run test scenario 3
     // GPS tracking session when GPS is disabled
     // Remove GPS tracking session
     //
     // Result: No GPS session should be started
    case 3:
     {
      // set up global phone context
      is_gps_enabled                = FALSE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;

      UlpLocationCriteria criteria;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_GNSS;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 5;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);
      rc = pGpsInterface->start ();
      sleep (5);
      criteria.action = ULP_REMOVE_CRITERIA;
      pUlpEngineInterface->update_criteria(criteria);
      rc = pGpsInterface->stop ();
      sleep (3);
     }
     break;
     // Run test scenario 4
     // GPS tracking session when GPS is enabled
     // While tracking session is on progress, GPS is disabled
     // Remove GPS tracking session
     //
     // Result: GPS tracking session is started, and then stopped
     // when GPS is disabled
    case 4:
     {
      // set up global phone context
      is_gps_enabled                = TRUE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;

      UlpLocationCriteria criteria;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_GNSS;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 5;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);
      rc = pGpsInterface->start();
      sleep (4);

      // set up global phone context
      is_gps_enabled                = FALSE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;
      //Simulate phone context update from AFW
      pthread_mutex_lock (&test_thread_mutex);
      test_thread_action = ACTION_PHONE_CONTEXT_UPDATE;
      pthread_cond_signal (&test_thread_cond);
      pthread_mutex_unlock (&test_thread_mutex);

      sleep (1);
      criteria.action = ULP_REMOVE_CRITERIA;
      pUlpEngineInterface->update_criteria(criteria);
      rc = pGpsInterface->stop ();
      sleep (3);
     }
     break;
     // Run test scenario 5
     // GPS tracking session when GPS is disabled
     // later on, user enables GPS setting
     // later on, GPS tracking session is removed
     //
     // Result: GPS tracking session is only started when gps setting is enabled,
     // and then stopped when the request is removed
    case 5:
     {
      // set up global phone context
      is_gps_enabled                = FALSE;
      is_network_position_available = FALSE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;

      UlpLocationCriteria criteria;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_GNSS;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 5;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);
      rc = pGpsInterface->start();
      sleep (4);

      garden_print("+++++++++++Ulp TC 5 +++++++++\n");
      // set up global phone context
      is_gps_enabled                = TRUE;
      is_network_position_available = FALSE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;
      pthread_mutex_lock (&test_thread_mutex);
      test_thread_action = ACTION_PHONE_CONTEXT_UPDATE;
      pthread_cond_signal (&test_thread_cond);
      pthread_mutex_unlock (&test_thread_mutex);
      sleep (1);
      criteria.action = ULP_REMOVE_CRITERIA;
      pUlpEngineInterface->update_criteria(criteria);
      rc = pGpsInterface->stop();
      sleep (3);
     }
     break;
     // Run test scenario 6
     // GPS single shot session when GPS is ensabled
     // later on, high accuracy tracking (5 second interval) session come in
     // later on, single shot fix request removed (potentially due to position has been satisfied)
     // later on, GPS session (1hz) come in
     // later on, 1hz tracking criteria removed
     // later on, 5hz tracking criteria removed
     // later on, stop request received
     //
     // ulp_brain_choose_providers:|position mode|GPS start| GPS engine
    case 6:
     {
      // set up global phone context
      is_gps_enabled                = TRUE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;
      is_agps_enabled               = TRUE;

      // Single shot fix request comes in
      UlpLocationCriteria criteria;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_GNSS;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);

      // ULP start received
      rc = pGpsInterface->start();
      sleep (4);

      // Periodic fix request comes in (5 seconds interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      criteria.min_interval = 5;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (2);

      // Single shot request gets removed (due to receive position report)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_GNSS;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      pUlpEngineInterface->update_criteria(criteria);


      // Periodic fix request comes in (1 second interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      criteria.min_interval = 1;
      pUlpEngineInterface->update_criteria(criteria);

      sleep (5);
      // Periodic fix request removed (1 second interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      criteria.min_interval = 1;
      pUlpEngineInterface->update_criteria(criteria);

      sleep (3);
      // Periodic fix request removed (5 second interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      criteria.min_interval = 5;
      pUlpEngineInterface->update_criteria(criteria);

      // Stop ulp
      rc = pGpsInterface->stop();
      sleep (3);
     }
     break;
     // Run test scenario 7
     // Network provider is enabled
     // First comes network single shot
     // Secondly comes network tracking session (30 seconds)
     // Secondly comes network tracking session (10 seconds)
     // Single shot removed
     // 30 seconds network tracking session removed
     // 10 seconds network tracking session removed
     // ulp stop received
     //
     // Search "ulp_brain_choose_providers:|request type: " for output
    case 7:
     {

      is_gps_enabled                = TRUE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;

      UlpLocationCriteria criteria;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_HYBRID;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);
      rc = pGpsInterface->start();
      sleep (4);

      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_LOW;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 30;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (4);
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_HYBRID;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (4);
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_LOW;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 30;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (4);
      // Stop ulp
      rc = pGpsInterface->stop();
      sleep (4);
     }
     break;
     // Run test scenario 8
     // GPS and Network provider is enabled
     // First comes network single shot
     // Secondly comes network tracking session (30 seconds)
     // Then comes GPS tracking session (10 seconds)
     // Single shot network is removed
     // GPS tracking session (10 seconds) is removed
     // Network tracking session (30 seconds) is removed
     // ulp stop received
     // Search log for "ulp_brain_choose_providers:|position mode|GPS engine|request type: "
    case 8:
     {
      // set up global phone context
      is_gps_enabled                = TRUE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;

      UlpLocationCriteria criteria;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_HYBRID;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);
      rc = pGpsInterface->start();
      sleep (4);
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_LOW;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 30;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (4);
      // Periodic GPS fix request comes in (10 seconds interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      criteria.min_interval = 10;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (2);

      // Single shot network session removed (request satisified)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_HYBRID;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
      pUlpEngineInterface->update_criteria(criteria);

      sleep (4);

      // Periodic GPS fix request goes away (10 seconds interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      criteria.min_interval = 10;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (2);

      // Periodic network tracking session goes away (30 seconds interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_LOW;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 30;
      pUlpEngineInterface->update_criteria(criteria);

      sleep (4);

      // Stop ulp
      rc = pGpsInterface->stop();
      sleep (4);
     }
     break;
     // Run test scenario 9
     // GPS and Network provider is enabled
     // First comes network single shot
     // Secondly comes network tracking session (30 seconds)
     // Then comes GPS tracking session (10 seconds)
     // Single shot network is removed
     // GPS disabled
     // GPS tracking session (10 seconds) is removed
     // Network tracking session (30 seconds) is removed
     // ulp stop received
     //
     // Search log for "ulp_brain_choose_providers:|position mode|GPS engine|request type: "
    case 9:
     {
      // set up global phone context
      is_gps_enabled                = TRUE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;

      UlpLocationCriteria criteria;
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_HYBRID;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (1);
      rc = pGpsInterface->start();
      sleep (4);
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_LOW;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 30;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (4);
      // Periodic GPS fix request comes in (10 seconds interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_ADD_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      criteria.min_interval = 10;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (2);

      // Single shot network session removed (request satisified)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
      criteria.provider_source = ULP_PROVIDER_SOURCE_HYBRID;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (4);
      // set up global phone context
      is_gps_enabled                = FALSE;
      is_network_position_available = TRUE;
      is_wifi_setting_enabled       = TRUE;
      is_battery_charging           = TRUE;
      //Simulate phone context update from AFW
      pthread_mutex_lock (&test_thread_mutex);
      test_thread_action = ACTION_PHONE_CONTEXT_UPDATE;
      pthread_cond_signal (&test_thread_cond);
      pthread_mutex_unlock (&test_thread_mutex);
      sleep (2);

      // Periodic GPS fix request goes away (10 seconds interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      criteria.min_interval = 10;
      pUlpEngineInterface->update_criteria(criteria);
      sleep (2);

      // Periodic network tracking session goes away (30 seconds interval)
      criteria.valid_mask = 0;
      criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
      criteria.action = ULP_REMOVE_CRITERIA;
      criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
      criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_LOW;
      criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
      criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
      criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
      // assume in seconds
      criteria.min_interval = 30;
      pUlpEngineInterface->update_criteria(criteria);

      sleep (4);

      // Stop ulp
      rc = pGpsInterface->stop();
      sleep (4);
     }
     break;
    // Run test scenario 10
    // GPS single shot when GPS is enabled, GPS_CAPABLITY support MSA
    // Remove single shot
    // Output: single shot will be set to tracking with long interval, ,MSA
    case 10:
    {
       // set up global phone context
       is_gps_enabled                = TRUE;
       is_network_position_available = TRUE;
       is_wifi_setting_enabled       = TRUE;
       is_battery_charging           = TRUE;

       UlpLocationCriteria criteria;
       criteria.valid_mask = 0;
       criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
       criteria.action = ULP_ADD_CRITERIA;
       criteria.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
       criteria.provider_source = ULP_PROVIDER_SOURCE_GNSS;
       criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
       criteria.recurrence_type = ULP_LOC_RECURRENCE_SINGLE;
       pUlpEngineInterface->update_criteria(criteria);
       sleep (1);
       rc = pGpsInterface->start();
       sleep (5);
       criteria.action = ULP_REMOVE_CRITERIA;
       pUlpEngineInterface->update_criteria(criteria);
       rc = pGpsInterface->stop();
       sleep (3);
    }
    break;
    // Run test scenario 11
    // high accuracy tracking (30 second interval) session come in
    // 10 seconds later, another high accuracy tracking (30 second interval) session come in
    // later on, one 30 second tracking criteria removed
    // later on, the other 30 second tracking criteria removed
    // later on, stop request received
    //
    // ulp_brain_choose_providers:|position mode|GPS start| GPS engine
   case 11:
       {
         // set up global phone context
         is_gps_enabled                = TRUE;
         is_network_position_available = TRUE;
         is_wifi_setting_enabled       = TRUE;
         is_battery_charging           = TRUE;

         UlpLocationCriteria criteria;

         // Periodic GPS fix request comes in (30 seconds interval)
         criteria.valid_mask = 0;
         criteria.valid_mask |= ULP_CRITERIA_HAS_ACTION;
         criteria.action = ULP_ADD_CRITERIA;
         criteria.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
         criteria.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_HIGH;
         criteria.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
         criteria.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
         criteria.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
         criteria.min_interval = 30;
         pUlpEngineInterface->update_criteria(criteria);
         rc = pGpsInterface->start();
         sleep (10);
         pUlpEngineInterface->update_criteria(criteria);
         sleep (10);

         // Remove first tracking session
         criteria.action = ULP_REMOVE_CRITERIA;
         pUlpEngineInterface->update_criteria(criteria);
         sleep (2);
         // Remove second tracking session
         pUlpEngineInterface->update_criteria(criteria);
         sleep (2);
         rc = pGpsInterface->stop();
         sleep (3);
       }
       break;
   case 12:
       {
       // Run test scenario 12
       // First, hybrid provider tracking (30 second interval) session come in
       // 10 seconds later, another low accuracy (30 second interval) session come in
       // later on, the 30 second hybrid provider tracking criteria removed
       // later on, the other 30 second low accuracy tracking criteria removed
       // later on, stop request received
       //
       // Search log for "ulp_brain_choose_providers:|position mode|GPS engine|request type: "
       // set up global phone context
       is_gps_enabled                = TRUE;
       is_network_position_available = TRUE;
       is_wifi_setting_enabled       = TRUE;
       is_battery_charging           = TRUE;

       UlpLocationCriteria criteria1;

       // Periodic GPS fix request comes in (30 seconds interval)
       criteria1.valid_mask = 0;
       criteria1.valid_mask |= ULP_CRITERIA_HAS_ACTION;
       criteria1.action = ULP_ADD_CRITERIA;
       criteria1.valid_mask |= ULP_CRITERIA_HAS_PROVIDER_SOURCE;
       criteria1.provider_source = ULP_PROVIDER_SOURCE_HYBRID;
       criteria1.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
       criteria1.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
       criteria1.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
       criteria1.min_interval = 30;
       pUlpEngineInterface->update_criteria(criteria1);
       rc = pGpsInterface->start();
       sleep (10);
       UlpLocationCriteria criteria2;

       // Periodic GPS fix request comes in (30 seconds interval)
       criteria2.valid_mask = 0;
       criteria2.valid_mask |= ULP_CRITERIA_HAS_ACTION;
       criteria2.action = ULP_ADD_CRITERIA;
       criteria2.valid_mask |= ULP_CRITERIA_HAS_PREFERRED_HORIZONTAL_ACCURACY;
       criteria2.preferred_horizontal_accuracy = ULP_HORZ_ACCURACY_LOW;
       criteria2.valid_mask |= ULP_CRITERIA_HAS_RECURRENCE_TYPE;
       criteria2.recurrence_type = ULP_LOC_RECURRENCE_PERIODIC;
       criteria2.valid_mask |= ULP_CRITERIA_HAS_MIN_INTERVAL;
       criteria2.min_interval = 30;
       pUlpEngineInterface->update_criteria(criteria2);
       sleep (10);

       // Remove first tracking session
       criteria1.action = ULP_REMOVE_CRITERIA;
       pUlpEngineInterface->update_criteria(criteria1);
       sleep (2);

       // Remove second tracking session
       criteria2.action = ULP_REMOVE_CRITERIA;
       pUlpEngineInterface->update_criteria(criteria2);
       sleep (2);
       pGpsInterface->stop();
       sleep (3);
       }
       break;
   default:
       garden_print(" Unknown ulp test case\n");
   }
}
#endif /* TEST_ULP */

void test_zpp(int zpptestCase)
{
   garden_print ("Starting ZPP test cases\n");
   int rc = 0;

   if (NULL == pUlpEngineInterface) {
      garden_print ("In test_zpp. Error.pUlpEngineInterface null\n");
      return;
   }

   switch (zpptestCase)
   {
    // Run test scenario 1
  // System event screen on
   case 1:
     {
       pthread_mutex_lock (&ulp_location_mutex);
       pUlpEngineInterface->system_update(ULP_LOC_SCREEN_ON);
       pthread_cond_wait (&ulp_location_cond, &ulp_location_mutex);
       pthread_mutex_unlock (&ulp_location_mutex);
       garden_print ("ZPP TestCase: System event screen on passed\n");

     }
    break;
    // System event time zone change
    case 2:
     {
         pthread_mutex_lock (&ulp_location_mutex);
         pUlpEngineInterface->system_update(ULP_LOC_TIMEZONE_CHANGE);
         pthread_cond_wait (&ulp_location_cond, &ulp_location_mutex);
         pthread_mutex_unlock (&ulp_location_mutex);
         garden_print ("ZPP TestCase:System event time zone change passed\n");
     }
    break;
   // Power connected 30s then power disconnected
   case 3:
    {
        pthread_mutex_lock (&ulp_location_mutex);
        pUlpEngineInterface->system_update(ULP_LOC_POWER_CONNECTED);
        pthread_cond_wait (&ulp_location_cond, &ulp_location_mutex);
        garden_print ("ZPP TestCase:System event Power connected. got 1st loc report");
        pthread_cond_wait (&ulp_location_cond, &ulp_location_mutex);
        pthread_mutex_unlock (&ulp_location_mutex);
        pUlpEngineInterface->system_update(ULP_LOC_POWER_DISCONNECTED);
        garden_print ("ZPP TestCase:System event Power connected passed\n");
    }
   break;

   // verify ZPP throtting thru threshold in gps.conf
   case 4:
    {
        struct timeval present_time;
        struct timespec expire_time;
        pthread_mutex_lock (&ulp_location_mutex);
        pUlpEngineInterface->system_update(ULP_LOC_SCREEN_ON);
        sleep(1);
        pUlpEngineInterface->system_update(ULP_LOC_TIMEZONE_CHANGE);
        /* Calculate absolute expire time */
        gettimeofday(&present_time, NULL);
        expire_time.tv_sec  = present_time.tv_sec + 10;
        expire_time.tv_nsec = present_time.tv_usec * 1000;
        pthread_cond_timedwait (&ulp_location_cond, &ulp_location_mutex, &expire_time);
        pthread_mutex_unlock (&ulp_location_mutex);
        pUlpEngineInterface->system_update(ULP_LOC_POWER_DISCONNECTED);
        garden_print ("ZPP TestCase:verify ZPP throtting passed\n");
    }
   break;

    default:
       garden_print(" Unknown zpp test case\n");
   }
}

void run_tracking_session(void)
{
   int  pipefd[2] = {0,0};
   char buf;
   int rc;

   if (pipe(pipefd) == -1) {
      LOC_LOGE("pipe error");
      return;
   }

   if (NULL != pGpsInterface)
   {
      gGardenSessionControl = new garden_app_session_tracker(pGpsInterface);

      if(NULL == gGardenSessionControl)
      {
         LOC_LOGE("%s] Failed to Create garden_tracking_session_control. \n", __func__);
         return;
      }
/*Create power event handler object for LE variants */
#ifdef USE_GLIB
      PowerEvtHandler powerEvtObj;
#endif
      gEventObserver = new EventObserver(rxSystemEvent);

      if (NULL == gEventObserver)
      {
         LOC_LOGE("%s] Failed to Create EventObserver. \n", __func__);
         return;
      }
#ifdef USE_GLIB
      set_phone_context();
#endif
   }

   // set up global phone context
   is_gps_enabled                = TRUE;
   is_network_position_available = TRUE;
   is_wifi_setting_enabled       = TRUE;
   is_battery_charging           = TRUE;

   //run the tracking session until the process stops
   while(g_run_active_client)
   {
      read(pipefd[0], &buf, 1);
   }

}


