/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <limits.h>

#include <izat_remote_api.h>
#include <gps_extended_c.h>
#include <loc_cfg.h>
#include <iostream>
#include <platform_lib_macros.h>

using namespace std;
int id1 = 1;
int tty_fd;
#define SERIAL_PORT "/dev/ttyHSL0"
#define NMEA_PORT "/dev/ttyGS0"

/* Raw Data subscription mask*/
#define RAW_POS_DATA_SUB_MASK           (0x001)
#define RAW_SVINFO_DATA_SUB_MASK        (0x002)
#define RAW_NEMA_DATA_SUB_MASK          (0x004)
#define RAW_DATA_SUB_MASK_ALL           (RAW_POS_DATA_SUB_MASK | \
                                         RAW_SVINFO_DATA_SUB_MASK | \
                                         RAW_NEMA_DATA_SUB_MASK)

/* Final Data subscription mask*/
#define FINAL_POS_DATA_SUB_MASK           (0x001)
#define FINAL_SVINFO_DATA_SUB_MASK        (0x002)
#define FINAL_NMEA_DATA_SUB_MASK          (0x004)
#define FINAL_DATA_SUB_MASK_ALL           (FINAL_POS_DATA_SUB_MASK | \
                                           FINAL_SVINFO_DATA_SUB_MASK | \
                                           FINAL_NEMA_DATA_SUB_MASK)


/** Structure that holds the command line options given to main **/
typedef struct command_line_options {
    int t;                        // timeout in seconds
    int w;                        // wait after time out before exit in seconds
    int rawSubMask;               // Raw data subscription mask.
    int finalSubMask;             // Final data subscription mask.
    int printLogs;                // Print nmea string
} CommandLineOptionsType;

// Default values
static CommandLineOptionsType sOptions = {
    100,                         // timeout in seconds.
    2,                           // wait after time out before exit in seconds.
    RAW_DATA_SUB_MASK_ALL,       // Raw data subscription mask..
    0,                           // Final data subscription mask
    1                            // Print logs to sonsole
};

/*
* 'open_port()' - Open NMEA port.
*/
int open_port(void)
{
    int n; /* File descriptor for the port */

    printf("open_port \n");
    tty_fd = open(NMEA_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (tty_fd == -1)
    {
        /* Could not open the port. */
        printf("open_port: Unable to open %s \n", NMEA_PORT);
    }
    printf ("open_port success \n");
}

static void print_and_sleep(int seconds) {
    printf("sleep for %d seconds... \n", seconds);
    sleep(seconds);
}

// SSTP Test
static void sstpErrorReportCallback(const char* errorString, void* clientData) {
    printf("%s clientData - %p errorString - %s \n", __func__, clientData, errorString);
}

static void sstpSiteUpdateCallback(const char* siteName, double lat, double lon,
                                   float unc, int32_t uncConf, void* clientData) {
    printf("%s clientData - %p siteName - %s latitude - %f longitude - %f"
           " uncertainty - %f uncertainty confidence - %d\n",
           __func__, clientData, siteName, lat, lon, unc, uncConf);
}

static void sstpMccUpdateCallback(uint32_t mcc, const char* confidence,
                                  void* clientData) {
    printf("%s clientData - %p mcc - %d confidence - %s \n", __func__,
           clientData, mcc, confidence);
}

void sstp_help(char* cmd) {
    printf("%s  -s <seconds to wait after listener registered> \n"
           " <seconds to wait after calling stopSstpUpdate> \n"
           " <seconds to wait after listener unregistered> \n"
           " Test sstpUpdate \n", cmd);

}

int sstp_main(int argc, char** argv) {
    if (argc < 5) {
        printf("Usage: \n");
        sstp_help(argv[0]);
    } else {
        printf("Testing izat_remote_api sstpUpdate \n");

        printf("registerSstpUpdater \n");
        void* handle = registerSstpUpdater(sstpSiteUpdateCallback,
                                           sstpMccUpdateCallback,
                                           sstpErrorReportCallback,
                                           (void*)0xDEADBEEF);

        print_and_sleep(atoi(argv[2]));

        printf("stopSstpUdate...\n");
        stopSstpUpdate(handle);

        print_and_sleep(atoi(argv[3]));

        printf("unregisterLocationUpdater...\n");
        unregisterSstpUpdater(handle);

        print_and_sleep(atoi(argv[4]));

        printf("exiting...\n");
    }

    return 0;
}

// raw nmea updater Test
static void rawNmeaUpdateCallback(UlpNmea *nmea, void* clientData) {

    int n;
    if (sOptions.printLogs)
        printf("RAW NMEA: %s \n", nmea->nmea_str);
    n = write(tty_fd, nmea->nmea_str, strlen(nmea->nmea_str) + 1);
    if (n < 0)
        printf("write() of %d bytes failed!\n", n);

}

// nmea updater Test
static void nmeaUpdateCallback(UlpNmea *nmea, void* clientData) {

    int n;
    if (sOptions.printLogs)
        printf("NMEA: %s \n", nmea->nmea_str);
    n = write(tty_fd, nmea->nmea_str, strlen(nmea->nmea_str) + 1);
    if (n < 0)
        printf("write() of %d bytes failed!\n", n);
}
static void locationUpdateCallback(UlpLocation *location,
                             GpsLocationExtended *locExtended,
                             void* clientData) {
    if (sOptions.printLogs) {
        printf("%s clientData - %p\n"
        "| position_source - 0x%x \n"
        "| LocGpsLocation.flags - 0x%x \n"
        "|\t lat - %f \n"
        "|\t lon - %f \n"
        "|\t accuracy - %f \n"
        "|\t altitude - %f \n"
        "|\t speed - %f \n"
        "|\t timestamp- %lld \n"
        ,__func__ , clientData,
        location->position_source,
        location->gpsLocation.flags,
        location->gpsLocation.latitude,
        location->gpsLocation.longitude,
        location->gpsLocation.accuracy,
        location->gpsLocation.altitude,
        location->gpsLocation.speed,
        location->gpsLocation.timestamp
        );
    }
}

static void rawLocationUpdateCallback(UlpLocation *location,
                                 GpsLocationExtended *locExtended,
                                 void* clientData) {
    if (sOptions.printLogs) {
        printf("%s clientData - %p\n"
        "| position_source - 0x%x \n"
        "| LocGpsLocation.flags - 0x%x \n"
        "|\t lat - %f \n"
        "|\t lon - %f \n"
        "|\t accuracy - %f \n"
        "|\t altitude - %f \n"
        "|\t speed - %f \n"
        "|\t timestamp- %lld \n"
        ,__func__ , clientData,
        location->position_source,
        location->gpsLocation.flags,
        location->gpsLocation.latitude,
        location->gpsLocation.longitude,
        location->gpsLocation.accuracy,
        location->gpsLocation.altitude,
        location->gpsLocation.speed,
        location->gpsLocation.timestamp
        );
    }
}

void svInfoUpdateCallback(LocGnssSvStatus *svStatus,
                                 GpsLocationExtended *locExtended,
                                 void* clientData)
{
    if (sOptions.printLogs) {
        printf("%s clientData - %p\n"
        "| GpsLocationExtended.flags - 0x%x \n"
        "| num_svs - %d \n"
        "|\t altitudeMeanSeaLevel - %f \n"
        "|\t pdop - %f \n"
        "|\t hdop - %f \n"
        "|\t vdop - %f \n"
        "|\t horUncEllipseSemiMajor - %f \n"
        "|\t horUncEllipseSemiMinor - %f \n"
        ,__func__ , clientData,
        locExtended->flags,
        svStatus->num_svs,
        locExtended->altitudeMeanSeaLevel,
        locExtended->pdop,
        locExtended->hdop,
        locExtended->vdop,
        locExtended->horUncEllipseSemiMajor,
        locExtended->horUncEllipseSemiMinor
        );
    }
}

void rawSvInfoUpdateCallback(LocGnssSvStatus *svStatus,
                                 GpsLocationExtended *locExtended,
                                 void* clientData)
{
    if (sOptions.printLogs) {
        printf("%s clientData - %p\n"
        "| GpsLocationExtended.flags - 0x%x \n"
        "| num_svs - %d \n"
        "|\t altitudeMeanSeaLevel - %f \n"
        "|\t pdop - %f \n"
        "|\t hdop - %f \n"
        "|\t vdop - %f \n"
        "|\t horUncEllipseSemiMajor - %f \n"
        "|\t horUncEllipseSemiMinor - %f \n"
        ,__func__ , clientData,
        locExtended->flags,
        svStatus->num_svs,
        locExtended->altitudeMeanSeaLevel,
        locExtended->pdop,
        locExtended->hdop,
        locExtended->vdop,
        locExtended->horUncEllipseSemiMajor,
        locExtended->horUncEllipseSemiMinor
        );
    }
}

void printHelp(char **arg)
{
    printf("usage: %s -l -t <time out in sec> -G <GNSS data sub mask> -D <DR data sub mask>\n", arg[0]);
    printf("    -t:  seconds to wait after listener registered: %d\n", sOptions.t);
    printf("    -w:  seconds to wait after listener unregistered: %d\n", sOptions.w);
    printf("    -G:  GNSS data type subscription mask\n"
           "         GNSS_POS_DATA      (0x001) Enable GNSS Position data indication\n"
           "         GNSS_SVINFO_DATA   (0x002) Enable GNSS SV Info data indication\n"
           "         GNSS_NMEA_DATA     (0x003) Enable GNSS NMEA data indication \n"
           "             Note:     Should subscribe for Position and SV report indication\n");
    printf("    -D:  DR data type subscription mask\n"
           "         DR_POS_DATA        (0x001) Enable DR Position data indication\n"
           "         DR_SVINFO_DATA     (0x002) Enable DR SV Info data indication\n"
           "         DR_NMEA_DATA       (0x003) Enable DR NMEA data indication\n"
           "             Note:     Should subscribe for Position and SV report indication\n");


    printf("    -p:  Print Log strings, Defaults: %d\n", sOptions.printLogs);
    printf("    -h:  print this help\n");
}

void location_help(char* cmd) {
    printf(" |\t %s -l <seconds to wait after listener registered> \n"
           " <seconds to wait after listener unregistered> \n"
           "|\t\t Test locationUpdate\n", cmd);
}

int location_main(int argc, char** argv) {
    if (argc < 4) {
        printf("Usage: \n");
        printHelp(argv);
    } else {
        int opt;
        extern char *optarg;
        char *endptr;
        void *handle1 = NULL, *handle2 = NULL;

        while ((opt = getopt (argc, argv, "t:w:G:D:p:z")) != -1) {
            switch (opt) {
                case 't':
                    sOptions.t = atoi(optarg);
                    printf("Timeout after registrantion: %d \n", sOptions.t);
                    break;
                case 'w':
                    sOptions.w = atoi(optarg);
                    printf("Timeout after unregistrantion: %d \n", sOptions.w);
                    break;
                case 'G':
                    sOptions.rawSubMask = strtol(optarg, NULL, 16);
                    printf("GNSS Data Sub Types : 0x%x \n", sOptions.rawSubMask);
                    break;
                case 'D':
                    sOptions.finalSubMask = strtol(optarg, NULL, 16);
                    printf("QDR Data Sub Types : 0x%x \n", sOptions.finalSubMask);
                    break;
                case 'p':
                    sOptions.printLogs = atoi(optarg);
                    printf("Print NMEA strings : %d \n", sOptions.printLogs);
                    break;
                case 'h':
                default:
                    printHelp(argv);
                    return 0;
            }
        }

        open_port();

        printf("Testing izat_remote_api locationUpdate \n");
        remoteClientInfo cbData;
        remoteClientInfo rawCbData;
        memset(&cbData, 0 , sizeof(remoteClientInfo));
        memset(&rawCbData, 0 , sizeof(remoteClientInfo));

        if (sOptions.finalSubMask & FINAL_POS_DATA_SUB_MASK){
            printf("registering for locationUpdateCallback \n");
            cbData.locCb = locationUpdateCallback;
        }
        if (sOptions.finalSubMask & FINAL_SVINFO_DATA_SUB_MASK){
            printf("registering for svInfoUpdateCallback\n");
            cbData.svReportCb = svInfoUpdateCallback;
        }
        if (sOptions.finalSubMask & FINAL_NMEA_DATA_SUB_MASK){
            printf("registering for nmeaUpdateCallback\n");
            cbData.nmeaCb = nmeaUpdateCallback;
        }

        if (sOptions.rawSubMask & RAW_POS_DATA_SUB_MASK){
            printf("registering for rawLocationUpdateCallback\n");
            rawCbData.locCb = rawLocationUpdateCallback;
        }
        if (sOptions.rawSubMask & RAW_SVINFO_DATA_SUB_MASK){
            printf("registering for rawSvInfoUpdateCallback \n");
            rawCbData.svReportCb = rawSvInfoUpdateCallback;
        }
        if (sOptions.rawSubMask & RAW_NEMA_DATA_SUB_MASK){
            printf("registering for rawNmeaUpdateCallback \n");
            rawCbData.nmeaCb = rawNmeaUpdateCallback;
        }

        if (sOptions.finalSubMask){
            printf("calling registerLocationUpdater \n");
            handle1 = registerLocationUpdater(&cbData, (void*)0xDEADBEEF);
        }
        if (sOptions.rawSubMask){
            printf("calling registerRawLocationUpdater \n");
            handle2 = registerRawLocationUpdater(&rawCbData, (void*)0xDEADBEEE);
        }
        print_and_sleep(sOptions.t);
        printf("unregisterLocationUpdater \n");
        if(handle1)
            unregisterLocationUpdater(handle1);
        if(handle2)
            unregisterRawLocationUpdater(handle2);

        print_and_sleep(sOptions.w);

        printf("exiting...\n");
    }

    return 0;
}

void printHelp(char* cmd) {
    printf("Usage: \n");
    location_help(cmd);
    sstp_help(cmd);
}

// main
int main(int argc, char** argv) {
    UTIL_READ_CONF_DEFAULT(LOC_PATH_GPS_CONF);

    if (argc < 3) {
        printHelp(argv[0]);
    } else {
        int opt;
        while ((opt = getopt (argc, argv, "l:s:")) != -1) {
            switch (opt) {
            case 'l':
                location_main(argc, argv);
                break;
            case 's':
                sstp_main(argc, argv);
                break;
            default:
                break;
            }
        }
    }
    exit(0);
}
