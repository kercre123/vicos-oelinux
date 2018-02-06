/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2014, 2016 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                  qdr core i/f header file

GENERAL DESCRIPTION
  This file contains the data structure, functions and variables used for
  QDR core interface API's

NOTE
  All the function calls (API's) described in this file are non-blocking calls,
  the function should return as soon as the intended activity is completed.

=============================================================================*/
#ifndef __QDR_IF__
#define __QDR_IF__
#ifdef __cplusplus
extern "C" {
#endif
#ifdef OFFLINE
#include <stdint.h>
#endif

typedef enum
{
   QDR_FAILURE=0,
   QDR_SUCCESS,
   QDR_INVALID_SENSOR,   /** if the sensor data is invalid or of unsupported types*/
   QDR_NL_UNINITIALISED
}qdr_return;

typedef enum
{
   QDR_THR_SYNC_INVALID=0,
   QDR_THR_SYNC_OBJECT1,
   QDR_THR_SYNC_MAX_OBJECTS,
}qdr_threadsync_object;

typedef enum
{
   QDR_INITIALIZED=0,
   QDR_UNINITIALIZED
}qdr_navstate;

typedef struct _tQdrSVState
{
   double  GPSTime;      /* GPS time */
   double  Pos[3];       /* 3D positions in ECEF */
   double  Vel[3];       /* 3D velocities in ECEF */
   float   Acc[3];       /* 3D accelerations in ECEF */
   double  ClkBias;      /* clock bias */
   float   ClkDrift;     /* clock drift */
   float   PosVar;       /* position variance in meter^2 */
   float   ClkVar;       /* clock bias variance in meter^2 */
   float   Iono;         /* ionosphere correction */
   float   ionoDelayStd; /* meters*/
   long    tow;          /* GPS time referenced from start of the week. */
                         /* [sec, 0 - 604800], toe or toa               */
   short   wno;          /* The GPS week number, Eph or Alm weekno  */
                         /* [weeks, 0 - life time of GPS in weeks]  */
   unsigned char EphFlag; /* ephemeris flag indicator (0: almanac 1: ephemeris, otherwise: unavailable) */
   unsigned char ephAge;  /* in days */
   unsigned char Updated; /* updated or not */
   unsigned char Svid;    /* sv id */
} tQdrSVState;

/**These prototypes are derived from
* OSP Message DGPS types code defines
* SIRF_MSG_SSB_DGPS_SRC_INTERNAL,SIRF_MSG_SSB_DGPS_SRC_NONE
* SIRF_MSG_SSB_DGPS_SRC_SBAS, SIRF_MSG_SSB_DGPS_SRC_SOFTWARE,
* SIRF_MSG_SSB_DGPS_SRC_USER
*/
#define DGPS_SRC_INTERNAL                                  ( 3 )
#define DGPS_SRC_NONE                                      ( 0 )
#define DGPS_SRC_SBAS                                      ( 1 )
#define DGPS_SRC_SOFTWARE                                  ( 4 )
#define DGPS_SRC_USER                                      ( 2 )

typedef enum _qdrCorrectionTypes
{
   QDR_COR_NONE            = DGPS_SRC_NONE,  /* Use no corrections       */
   QDR_COR_SBAS            = DGPS_SRC_SBAS,  /* Use SBAS channel         */
   QDR_COR_SERIAL          = DGPS_SRC_USER,  /* Use external receiver    */
   QDR_COR_INTERNAL_BEACON = DGPS_SRC_INTERNAL,  /* Use internal DGPS Beacon */
   QDR_COR_SOFTWARE        = DGPS_SRC_SOFTWARE   /* MI_SetDGPSCorrections    */
}qdrCorrectionTypes;

typedef struct  _tQdrDGPSMeasStruct/* Structure definition for  NAV Library DGPS Interface */
{
   short SVID;
   short IOD;
   qdrCorrectionTypes Source;  /* This is already defined in mi_if.h.
                                    We are using the same types here. */
   float PRCorrection;   /* m   */
   float PRRCorrection;  /* m/s */
   float CorrectionAge;  /* sec */
   float PRCVariance;    /* m;   Only for SBAS? Or from some other mechanism?)*/
   float PRRCVariance;   /* m/s; Only for SBAS? Or from some other mechanism?)*/
} tQdrDGPSMeasStruct;

typedef enum _qdr_timeSourcet
{
   QDR_TIME_NOT_SET=0,
   QDR_TIME_SET_GPS,
   QDR_TIME_SET_GLO,
   QDR_TIME_SET_GAL,
   QDR_TIME_SET_BDS,
   QDR_TIME_SET_EXT
}qdr_timeSourcet;

typedef uint8_t qdr_timevalid;
#define QDR_SYS_TIME_INVALID  0
#define QDR_SYS_TIME_VALID    0x1
#define QDR_LS_VALID          0x2
#define QDR_GGTAU_VALID       0x4

typedef struct _qdr_sysTime
{
   short int week;
   unsigned int swttag;
   double swtow; /**In Second*/
   float swtEstAcc;
   uint8_t leapSecond;
   qdr_timevalid time_valid;
   double glo2gpsTau;             /**tauGPS in sec */
   qdr_timeSourcet src;
   uint64_t pps_timeStamp;        /**1PPS timestamp in us**/
   float pps_unc;                 /**PPS unc unknown if it is 10ms*/
}qdr_sysTime;

typedef struct
{
   unsigned long  Timetag;
   unsigned long  TimetagLsw;
   double         Pseudorange;
   double         DeltaRange;
   double         sinel;
   int            az;
   double         measTow;
   double         CarrierPhase;
   float          DeltaCP;
   unsigned short TimeInTrack;
   unsigned short DRInterval;   /* delta-range interval */
   unsigned short DRTime;       /* mean time of the delta-range interval */
   short          ExtrapolationTime;
   unsigned short measWeek;
   unsigned char  SVID;
   unsigned char  SyncFlags;
   unsigned char  LowPowerCount;
   unsigned char  PhaseErrorCount;
   unsigned char  CtoN[10];
} tQdrGnssMeas2NAV;

typedef struct _tQdrNavigationData
{
   unsigned int    solution_validity;
   unsigned int    solution_info;
   unsigned short  gps_week;
   unsigned int    tow;
   unsigned int    tow_sub_ms;
   short           time_bias;
   unsigned char   time_accuracy;
   unsigned char   time_source;
   unsigned short  utc_year;
   unsigned char   utc_month;
   unsigned char   utc_day;
   unsigned char   utc_hour;
   unsigned char   utc_min;
   unsigned short  utc_sec;
   unsigned char   utc_offset;
   unsigned char   datum;
   double          clk_bias;
   unsigned int    clk_bias_error;
   int             clk_offset;
   unsigned int    clk_offset_error;
   int             lat;
   int             lon;
   int             alt_ellips;
   int             alt_msl;
   unsigned short  sog;
   unsigned short  cog;
   short           climb_rate;
   short           heading_rate;
   unsigned int    distance_travel;
   unsigned short  heading_error;
   unsigned short  distance_travel_error;
   unsigned int    ehpe;
   unsigned int    evpe;
   unsigned short  ehve;
   unsigned char   gdop;
   unsigned char   pdop;
   unsigned char   hdop;
   unsigned char   vdop;
   unsigned char   tdop;
   unsigned char   num_svs_in_sol;
   unsigned int    sv_list_1;
   unsigned int    sv_list_2;
   unsigned int    sv_list_3;
   unsigned int    sv_list_4;
   unsigned int    sv_list_5;
   unsigned int    additional_info;
} tQdrNavigationData;

/*Core supports 32 SV's to support multiconstellation operations */
#define QDR_CORE_SAT_DATA_NUM_OF_SATS ( 32 )

/* Body frame dynamics parameters updated by the core */
typedef struct _tQdrNavBodyFrameParams{
   /* Forward Acceleration in body frame (m/s2)*/
   float           longAccel;
   /* Sideward Acceleration in body frame (m/s2)*/
   float           latAccel;
   /* Vertical Acceleration in body frame (m/s2)*/
   float           vertAccel;
   /* Heading Rate (Radians/second) */
   float           yawRate;
   /* Body pitch   (Radians/second) */
   float           pitch;
   /* Data Validity */
   uint32_t        dataMask;
} tQdrNavBodyFrameParams;

/* For Body frame data updates */
typedef struct _tQdrCoreNavBfdParams{
    unsigned int towLatch;
    tQdrNavBodyFrameParams bfdData;
} tQdrCoreNavBfdParams;

typedef struct _tQdrSvStateDataPerSV {
    unsigned short         sat_info;
    unsigned short         azimuth;
    unsigned short         elevation;
    unsigned int           status;
    unsigned short         avg_cno;
} tQdrSvStateDataPerSV;

typedef struct _tQdrSvStateData {
    unsigned short         gps_week;
    unsigned int           tow;
    unsigned int           tow_sub_ms;
    short                  time_bias;
    unsigned char          time_accuracy;
    unsigned char          time_source;
    unsigned char          num_of_sats;
    unsigned int           gps_svs_used_infix;
    unsigned int           glo_svs_used_infix;
    unsigned long long     gal_svs_used_infix;
    unsigned long long     bds_svs_used_infix;
    unsigned int           qzss_svs_used_infix;
    unsigned int           sbas_svs_used_infix;
    tQdrSvStateDataPerSV sat[QDR_CORE_SAT_DATA_NUM_OF_SATS];
} tQdrSvStateData;


/*Callback Types*/
typedef qdr_return (* qdr_cb_retrieveNVData_t) ( unsigned char            dataIDToRetrieve,
                                                unsigned int           dataLength,
                                                unsigned char            *pPayload);
typedef qdr_return (* qdr_cb_storeNVData_t) (   unsigned char            dataIDToStore,
                                                unsigned int           dataLength,
                                                unsigned char            *pPayload);

typedef void (* qdr_cb_sendDIAGData_t) (unsigned int diagID,unsigned int packetLength,void  *pPayload);

/**Diag ID's from QDR core, True mapping is done outside*/
#define QDR_DIAG_VERSION_INFO               0x0001
#define QDR_DIAG_GNSS_NAVINIT_DATA          0x0002 /**Structure type "tQdrNavInitData"*/
#define QDR_DIAG_A6DOF_NVM_INIT_DATA        0x0003
#define QDR_DIAG_A6DOF_NAVSTATE             0x0004 /**Structure type "tQdrA6DOFNavstate"*/
#define QDR_DIAG_A6DOF_GNSS_ODO_INFO        0x0005 /**Structure type "tQdrA6DOFGnssOdoInfo"*/
#define QDR_DIAG_A6DOF_GNSS_CHAN_INFO       0x0006 /**Structure type "tQdrA6DOFGnssChanList"*/
#define QDR2_DIAG_NAVSTATE                  0x0007 /**Structure type "MI_DR_NAV_STATE"*/
#define QDR2_DIAG_NAVSTATUS                 0x0008 /**Structure type "MI_DR_NAV_STATUS"*/
#define QDR2_DIAG_NAVSUBSYS                 0x0009 /**Structure type "MI_DR_NAV_SUBSYS"*/
#define QDR2_DIAG_NVM_INIT_DATA             0x000A /**Structure type "tDrRamData"*/


/**Nav Init Data structure Stored info from Last session*/
typedef struct _tqdrnlInit  /* Message 31 */
{
   unsigned char  EnableConAltMode;
   unsigned char  AltMode;
   unsigned char  AltSource;
   float          Altitude;
   unsigned char  DegradedMode;
   short          DegradedTimeout;
   short          DRTimeout;
   short          CoastTimeout;
   unsigned char  TrackSmoothMode;
   unsigned char  DOPSelection;
   short          HDOPThresh;
   short          GDOPThresh;
   short          PDOPThresh;
   unsigned char  DGPSSelection;
   short          DGPSTimeout;
   short          ElevNavMask;
   short          PowNavMask;
   unsigned char  EditingResidualMode;
   short          EditingResidualThreshold;
   unsigned char  SSDMode;
   short          SSDThreshold;
   unsigned char  StaticNavMode;
   short          StaticNavThreshold;
   double         PosX; /**Last stored Nav Position X*/
   double         PosY; /**Last stored Nav Position Y*/
   double         PosZ; /**Last stored Nav Position Z*/
   unsigned char  PositionInitSource;
   double         GPSTime; /**Last stored TOW*/
   short          GPSWeek; /**Last stored Week*/
   unsigned char  TimeInitSource;
   double         Drift;
   unsigned char  ClockDriftInitSource;
} tqdrnlInit;

typedef struct
{
   short             SeqNum;         /*For testing and user stuff*/
   unsigned short    OkAcrossReset;  /*TRUE:  DR stay valid after a RESET if vehicle staionary at IGN off*/
   unsigned char     SensorDataType; /* Need to remember Bus Type Across reset*/

   unsigned char     new_data; /**Reserved flag used for bookkeeping on the interface*/

   unsigned char     valid;   /* Holds whether the cal is valid and navigation is resumable */
   char              prerot;  /**Switch to control the pre-rotation of raw axes*/
   unsigned char     b2s_rp_gain; /**RP Filter Gain*/
   unsigned char     b2s_yaw_gain; /**Yaw Filter gain*/
   unsigned char     b2s_estimate; /**Estimation of B2S*/
   unsigned char     gsf_gain; /**Gyro scale factor filter Gain*/
   //unsigned char asf_gain[3];
   unsigned char     osf_gain; /**Odo scale factor filter Gain*/
   unsigned char     co_gain; /**Gain of Stationary Detect filter*/
   unsigned char     gnss_loss; /**GNSS loss used to calibrate LOS/NLOS detection*/
   /* We might want to store the time to use for uncertainty growth */

   double            llh[3]; /**Last saved Latitude,Longitude and Altitude */
   float             att[3]; /**Attitude information X,Y and Z*/
   float             acc_bias[3]; /**Accelerometer Bias X, Y and Z*/
   float             gyro_bias[3]; /**Gyroscope Bias X, Y and Z*/
   float             unc[12]; /**DR Nav filter State Uncertainities*/
   float             b2s[3]; /**B2S Roll, Pitch and Gain*/
   float             gsf; /**Gyro Scale factor*/
   float             osf; /**Odo Scale factor*/
   float             gco; /**Stationary Gyro Threshold*/
   float             aco; /**Stationary Accelerometer Threshold*/
   float             reserved[4]; /**Reserved*/
   double            tow; /**TOW*/
}  tQdrA6DOFNvmData;

typedef struct
{
   short             SeqNum;
   unsigned short    OkAcrossReset;      /* TRUE:  DR data can be used after a RESET */
                                         /* FALSE: DR data cannot be used after a RESET */
   float             DRHeading;          /* deg */
   float             DRHeadingError;     /* deg, 1-sigma */
   float             DRSpeedError;       /* m/sec, 1-sigma */
   float             DRPositionError;    /* meters, 1-sigma */
   /*
    Odometer data
    */
   float             SpeedSf;           /* dimensionless */
   float             OdoSpeedSf;        /* dimensionless */
   /*
    Gyro Data
    */
   float             HeadingRateBias;   /* deg/sec */
   float             HeadingRateSf;     /* dimensionless */
   double            HeadingRateSf_SD;  /* dimensionless */


   /* end of update */

   /*
   /* Legacy Differential Wheel Speed Data to use for existing files (These things will not be used in real time system)
   */
   float             LFSpeedSF;     /* Left Front Wheel Speed Scale Factor, dimensionless */
   float             RFSpeedSF;     /* Right Front Wheel Speed Scale Factor, dimensionless */
   float             LRSpeedSF;     /* Left Rear Wheel Speed Scale Factor, dimensionless */
   float             RRSpeedSF;     /* Right Rear Wheel Speed Scale Factor, dimensionless */
   float             AxleLength;    /* Length of rear axle, meters */
   float             AxleSep;       /* Distance from rear to front axle, meters (positive forward) */
   float             AntennaDist;   /* Distance from rear axle to GPS antenna, meters (positive forward)*/

   char              WheelSelectCtr;  /*added */

   unsigned short    FirstHRSFDone; /* Indicates First Heading Rate Scale Factor estimate was done */

   unsigned short    DiffWheelSpdCalOK; /* Indicates whether DWS calibration has been successful */

   unsigned char     LFSpeedSFCalOk; /* Indicates whether individual speed has been calibrated */
   unsigned char     RFSpeedSFCalOk; /* Indicates whether individual speed has been calibrated */
   unsigned char     LRSpeedSFCalOk; /* Indicates whether individual speed has been calibrated */
   unsigned char     RRSpeedSFCalOk; /* Indicates whether individual speed has been calibrated */

   float             HeadingRateSf_Front;
   float             HeadingRateSf_Rear;
   float             HeadingRateSf_Front_SD;
   float             HeadingRateSf_Rear_SD;

   float             RearDSF_SD;
   float             FrontDSF_SD;

   short             DrNavControl;      /* GPS Only, DR with Stored Cal, or DR with GPS Cal */



   double            RawLonAccel;
   double            RawLatAccel;
   double            RawUpAccel;

   double            YawAngle_rads ;   /* radians */
   double            YawAngleSD_rads;   /* radians */

   double            PitchAngle_rads;      /* radians */
   double            RollAngle_rads;      /* radians */

   short             Sensor2YawedDone;
   short             YawAngleComputed;

   short             UserResetWithData; /*TRUE  = User has issued Reset with Data for us to update DR with */
                                        /*FALSE = No data from user to update DR with

   /*TM 01/16/08 */
   short             ValidDrCal;
   short             OdoSpeedSFCalOk;
   unsigned char     SensorDataType; /*Need to remember Bus Type Across reset */

   /*
   /* Start-Up Altitude Checks
   */
   float             TOWLastValidKFAltitude; /* Time-Of-Week (in secs) of the last time KF computed an altitude */
   float             LastValidKFAltitude;    /* Last, good, KF-computed altitude */
   /*
   /* END OF Start-Up Altitude Checks
    */

   unsigned short    SessionCounter; /* Starts at zero and incs every hard/soft reset. Only factory clears it */
   int               CheckSum;

   double            Lat;
   double            Lon;
   double            Alt;
   unsigned char     GyroDetectionStatus;
   float             PowerOffset;   /* GPSSystemState.PowerOffset   */
   unsigned char     PowerOffsetValid;     /*GPSSystemState.PowerOffsetValid */

   float             ENE;
   float             EVE;
   char              WheelSelectCtrStrt;
   unsigned char     GState;


   unsigned char     PL_Mode;

   double            PL_Lat;
   double            PL_Lon;
   float             PL_Alt;

   float             PL_DeltaPos;
   float             PL_TotalDist;
   float             PL_TotalStrtDist;
   unsigned char     PL_NumHighSpeedEpochs;
   float             PL_ManeuverAnglePL;
   double            tow; /**TOW*/

}  tQdr2NvmData;


/**Nav Auxillary Init Data structure*/
typedef struct _tqdrnlAuxInit  /* Message 64 Sid 1 */
{
   unsigned int time_init_unc;        /* Initial time uncertainty (usec) */
   unsigned short saved_pos_week;       /* Week of saved position */
   unsigned int saved_pos_tow;        /* Time of week of saved position (sec) */
   unsigned short saved_pos_ehe;        /* EHE of saved position (100m) */
   unsigned short saved_pos_eve;        /* EVE of saved position (m) */
   unsigned char  sw_version;           /* Tracker SW version */
   unsigned char  icd_version;          /* Tracker ICD version */
   unsigned short chip_version;         /* Tracker HW Chip version */
   unsigned int acq_clk_speed;        /* Default rate of Tracker's internal clock (Hz) */
   unsigned int default_clock_offset; /* Default freq offset of Tracker's internal clock (Hz) */
   unsigned int tracker_status;       /* Tracker System status:
                                   Bit 0: Status 0=good; 1=bad
                                   Bit 1: Cache  0=Disable; 1=Enable
                                   Bit 2: RTC    0=Invalid; 1=Valid
                                   Bit 3-31: reserved */

   unsigned int reserved;

} tqdrnlAuxInit;

/**Complete structured of NVM Data retrieved*/
/**Diag ID QDR_DIAG_GNSS_NAVINIT_DATA*/
typedef struct _tQdrNavInitData  /* Message 31 and 64,1 */
{
   tqdrnlInit nl_init;
   tqdrnlAuxInit nl_auxinit;
} tQdrNavInitData;

/* QDR Core API Diag Structure for 6DOF Nav State and mirror of MI_DR_A6DOF_NAV_STATE Data for OSP message 48,20*/
/**Diag ID QDR_DIAG_A6DOF_NAVSTATE*/
typedef struct _tQdrA6DOFNavstate {
   double              tow;
   unsigned char       a6dof_state;
   unsigned char       calibration_percent;
   unsigned char       OkAcrossReset;
   unsigned char       sensor_valid;
   char                prerot;
   char                stationary;
   char                constantVel;
   float               Yaw;
   unsigned int        yaw_status;
   unsigned char       yaw_gain;
   unsigned char       yaw_ctr;
   float               Roll;
   float               pitch;
   unsigned int        rp_status;
   unsigned char       roll_pitch_gain;
   unsigned char       rp_ctr;
   float               gyro_delta;
   unsigned int        gyro_status;
   unsigned char       gyro_gain;
   unsigned char       gsf_stm_state;
   float               gsf_stm_gnss_accum;
   float               gsf_stm_gyro_accum;
   float               gyro_bias_0;
   float               gyro_bias_1;
   float               gyro_bias_2;
   float               gyro_sf;
   float               acc_delta;
   float               acc_bias_0;
   float               acc_bias_1;
   float               acc_bias_2;
   float               acc_sf_0;
   float               acc_sf_1;
   float               acc_sf_2;
   float               ns_roll;
   float               ns_pitch;
   float               ns_yaw;
   float               Roll_e;
   float               Pitch_e;
   float               Yaw_e;
} tQdrA6DOFNavstate;

/* QDR Core API Diag Structure for GNSS odo Info MI_DR_A6DOF_GNSS_ODO for OSP message 48,21 */
/**Diag ID QDR_DIAG_A6DOF_GNSS_ODO_INFO*/
typedef struct _tQdrA6DOFGnssOdoInfo {
   double              tow;
   unsigned short      gnss_valid;
   float               hdg;
   float               hdg_rate;
   float               gnss_pitch;
   float               spd;
   float               spd_rate;
   float               odo;
   unsigned char       reverse_from_prndl;
   char                reverse_output;
   float               odo_sf;
   unsigned int        odo_status;
   unsigned char       odo_gain;
   float               gnss_spd_accum;
   float               odo_accum;
} tQdrA6DOFGnssOdoInfo;

/* QDR Core API Diag Structure for GNSS Channel Info. mirror of MI_DR_A6DOF_GNSS_CHAN for OSP message 48,22 */
typedef struct _tQdrA6DOFGnssChanInfo {
   unsigned char       gnss_meas_available;
   unsigned char       sv_id;
   float               pr_res;
   float               dr_res;
   unsigned short      sv_elevation;
   unsigned short      sv_azimuth;
   float               sv_meas_avg_pwr;
} tQdrA6DOFGnssChanInfo;
/* QDR Core API Diag Structure for GNSS Channel data. Mirror of MI_DR_A6DOF_GNSS_CHAN_LIST for OSP message 48,22 */
/**Diag ID QDR_DIAG_A6DOF_GNSS_CHAN_INFO*/
typedef struct _tQdrA6DOFGnssChanList {
   double               tow;
   unsigned char        num_chan;
   tQdrA6DOFGnssChanInfo gnss_chan_list[QDR_CORE_SAT_DATA_NUM_OF_SATS];
} tQdrA6DOFGnssChanList;

/* QDR2 Core API Diag Structure for NavState and mirror of MI_DR_NAV_STATUS and MI_DR_NAV_STATE Data
   for OSP message 48,1 and 48,2 respectively */
/**Diag ID QDR2_DIAG_NAVSTATUS*/
typedef struct _tQdr2NavStatus {
      unsigned int  Nav;             /* DR Navigation Validity:
                               *  0 = DR Nav is valid
                               *  otherwise, bit map as follows:
                               *    Bit 0 : 1 = GPS Only Required
                               *    Bit 1 : 1 = Speed is not 0 at startup
                               *    Bit 2 : 1 = DR Position is not valid
                               *    Bit 3 : 1 = DR Heading is not valid
                               *    Bit 4 : 1 = DR Calibration is not valid
                               *    Bit 5 : 1 = DR Data is not valid
                               *    Bit 6 : 1 = System has gone into Cold Start
                               *    Bit 7 : reserved
                               */
      unsigned short Data;            /* DR Data Validity:
                               *  0 = DR Data is valid
                               *  otherwise, bit map as follows:
                               *    Bit 0 : 1 = DR Gyro Subsystem Operational is false
                               *    Bit 1 : 1 = DR Speed Subsystem Operational is false
                               *    Bit 2 : 1 = DR Measurement Time < 0
                               *    Bit 4 : 1 = No DR Data for > 2 seconds
                               *    Bit 5 : 1 = DR Data timestamp did not advance
                               *    Bit 6 : 1 = DR data bytes all 0x00 or all 0xff
                               *    Bit 7 : 1 = Composite wheeltick count jumped too much
                               *                (by more than 400) between successive DR messages
                               *    Bit 8 : 1 = Input Gyro data bits value of 0x0000 or 0x3fff
                               *    Bit 9 : 1 = More than 10 DR messages in one second
                               *    Bit 10: 1 = Delta time <= 0
                               *    other bits are reserved
                               */
      unsigned int  Cal_GbCal;       /* DR Calibration Validity (Bits 0 to 3):
                               *  0 = DR Calibration is valid
                               *  otherwise, bit map as follows:
                               *    Bit 0 : 1 = DR Gyro Bias cal is not valid
                               *    Bit 1 : 1 = DR Gyro Scale Factor cal is not valid
                               *    Bit 2 : 1 = DR Speed Scale Factor cal is not valid
                               *    Bit 3 : 1 = GPS calibration is required and is not yet available
                               * DR Gyro Bias Calibration Validity (Bits 4 to 6):
                               *  0 = DR Gyro Bias Calibration is valid
                               *  otherwise, bit map as follows:
                               *    Bit 4 : 1 = DR Data is not valid
                               *    Bit 5 : 1 = Zero-Speed Gyro Bias Calibration was not updated
                               *    Bit 6 : 1 = Heading Rate Scale Factor <= -1
                               */
      unsigned int  GsfCal_SsfCal;   /* DR Gyro Scale Factor Calibration Validity (Bits 0 to 3):
                               *  0 = DR Gyro Scale Factor calibration is valid
                               *  otherwise, bit map as follows:
                               *    Bit 0 : 1 = DR Heading is not valid
                               *    Bit 1 : 1 = DR Data is not valid
                               *    Bit 2 : 1 = DR Position is not valid
                               *    Bit 3 : 1 = Heading Rate Scale Factor <= -1
                               * DR Speed Scale Factor Calibration Validity (Bits 4 to 7):
                               *  0 = DR Speed Scale Factor calibration is valid
                               *  otherwise, bit map as follows:
                               *    Bit 4 : 1 = DR Data is not valid
                               *    Bit 5 : 1 = DR Position is not valid
                               *    Bit 6 : 1 = GPS velocity is not valid for DR
                               *    Bit 7 : 1 = DR Speed Scale Factor <= -1
                               */
      unsigned int  NavAcrossReset_Pos; /* DR Nav Validity across reset (Bits 0 to 1):
                                  *  0 = DR Nav is valid across reset
                                  *  otherwise, bit map as follows:
                                  *    Bit 0 : 1 = DR Navigation is not valid
                                  *    Bit 1 : 1 = Speed > 0.1 m/sec
                                  * DR Position Validity (Bits 3 to 6):
                                  *  0 = DR Position is valid
                                  *  otherwise, bit map as follows:
                                  *    Bit 3 : 1 = Speed != 0 at startup
                                  *    Bit 4 : 1 = GPS Position is not valid
                                  *    Bit 5 : 1 = System has gone into Cold Start
                                  *    Bit 6 : 1 = DR Data is not valid
                                  * Bits 2 and 7 : reserved
                                  */
      unsigned int  Hd;               /* DR Heading Validity (Bits 0 to 6):
                                *  0 = DR Heading is valid
                                *  otherwise, bit map as follows:
                                *    Bit 0 : 1 = Speed != 0 at startup
                                *    Bit 1 : 1 = GPS Position is not valid
                                *    Bit 2 : 1 = GPS Speed is not valid
                                *    Bit 3 : 1 = GPS Updated Heading flag is false
                                *    Bit 4 : 1 = Delta GPS Time <= 0.0 or >= 2.0
                                *    Bit 5 : 1 = System has gone into Cold Start
                                *    Bit 6 : 1 = DR Data is not valid
                                * Other bits are reserved
                                */
      unsigned int  GyrSubOp_SpdSubOp; /* DR Gyro Subsystem Operational (Bits 0 to 2):
                                 *  0 = OK
                                 *  otherwise, bit map as follows:
                                 *    Bit 0 : 1 = High, Persistent Turn Rate
                                 *    Bit 1 : 1 = Low, Persistent Turn Rate
                                 *    Bit 2 : 1 = Gyro Turn Rate residual is too large
                                 * DR Speed Subsystem Operational (Bits 4 to 6):
                                 *  0 = OK
                                 *  otherwise, bit map as follows:
                                 *    Bit 4 : 1 = DR Speed Data = 0 when GPS Speed != 0
                                 *    Bit 5 : 1 = DR Speed Data != 0 when GPS Speed = 0
                                 *    Bit 6 : 1 = DR Speed residual is too large
                                 * Other bits are reserved.
                                 */
      unsigned int  NavStIntRan_ZGbCalUpd; /* DR Nav State Integration Ran (Bits 0 to 2):
                                     *  0 = OK
                                     *  otherwise, bit map as follows:
                                     *    Bit 0 : 1 = DR Position is not valid
                                     *    Bit 1 : 1 = DR Heading is not valid
                                     *    Bit 2 : 1 = DR Data is not valid
                                     * Zero-Speed Gyro Bias Calibration Updated (Bits 4 to 6):
                                     *  0 = Updated
                                     *  otherwise, bit map as follows:
                                     *    Bit 4 : 1 = GPS Speed > 0.1 m/sec
                                     *    Bit 5 : 1 = Zero Speed During Cycle is false
                                     *    Bit 6 : 1 = Zero Speed Previous is false
                                     * Other bits are reserved.
                                    */
      unsigned int  GbsfCalUpd_SpdCalUpd_UpdNavSt; /* DR Gyro Bias and Scale Factor Calibration Updated (Bits 0 to 3):
                                             *  0 = Updated
                                             *  otherwise, bit map as follows:
                                             *    Bit 0 : 1 = DR Data is not valid
                                             *    Bit 1 : 1 = DR Position is not valid
                                             *    Bit 2 : 1 = GPS velocity is not valid for DR
                                             *    Bit 3 : 1 = GPS Updated Heading is false
                                             * DR Speed Calibration Updated (Bits 4 to 6):
                                             *  0 = Updated
                                             *  otherwise, bit map as follows:
                                             *    Bit 4 : 1 = DR Data is not valid
                                             *    Bit 5 : 1 = DR Position is not valid
                                             *    Bit 6 : 1 = GPS velocity is not valid for DR
                                             * DR Navigation State Updated (Bit 7)
                                             *    0 = Updated;  1 = DR Navigation is not valid
                                            */
      unsigned int  GpsUpdPos;       /* GPS Updated Position (Bits 0 to 7):
                               *  0 = Updated
                               *  otherwise, bit map as follows:
                               *    Bit 0 : 1 = Update Mode != KF
                               *    Bit 1 : 1 = EHE > 10.0
                               *    Bit 2 : 1 = No previous GPS KF update < 4 SVs
                               *    Bit 3 : 1 = GPS EHPE > DR EHPE
                               *    Bit 4 : 1 = DR EHPE < 10 even if GPS EHPE < DR EHPE
                               *    Bit 5 : 1 = Less than 4 SVs
                               *    Bit 6 : 1 = 0 SV
                               *    Bit 7 : 1 = DR NAV ONLY required
                              */
      unsigned int  GpsUpdHd;        /* GPS Updated Heading (Bits 0 to 6):
                               *  0 = Updated
                               *  otherwise, bit map as follows:
                               *    Bit 0 : 1 = Update Mode != KF
                               *    Bit 1 : 1 = GPS Speed <= 2.0 m/sec
                               *    Bit 2 : 1 = Less than 4 SVs
                               *    Bit 3 : 1 = Horizontal Velocity Variance > 1.0 (m/sec)*(m/sec)
                               *    Bit 4 : 1 = GPS Heading Error >= DR Heading Error * 1.2
                               *    Bit 5 : 1 = GPS KF Updated is false
                               *    Bit 6 : 1 = Initial Speed Transient is not complete
                               *    Bit 7 : Reserved
                              */
      unsigned int  GpsPos_GpsVel;   /* GPS Position Validity for DR (Bits 0 to 2):
                               *  0 = Valid
                               *  otherwise, bit map as follows:
                               *    Bit 0 : 1 = Less than 4 SVs
                               *    Bit 1 : 1 = EHPE > 30
                               *    Bit 2 : 1 = GPS Updated Position is false
                               * Bit 3 : reserved
                               * GPS Velocity Validity for DR (Bits 4 to 7):
                               *  0 = Valid
                               *  otherwise, bit map as follows:
                               *    Bit 4 : 1 = GPS Position is not valid for DR
                               *    Bit 5 : 1 = EHVE > 3
                               *    Bit 6 : 1 = GPS Speed < 2 m/sec
                               *    Bit 7 : 1 = GPS did not update the Heading
                              */
      unsigned int  DWSHdRtSFCalValid; /* DWS Heading Rate Scale Factor Calibration Validity:
                                 *  0 = Valid
                                 *  otherwise, bit map as follows:
                                 *    Bit 0 : 1 = Heading Rate Scale Factor <= -1.0
                                 *    Bits 1 - 7 : Reserved
                                */
      unsigned int  DWSHdRtSFCalUpd; /* DWS Heading Rate Scale Factor Calibration Was Updated:
                               *  0 = Updated
                               * otherwise, bit map as follows:
                               *    Bit 0 : 1 = GPS Heading Rate is not valid
                               *    Bit 1 : 1 = Absolute value of GPS Heading Rate < 5.0
                               *    Bit 2 : 1 = Absolute value of GPS Heading Rate >= 90.0
                               *    Bit 3 : 1 = Left Rear Speed SF Cal is not valid
                               *    Bit 4 : 1 = Right Rear Speed SF Cal is not valid
                               *    Bit 5 : 1 = Absolute value of prev Rear Axle Hd Rt <= 0.0
                               *    Bit 6 : 1 = (GPS Hd Rt * prev Rear Axle Hd Rt) <= 1.0
                               *    Bit 7 : reserved
                              */
      unsigned short DWSSpdSFCalValid; /* DWS Speed Scale Factor Calibration Validity:
                                *  0 = Valid
                                * otherwise, bit map as follows:
                                *  Bit 0 : 1 = Right Rear Speed SF <= -1.0
                                *  Bit 1 : reserved for RR status
                                *  Bit 2 : reserved for RR status
                                *  Bit 3 : reserved for RR status
                                *  Bit 4 : 1 = Left Rear Speed SF <= -1.0
                                *  Bit 5 : reserved for LR status
                                *  Bit 6 : reserved for LR status
                                *  Bit 7 : reserved for LR status
                                *  Bit 8 : 1 = Right Front Speed SF <= -1.0
                                *  Bit 9 : reserved for RF status
                                *  Bit 10: reserved for RF status
                                *  Bit 11: reserved for RF status
                                *  Bit 12: 1 = Left Front Speed SF <= -1.0
                                *  Bit 13: reserved for LF status
                                *  Bit 14: reserved for LF status
                                *  Bit 15: reserved for LF status
                               */
      unsigned int DWSSpdSFCalUpd;    /* DWS Speed Scale Factor Cal was updated:
                                *  0 = Updated
                                * otherwise, bit map as follows:
                                *  Bit 0 : 1 = GPS Speed is not valid for DR
                                *  Bit 1 : 1 = GPS Heading Rate is not valid
                                *  Bit 2 : 1 = Absolute value of GPS Hd Rate >= 0.23
                                *  Bit 3 : 1 = GPS Heading Rate Error >= 0.5
                                *  Bit 4 : 1 = Average GPS Speed <= 0.0
                                *  Bit 5 : 1 = DR Position is not valid
                                *  Bits 6 - 7 : reserved
                               */

} tQdr2NavStatus;

/* QDR2 Core API Diag Structure for NavState and mirror of MI_DR_NAV_STATE Data for OSP message 48,2*/
/**Diag ID QDR2_DIAG_NAVSTATE*/
typedef struct _tQdr2NavState {
   unsigned short Spd;       /* DR Speed (m/sec) with resolution 0.01 */
   unsigned short SpdE;      /* DR Speed Error (m/sec) with resolution 0.01 */
   short  Ssf;       /* DR Speed Scale Factor (no unit) with resolution 0.0001 */
   unsigned short SsfE;      /* DR Speed Scale Factor Error (no unit) with resolution 0.0001 */
   short  HdRte;     /* DR Heading Rate (deg/sec) with resolution 0.01 */
   unsigned short HdRteE;    /* DR Heading Rate Error (deg/sec) with resolution 0.01 */
   short  Gb;        /* DR Gyro Bias (deg/sec) with resolution 0.01 */
   unsigned short GbE;       /* DR Gyro Bias Error (deg/sec) with resolution 0.01 */
   short  Gsf;       /* DR Gyro Scale Factor (no unit) with resolution 0.0001 */
   unsigned short GsfE;      /* DR Gyro Scale Factor Error (no unit) with resolution 0.0001 */
   unsigned int TPE;       /* Total DR Position Error (meters) with resolution 0.01 */
   unsigned short THE;       /* Total DR Heading Error (degrees) with resolution 0.01 */
   unsigned int  NavCtrl;   /* DR Nav Mode Control:
                      *   0x01 = GPS Only Navigation Required (No DR Nav Allowed)
                      *   0x02 = GSP + DR using default or stored calibration
                      *   0x03 = GPS + DR using current GPS calibration
                      *   0x04 = DR Only Navigation (no GPS)
                     */
   unsigned int  Reverse;   /* DR Direction:  0 = forward; 1 = reverse */
   unsigned short Hd;        /* DR Heading (deg/sec) with resolution 0.01 */
   unsigned int  SensorPkg; /* Identify which sensor package used:
                      *   0 = Gyro and Odo
                      *   1 = Wheel Speed and Odo
                      */
   unsigned short OdoSpd;            /* Odometer Speed (m/sec) with resolution 0.01 */
   short  OdoSpdSF;          /* Odometer Speed Scale Factor (no unit) with resolution 0.0001 */
   unsigned short OdoSpdSFErr;       /* Odometer Speed Scale Factor Error (no unit) with resolution 0.0001 */
   short  LFWheelSpdSF;      /* Left Front Wheel Speed Scale Factor (no unit) with resolution 0.0001 */
   unsigned short LFWheelSpdSFErr;   /* Left Front Wheel Speed Scale Factor Error (no unit) with resolution 0.0001 */
   short  RFWheelSpdSF;      /* Right Front Wheel Speed Scale Factor (no unit) with resolution 0.0001 */
   unsigned short RFWheelSpdSFErr;   /* Right Front Wheel Speed Scale Factor Error (no unit) with resolution 0.0001 */
   short  LRWheelSpdSF;      /* Left Rear Wheel Speed Scale Factor (no unit) with resolution 0.0001 */
   unsigned short LRWheelSpdSFErr;   /* Left Rear Wheel Speed Scale Factor Error (no unit) with resolution 0.0001 */
   short  RRWheelSpdSF;      /* Right Rear Wheel Speed Scale Factor (no unit) with resolution 0.0001 */
   unsigned short RRWheelSpdSFErr;   /* Right Rear Wheel Speed Scale Factor Error (no unit) with resolution 0.0001 */
   short  RearAxleSpdDelta;  /* Rear Axle Speed Delta (m/sec) with resolution 0.01 */
   unsigned short RearAxleAvgSpd;    /* Rear Axle Average Speed (m/sec) with resolution 0.01 */
   unsigned short RearAxleSpdErr;    /* Rear Axle Speed Error (m/sec) with resolution 0.01 */
   short  RearAxleHdRt;      /* Rear Axle Heading Rate (deg/sec) with resolution 0.01 */
   unsigned short RearAxleHdRtErr;   /* Rear Axle Heading Rate Error (deg/sec) with resolution 0.01 */
   short  FrontAxleSpdDelta; /* Front Axle Speed Delta (m/sec) with resolution 0.01 */
   unsigned short FrontAxleAvgSpd;   /* Front Axle Average Speed (m/sec) with resolution 0.01 */
   unsigned short FrontAxleSpdErr;   /* Front Axle Speed Error (m/sec) with resolution 0.01 */
   short  FrontAxleHdRt;     /* Front Axle Heading Rate (deg/sec) with resolution 0.01 */
   unsigned short FrontAxleHdRtErr;  /* Front Axle Heading Rate Error (deg/sec) with resolution 0.01 */
} tQdr2NavState;

typedef struct _tQdr2NavSubSys{

   short  GpsHdRte;     /* GPS Heading Rate (deg/sec) with resolution 0.01 */
   unsigned short GpsHdRteE;    /* GPS Heading Rate Error (deg/sec) with resolution 0.01 */
   unsigned short GpsHd;        /* GPS Heading (deg) with resolution 0.01 */
   unsigned short GpsHdE;       /* GPS Heading Error (deg) with resolution 0.01 */
   unsigned short GpsSpd;       /* GPS Speed (m/sec) with resolution 0.01 */
   unsigned short GpsSpdE;      /* GPS Speed Error (m/sec) with resolution 0.01 */
   unsigned int GpsPosE;      /* GPS Position Error (m) with resolution 0.01 */
   short  DrHdRte;      /* DR Heading Rate (m/sec) with resolution 0.01 */
   unsigned short DrHdRteE;     /* DR Heading Rate Error (m/sec) with resolution 0.01 */
   unsigned short DrHd;         /* DR Heading (deg) with resolution 0.01 */
   unsigned short DrHdE;        /* DR Heading Error (deg) with resolution 0.01 */
   unsigned short DrSpd;        /* DR Speed (m/sec) with resolution 0.01 */
   unsigned short DrSpdE;       /* DR Speed Error (m/sec) with resolution 0.01 */
   unsigned int DrPosE;       /* DR Position Error (m) with resolution 0.01 */
   unsigned char  DrCalPercent; /* DR calibration Percentage */
   unsigned char  Reserved;

} tQdr2NavSubSys;

#define QDR_DIAG_VERSION_INFO               0x0001
typedef void (*qdr_cb_reportSvStatus_t)( tQdrSvStateData *pQdrSvstatusData);
typedef void (* qdr_cb_reportPosition_t)(tQdrNavigationData *pQdrNavData,
                                         tQdrCoreNavBfdParams *pQdrBFData);
typedef void (* qdr_cb_clearNVData_t)(unsigned char dataIDToClear);
typedef qdr_return (* qdr_cb_getSVState_t)(unsigned char svid,double measTow,
                                           tQdrSVState *pSvState);
typedef qdr_return (* qdr_cb_getDGPSCorrection_t)(unsigned char svid, double measTow,
                                                  tQdrDGPSMeasStruct *pDGPS);
typedef void (* qdr_cb_debugLog_t)(const char *pFmt,int cnt);
typedef unsigned int (* qdr_getsystime_t)(void);
typedef void (* qdr_cb_reportNavInitData_t)(tQdrNavInitData *pQdrNavInitData);
typedef qdr_return (* qdr_cb_createThreadsyncObject)(qdr_threadsync_object objectID,
                                                     unsigned char createLocked);
typedef void (* qdr_cb_destroyThreadsyncObject)(qdr_threadsync_object objectID);
typedef qdr_return (* qdr_cb_waitlockThreadsyncObject)(qdr_threadsync_object objectID);
typedef qdr_return (* qdr_cb_unlockThreadsyncObject)(qdr_threadsync_object objectID);


//typedef void (* qdr_cb_debugLog_t)(const char *pFmt,...);
//typedef void (* qdr_cb_debugLog_t)( qdr_outPutpost *Pos);

typedef enum
{
   eQDR_DIAGLOG_SENSOR_DATA_PRODUCED,
   eQDR_DIAGLOG_SENSOR_DATA_CONSUMED,
   eQDR_DIAGLOG_QDR_A6DOF_NF_STATE_KF,
   eQDR_DIAGLOG_QDR_A6DOF_NF_STATE_LSQ,
   eQDR_DIAGLOG_QDR_A6DOF_NF_STATE_PREROTATION,
   eQDR_DIAGLOG_QDR_A6DOF_NF_STATE_B2S_RP,
   eQDR_DIAGLOG_QDR_A6DOF_NF_STATE_B2S_YAW,
   eQDR_DIAGLOG_QDR_A6DOF_NF_STATE_NAVIGATING,
   eQDR_DIAGLOG_QDR_NAVLITE_START,
   eQDR_DIAGLOG_QDR_NAVLITE_END
}dr_gluediagevent;

typedef qdr_return (* qdr_log_diagEvent_t)(dr_gluediagevent event_id);

/** structure for QDR Core callbacks*/
typedef struct _tQdrCallbacks
{
   qdr_cb_getSVState_t      getSVState;
   qdr_cb_getDGPSCorrection_t getDGPScorrection;
   qdr_cb_reportPosition_t  reportPosition_cb;
   qdr_cb_reportNavInitData_t reportNavInitData_cb;
   qdr_cb_reportSvStatus_t  reportSVStatus_cb;
   qdr_cb_sendDIAGData_t   sendDIAGData_cb;
   qdr_cb_storeNVData_t     storeNVData_cb;
   qdr_cb_retrieveNVData_t  retrieveNVData_cb;
   qdr_cb_clearNVData_t     clearNVData_cb;
}tQdrCallbacks;

/** structure for Platform IF callbacks*/
typedef struct _tQdrPlatfCallbacks
{
   qdr_cb_debugLog_t        debugLog_cb;
   qdr_getsystime_t         getTime_cb;
   qdr_log_diagEvent_t      log_diag_event_cb;
   qdr_cb_createThreadsyncObject createThreadSyncObject_cb;
   qdr_cb_destroyThreadsyncObject destroyThreadSyncObject_cb;
   qdr_cb_waitlockThreadsyncObject lockThreadSyncObject_cb;
   qdr_cb_unlockThreadsyncObject   unlockThreadSyncObject_cb;

}tQdrPlatfCallbacks;

/**NVM Parameters for NAV-A6DOF*/
#define QDR_NV_FILE_QDR3_DATA        0x0001
/**CtrlnavCtrl*/
#define QDR_NV_FILE_NLCTRL_DATA        0x0002

/** NAVInitDataSrc,tNLAuxInitInfo,MI_NAV_INIT data to be stored*/
#define QDR_NV_FILE_NLINIT_DATA        0x003

/**NVM Parameters for NAV*/
#define QDR_NV_FILE_QDR2_DATA        0x0004

qdr_return qdr_init(tQdrCallbacks *pQdrCallbacks,tQdrPlatfCallbacks *pQdrPlatfCallbacks);
qdr_return qdr_shutdown(void);

#define A6DOF_MAX_SENSORS  9
#define A6DOF_MAX_SENSOR_SETS  11

typedef struct _tQdr_SensorSet
{
   unsigned short    validity;
   unsigned int      time;
   short     data[A6DOF_MAX_SENSORS];
} tQdr_SensorSet;

/**This is a bitmap that describes the validity of the various information
* elements in the sensor data set.*/
#define A6DOF_SENSOR_VALID_TIME_TAG  (0x0001)
#define A6DOF_SENSOR_VALID_DATA0     (0x0002)
#define A6DOF_SENSOR_VALID_DATA1     (0x0004)
#define A6DOF_SENSOR_VALID_DATA2     (0x0008)
#define A6DOF_SENSOR_VALID_DATA3     (0x0010)
#define A6DOF_SENSOR_VALID_DATA4     (0x0020)
#define A6DOF_SENSOR_VALID_DATA5     (0x0040)
#define A6DOF_SENSOR_VALID_DATA6     (0x0080)
#define A6DOF_SENSOR_VALID_DATA7     (0x0100)
#define A6DOF_SENSOR_VALID_DATA8     (0x0200)
/** This bit will be appended in the API call to indicate that the data
*corresponds to the GNSS epoch and DR sync pulse */
#define A6DOF_EPOCH_DATA_FLAG        (0x8000)


typedef struct _tQdrSensorDataSet2NAV
{
   unsigned char            type;
   unsigned char            num_sets;
   unsigned short           reserved;
   tQdr_SensorSet DataSet[A6DOF_MAX_SENSOR_SETS];
 } tQdrSensorData2NAV;

/*******************************************
* Definitions for Sensor Data Types
********************************************/

#define QDR_SDT_UNKNOWN           (0xff)

#define QDR_SDT_ODO_GYRO_REV      (1)
#define QDR_SDT_DIF_PULSES_REV    (2)
#define QDR_SDT_DIF_SPEEDS_REV    (3)
#define QDR_SDT_DIF_ANGLRT_REV    (4)
#define QDR_SDT_ODO_GYRO_NOREV    (5)
#define QDR_SDT_DIF_PULSES_NOREV  (6)
#define QDR_SDT_DIF_SPEEDS_NOREV  (7)
#define QDR_SDT_DIF_ANGLRT_NOREV  (8)
#define QDR_SDT_3G_3A_ODO_REV     (14)

/**An array of sensor data sets each corresponding to a 100ms period*/

qdr_return qdr_transfer1HzSensorData(tQdrSensorData2NAV *pQdrSensorData);

qdr_return qdr_setSystemTime(qdr_sysTime *time);

qdr_return qdr_transferGnssMsmt(unsigned char numChannels,tQdrGnssMeas2NAV *pGnssMeasData);

qdr_return qdr_do1HzNav(void);
qdr_return qdr_do10HzNav(unsigned char SensorDataType,tQdr_SensorSet *pSensorDataSet);

qdr_return qdr_getVersionInfo(void);

void NavStoreNVMData(void);
void NavInitData(void);

qdr_return qdr_clearNVData(unsigned int  dataIDToClear);
#define QDR_CLEAR_DR_DATA              0x4000
#define QDR_CLEAR_ALL                  0xFFFF

qdr_return qdr_clearRAMData(unsigned int  dataIDToClear);
#define QDR_CLEAR_EPHEMERIS            0x0001
#define QDR_CLEAR_ALMANAC              0x0002
#define QDR_CLEAR_POSITION             0x0004
#define QDR_CLEAR_TIME                 0x0008
#define QDR_CLEAR_IONO                 0x0010
#define QDR_CLEAR_UTC                  0x0020
#define QDR_CLEAR_HEALTH               0x0040
#define QDR_CLEAR_SVDIR                0x0080
#define QDR_CLEAR_SVSTEER              0x0100
#define QDR_CLEAR_SADATA               0x0200
#define QDR_CLEAR_RTI                  0x0400
#define QDR_CLEAR_CELLDB_INFO          0x8000
#define QDR_CLEAR_DR_DATA              0x4000
#define QDR_CLEAR_ALL                  0xFFFF

#ifdef __cplusplus
}
#endif
#endif /*__QDR_IF__*/
