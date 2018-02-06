/*
*    Copyright (c) 2014-2015 Qualcomm Technologies, Inc. All Rights Reserved.
*    Qualcomm Technologies Proprietary and Confidential.
*
*/
package com.qualcomm.qti;

/**
* VehicleSignalConstants provides constant definitions for various vehicle signals.
* Please note that this API is work in progress, and there might be few changes going forward.
*
*/
public final class VehicleSignalConstants {

    /**
    * These intents are thrown from VNW when certain CAN message arrives.
    * Please refer to vnwmappings.xml to find out more about how to configure specific
    * CAN message. Receiver of these intents is responsible for taking the appropriate action.
    */
    public static final String VNW_INTENT_VOLUME_UP        = "VNW.INTENT.VOLUME.UP";
    public static final String VNW_INTENT_VOLUME_DOWN      = "VNW.INTENT.VOLUME.DOWN";
    public static final String VNW_INTENT_MEDIA_NEXT       = "VNW.INTENT.MEDIA.NEXT";
    public static final String VNW_INTENT_MEDIA_PREV       = "VNW.INTENT.MEDIA.PREV";
    public static final String VNW_INTENT_MEDIA_PAUSE      = "VNW.INTENT.MEDIA.PAUSE";
    public static final String VNW_INTENT_MEDIA_PLAY       = "VNW.INTENT.MEDIA.PLAY";
    public static final String VNW_INTENT_MEDIA_FORWARD    = "VNW.INTENT.MEDIA.FWD";
    public static final String VNW_INTENT_MEDIA_REWIND     = "VNW.INTENT.MEDIA.RWD";
    public static final String VNW_INTENT_HOME_BTN         = "VNW.INTENT.HOME.BTN";
    public static final String VNW_INTENT_POWER_BTN        = "VNW.INTENT.POWER.BTN";
    public static final String VNW_INTENT_MENU_BTN         = "VNW.INTENT.MENU.BTN";
    public static final String VNW_INTENT_ENTER_BTN        = "VNW.INTENT.ENTER.BTN";
    public static final String VNW_INTENT_BACK_BTN         = "VNW.INTENT.BACK.BTN";
    public static final String VNW_INTENT_START_VOICE_CALL = "VNW.INTENT.START.VOICE.CALL";
    public static final String VNW_INTENT_END_VOICE_CALL   = "VNW.INTENT.END.VOICE.CALL";
    public static final String VNW_INTENT_DPAD_LEFT        = "VNW.INTENT.DPAD.LEFT";
    public static final String VNW_INTENT_DPAD_RIGHT       = "VNW.INTENT.DPAD.RIGHT";
    public static final String VNW_INTENT_DPAD_CENTER      = "VNW.INTENT.DPAD.CENTER";
    public static final String VNW_INTENT_DPAD_UP          = "VNW.INTENT.DPAD.UP";
    public static final String VNW_INTENT_DPAD_DOWN        = "VNW.INTENT.DPAD.DOWN";
    public static final String VNW_INTENT_VOICE_ASSIST     = "VNW.INTENT.VOICE.ASSIST";

    //Data types associated with signals.
    //Please refer to VehicleInterfaceData.getSignalsDataType.
    public static final int DATA_TYPE_UNKNOWN                       = 0;
    public static final int DATA_TYPE_INTEGER                       = 1;
    public static final int DATA_TYPE_FLOAT                         = 2;
    public static final int DATA_TYPE_STRING                        = 3;

    //Constant definations for signals related to Transmission
    public static final short VEHICLE_POWERMODE_OFF                 = 1;
    public static final short VEHICLE_POWERMODE_ACC                 = 2;
    public static final short VEHICLE_POWERMODE_RUN                 = 3;
    public static final short VEHICLE_POWERMODE_IGNITION            = 4;
    public static final short TRANSMISSION_GEARSTATUS_NEUTRAL       = 0;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL1       = 1;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL2       = 2;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL3       = 3;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL4       = 4;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL5       = 5;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL6       = 6;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL7       = 7;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL8       = 8;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL9       = 9;
    public static final short TRANSMISSION_GEARSTATUS_MANUAL10      = 10;
    public static final short TRANSMISSION_GEARSTATUS_AUTO1         = 11;
    public static final short TRANSMISSION_GEARSTATUS_AUTO2         = 12;
    public static final short TRANSMISSION_GEARSTATUS_AUTO3         = 13;
    public static final short TRANSMISSION_GEARSTATUS_AUTO4         = 14;
    public static final short TRANSMISSION_GEARSTATUS_AUTO5         = 15;
    public static final short TRANSMISSION_GEARSTATUS_AUTO6         = 16;
    public static final short TRANSMISSION_GEARSTATUS_AUTO7         = 17;
    public static final short TRANSMISSION_GEARSTATUS_AUTO8         = 18;
    public static final short TRANSMISSION_GEARSTATUS_AUTO9         = 19;
    public static final short TRANSMISSION_GEARSTATUS_AUTO10        = 20;
    public static final short TRANSMISSION_GEARSTATUS_DRIVE         = 32;
    public static final short TRANSMISSION_GEARSTATUS_PARKING       = 64;
    public static final short TRANSMISSION_GEARSTATUS_REVERSE       = 128;
    public static final short ENGINE_COOLANT_LEVEL_NORMAL           = 0;
    public static final short ENGINE_COOLANT_LEVEL_LOW              = 1;

    //Constant definations for signals related to Safety
    public static final String ABS_AVAILABLE                        = "ABS_AVAILABLE";
    public static final String ABS_IDLE                             = "ABS_IDLE";
    public static final String ABS_ACTIVE                           = "ABS_ACTIVE";
    public static final String TCS_AVAILABLE                        = "TCS_AVAILABLE";
    public static final String TCS_IDLE                             = "TCS_IDLE";
    public static final String TCS_ACTIVE                           = "TCS_ACTIVE";
    public static final String ESC_AVAILABLE                        = "ECS_AVAILABLE";
    public static final String ESC_IDLE                             = "ECS_IDLE";
    public static final String ESC_ACTIVE                           = "ECS_ACTIVE";
    public static final String AIRBAG_ACTIVATE                      = "AIRBAG_ACTIVATE";
    public static final String AIRBAG_DEACTIVATE                    = "AIRBAG_DEACTIVATE";
    public static final String AIRBAG_DEPLOYMENT                    = "AIRBAG_DEPLOYMENT";
    public static final String DOOR_STATUS_OPEN                     = "DOOR_OPEN";
    public static final String DOOR_STATUS_AJAR                     = "DOOR_AJAR";
    public static final String DOOR_STATUS_CLOSE                    ="DOOR_CLOSE";
    public static final String OCCUPANT_ADULT                       = "ADULT_OCCUPANT";
    public static final String OCCUPANT_CHILD                       = "CHILD_OCCUPANT";
    public static final String OCCUPANT_VACANT                      = "VACANT";

    //Constant definations for signals related to Parking
    public static final short SECURITYALERT_AVAILABLE               = 1;
    public static final short SECURITYALERT_IDLE                    = 2;
    public static final short SECURITYALERT_ACTIVATED               = 3;
    public static final short SECURITYALERT_ALARM_DETECTED          = 4;

    //Constant definations for signals related to Vehicle overview
    public static final String  VEHICLETYPE_SEDAN                   = "SEDAN";
    public static final String  VEHICLETYPE_COUPE                   = "COUPE";
    public static final String  VEHICLETYPE_CABRIOLET               = "CABRIOLET";
    public static final String  VEHICLETYPE_ROADSTER                = "ROADSTER";
    public static final String  VEHICLETYPE_SUV                     = "SUV";
    public static final String  VEHICLETYPE_TRUCK                   = "TRUCK";
    public static final String  VEHICLETYPE_MINIVAN                 = "MINI-VAN";
    public static final String  VEHICLETYPE_VAN                     = "VAN";
    public static final short FUELTYPE_GASOLINE                     = 0x01;
    public static final short FUELTYPE_METHANOL                     = 0x02;
    public static final short FUELTYPE_ETHANOL                      = 0x03;
    public static final short FUELTYPE_DIESEL                       = 0x04;
    public static final short FUELTYPE_LPG                          = 0x05;
    public static final short FUELTYPE_CNG                          = 0x06;
    public static final short FUELTYPE_PROPANE                      = 0x07;
    public static final short FUELTYPE_ELECTRIC                     = 0x08;
    public static final short FUELTYPE_BI_FUEL_RUNNING_GASOLINE     = 0x09;
    public static final short FUELTYPE_BI_FUEL_RUNNING_METHANOL     = 0x0A;
    public static final short FUELTYPE_BI_FUEL_RUNNING_ETHANOL      = 0x0B;
    public static final short FUELTYPE_BI_FUEL_RUNNING_LPG          = 0x0C;
    public static final short FUELTYPE_BI_FUEL_RUNNING_CNG          = 0x0D;
    public static final short FUELTYPE_BI_FUEL_RUNNING_PROP         = 0x0E;
    public static final short FUELTYPE_BI_FUEL_RUNNINGELECTRICITY   = 0x0F;
    public static final short FUELTYPE_BI_FUEL_MIXEDGASELECTRIC     = 0x10;
    public static final short FUELTYPE_HYBRID_GASOLINE              = 0x11;
    public static final short FUELTYPE_HYBRID_ETHANOL               = 0x12;
    public static final short FUELTYPE_HYBRID_DIESEL                = 0x13;
    public static final short FUELTYPE_HYBRID_ELECTRIC              = 0x14;
    public static final short FUELTYPE_HYBRID_MIXEDFUEL             = 0x15;
    public static final short FUELTYPE_HYBRID_REGENERATIVE          = 0x16;
    public static final String TRANSMISSION_GEAR_TYPE_AUTO          = "AUTO-TRANSMISSION";
    public static final String TRANSMISSION_GEAR_TYPE_MANUAL        = "MANUAL-TRANSMISSION";
    public static final String TRANSMISSION_GEAR_TYPE_CVT           = "CVT-TRANSMISSION";

    //Constant definations for signals related to Media
    public static final short MEDIA_AUDIO_SOURCE_USB                = 0;
    public static final short MEDIA_AUDIO_SOURCE_BT                 = 1;
    public static final short MEDIA_AUDIO_SOURCE_RADIO              = 2;
    public static final short MEDIA_AUDIO_SOURCE_CD                 = 3;
    public static final short MEDIA_AUDIO_SOURCE_IPOD               = 4;

    //Constant definations for signals related to Maintenance
    public static final short TIRE_PRESSURESTATUS_NORMAL            = 0;
    public static final short TIRE_PRESSURESTATUS_LOW               = 1;
    public static final short TIRE_PRESSURESTATUS_HIGH              = 2;

    //Constant definations for signals related to driver specific personalization
    public static final short ENGLISH                               = 1;
    public static final short SPANISH                               = 2;
    public static final short FRENCH                                = 3;
    public static final short DRIVING_MODE_COMFORT                  = 1;
    public static final short DRIVING_MODE_AUTO                     = 2;
    public static final short DRIVING_MODE_SPORT                    = 3;
    public static final short DRIVING_MODE_ECO                      = 4;
    public static final short DRIVING_MODE_MANUAL                   = 5;
    public static final short GENERATED_VEHICLE_SOUNDMODE_NORMAL    = 1;
    public static final short GENERATED_VEHICLE_SOUNDMODE_QUIET     = 2;
    public static final short GENERATED_VEHICLE_SOUNDMODE_SPORTIVE  = 3;

    //Constant definations for signals related to Climate information
    public static final short RAIN_SENSOR_NORAIN                    = 0;
    public static final short RAIN_SENSOR_LEVEL1                    = 1;
    public static final short RAIN_SENSOR_LEVEL2                    = 2;
    public static final short RAIN_SENSOR_LEVEL3                    = 3;
    public static final short RAIN_SENSOR_LEVEL4                    = 4;
    public static final short RAIN_SENSOR_LEVEL5                    = 5;
    public static final short RAIN_SENSOR_LEVEL6                    = 6;
    public static final short RAIN_SENSOR_LEVEL7                    = 7;
    public static final short RAIN_SENSOR_LEVEL8                    = 8;
    public static final short RAIN_SENSOR_LEVEL9                    = 9;
    public static final short RAIN_SENSOR_HEAVIEST_RAIN             = 10;
    public static final short WIPER_OFF                             = 0;
    public static final short WIPER_ONCE                            = 1;
    public static final short WIPER_SLOWEST                         = 2;
    public static final short WIPER_SLOW                            = 3;
    public static final short WIPER_FAST                            = 4;
    public static final short WIPER_FASTEST                         = 5;
    public static final short WIPER_AUTO                            = 10;
    public static final short HVACFAN_DIRECTION_FRONT_PANEL         = 1;
    public static final short HVACFAN_DIRECTION_FLOOR_DUCT          = 2;
    public static final short HVACFAN_DIRECTION_FRONT_FLOOR         = 3;
    public static final short HVACFAN_DIRECTION_DEFROSTER_FLOOR     = 4;
}
