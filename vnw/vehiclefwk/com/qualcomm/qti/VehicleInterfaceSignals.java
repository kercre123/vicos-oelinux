/*
 *    Copyright (c) 2014-2015 Qualcomm Technologies, Inc. All Rights Reserved.
 *    Qualcomm Technologies Proprietary and Confidential.
 *
 */

package com.qualcomm.qti;

/**
 * VehicleInterfaceSignals provides definitions to be used by application
 * developer to set/get vehicle data. Please note that this class is work in
 * progress, and there might be few changes going forward.
 * For certain CAN messages with specific signals, an intent may be fired from VNW.
 * Please refer to VehicleSignalConstants for intent strings.
 * Please refer to vnwmappings.xml for configuring signals and intents.
 */
public final class VehicleInterfaceSignals {
    /**
    * Definition for signal Ids.
    * These IDs are unique and they help to identify specific signal.
    */
    public static final int VIM_ODOMETER_SIGNAL = 1000;

    public static final int VIM_TRANSMISSION_OIL_LIFE_LVL_SIGNAL = 1001;

    public static final int VIM_TRANSMISSION_OIL_TEMP_SIGNAL = 1002;

    public static final int VIM_BRAKE_FLUID_LVL_SIGNAL = 1003;

    public static final int VIM_WASHER_FLUID_LVL_SIGNAL = 1004;

    public static final int VIM_MALFUNCTION_INDICATOR_SIGNAL = 1005;

    public static final int VIM_BATTERY_VOLTAGE_SIGNAL = 1006;

    public static final int VIM_BATTERY_CURRENT_SIGNAL = 1007;

    public static final int VIM_TIRE_PRESSURE_FRONT_LEFT_SIGNAL = 1008;

    public static final int VIM_TIRE_PRESSURE_FRONT_RIGHT_SIGNAL = 1009;

    public static final int VIM_TIRE_PRESSURE_REAR_LEFT_SIGNAL = 1010;

    public static final int VIM_TIRE_PRESSURE_REAR_RIGHT_SIGNAL = 1011;

    public static final int VIM_SECURITY_ALERT_SIGNAL = 1012;

    public static final int VIM_PARKING_LIGHTS_SIGNAL = 1013;

    public static final int VIM_PARKING_BRAKES_SIGNAL = 1014;

    public static final int VIM_INTERIOR_TEMP_SIGNAL = 1015;

    public static final int VIM_EXTERIOR_TEMP_SIGNAL = 1016;

    public static final int VIM_EXTERIOR_BRGHTNESS_SIGNAL = 1017;

    public static final int VIM_RAIN_SENSOR_SIGNAL = 1018;

    public static final int VIM_MAIN_WSHIELD_WIPER_SIGNAL = 1019;

    public static final int VIM_REAR_WSHIELD_WIPER_SIGNAL = 1020;

    public static final int VIM_HVAC_FAN_DIRECTION_SIGNAL = 1021;

    public static final int VIM_HVAC_FAN_SPEED_SIGNAL = 1022;

    public static final int VIM_HVAC_FAN_TARGET_TEMP_SIGNAL = 1023;

    public static final int VIM_AIR_CONDITIONING_SIGNAL = 1024;

    public static final int VIM_AIR_CIRCULATION_SIGNAL = 1025;

    public static final int VIM_HEATER_SIGNAL = 1026;

    public static final int VIM_HEATER_STRNG_WHEEL_SIGNAL = 1027;

    public static final int VIM_HEATER_DRVNG_SEAT_SIGNAL = 1028;

    public static final int VIM_HEATER_PSNGR_SEAT_SIGNAL = 1029;

    public static final int VIM_DEFROST_WSHIELD_SIGNAL = 1030;

    public static final int VIM_DEFROST_FRONT_WINDOW_SIGNAL = 1031;

    public static final int VIM_DEFROST_REAR_WINDOW_SIGNAL = 1032;

    public static final int VIM_DEFROST_SIDE_MIRRORS_SIGNAL = 1033;

    public static final int VIM_DRVR_WINDOW_SIGNAL = 1034;

    public static final int VIM_PSNGR_WINDOW_SIGNAL = 1035;

    public static final int VIM_REAR_LEFT_WINDOW_SIGNAL = 1036;

    public static final int VIM_REAR_RIGHT_WINDOW_SIGNAL = 1037;

    public static final int VIM_SUN_ROOF_OPEN_SIGNAL = 1038;

    public static final int VIM_SUN_ROOF_TILT_SIGNAL = 1039;

    public static final int VIM_CONV_ROOF_SIGNAL = 1040;

    public static final int VIM_TRANSMISSION_GEAR_STATUS_SIGNAL = 1041;

    public static final int VIM_VEHICLE_POWER_MODE_SIGNAL = 1042;

    public static final int VIM_RMNG_FUEL_LVL_SIGNAL = 1043;

    public static final int VIM_RMNG_DRVNG_RANGE_SIGNAL = 1044;

    public static final int VIM_ENGN_OIL_RMNG_SIGNAL = 1045;

    public static final int VIM_ENGN_OIL_CHNG_SIGNAL = 1046;

    public static final int VIM_ENGN_OIL_TEMP_SIGNAL = 1047;

    public static final int VIM_COOLANT_LVL_SIGNAL = 1048;

    public static final int VIM_COOLANT_TEMP_SIGNAL = 1049;

    public static final int VIM_STRNG_WHEEL_ANGLE_SIGNAL = 1050;

    public static final int VIM_VEHICLE_WMI_SIGNAL = 1051;

    public static final int VIM_VEHICLE_VIN_SIGNAL = 1052;

    public static final int VIM_VEHICLE_TYPE_SIGNAL = 1053;

    public static final int VIM_VEHICLE_DOOR_TYPE_1ST_ROW_SIGNAL = 1054;

    public static final int VIM_VEHICLE_DOOR_TYPE_2ND_ROW_SIGNAL = 1055;

    public static final int VIM_VEHICLE_DOOR_TYPE_3RD_ROW_SIGNAL = 1056;

    public static final int VIM_VEHICLE_FUEL_TYPE_SIGNAL = 1057;

    public static final int VIM_VEHICLE_TRANS_GEAR_TYPE_SIGNAL = 1058;

    public static final int VIM_VEHICLE_WHEEL_INFO_RADIUS_SIGNAL = 1059;

    public static final int VIM_VEHICLE_WHEEL_INFO_TRACK_SIGNAL = 1060;

    public static final int VIM_SPEEDO_METER_SIGNAL = 1061;

    public static final int VIM_ENGINE_SPEED_SIGNAL = 1062;

    public static final int VIM_TRIP_METER_1_MILEAGE_SIGNAL = 1063;

    public static final int VIM_TRIP_METER_2_MILEAGE_SIGNAL = 1064;

    public static final int VIM_TRIP_METER_1_AVG_SPEED_SIGNAL = 1065;

    public static final int VIM_TRIP_METER_2_AVG_SPEED_SIGNAL = 1066;

    public static final int VIM_TRIP_METER_1_FUEL_CONSUMPTION_SIGNAL = 1067;

    public static final int VIM_TRIP_METER_2_FUEL_CONSUMPTION_SIGNAL = 1068;

    public static final int VIM_CRUISE_CONTROL_STATUS_SIGNAL = 1069;

    public static final int VIM_CRUISE_CONTROL_SPEED_SIGNAL = 1070;

    public static final int VIM_ANTI_LOCK_BRK_SYSTEM_SIGNAL = 1071;

    public static final int VIM_TRACTION_CONTROL_SYSTEM_SIGNAL = 1072;

    public static final int VIM_ELECTRONIC_STABILITY_CONTROL_SIGNAL = 1073;

    public static final int VIM_VEHICLE_TOP_SPEED_LIMIT_SIGNAL = 1074;

    public static final int VIM_AIR_BAG_STATUS_DRIVER_SIGNAL = 1075;

    public static final int VIM_AIR_BAG_STATUS_PSNGR_SIGNAL = 1076;

    public static final int VIM_AIR_BAG_STATUS_SIDE_SIGNAL = 1077;

    public static final int VIM_DOOR_OPEN_STATUS_DRIVER_SIGNAL = 1078;

    public static final int VIM_DOOR_OPEN_STATUS_PSNGR_SIGNAL = 1079;

    public static final int VIM_DOOR_OPEN_STATUS_REAR_LEFT_SIGNAL = 1080;

    public static final int VIM_DOOR_OPEN_STATUS_REAR_RIGHT_SIGNAL = 1081;

    public static final int VIM_DOOR_OPEN_STATUS_TRUNK_SIGNAL = 1082;

    public static final int VIM_FUEL_FILTER_CAP_STATUS_SIGNAL = 1083;

    public static final int VIM_DOOR_OPEN_STATUS_HOOD_SIGNAL = 1084;

    public static final int VIM_DOOR_LOCK_STATUS_DRIVER_SIGNAL = 1085;

    public static final int VIM_DOOR_LOCK_STATUS_PSNGR_SIGNAL = 1086;

    public static final int VIM_DOOR_LOCK_STATUS_REAR_LEFT_SIGNAL = 1087;

    public static final int VIM_DOOR_LOCK_STATUS_REAR_RIGHT_SIGNAL = 1088;

    public static final int VIM_CHILD_SAFETY_LOCK_REAR_LEFT_SIGNAL = 1089;

    public static final int VIM_CHILD_SAFETY_LOCK_REAR_RIGHT_SIGNAL = 1090;

    public static final int VIM_OCCUPANT_STATUS_DRIVER_SIGNAL = 1091;

    public static final int VIM_OCCUPANT_STATUS_PSNGR_SIGNAL = 1092;

    public static final int VIM_OCCUPANT_STATUS_REAR_LEFT_SIGNAL = 1093;

    public static final int VIM_OCCUPANT_STATUS_REAR_RIGHT_SIGNAL = 1094;

    public static final int VIM_OCCUPANT_STATUS_REAR_CENTER_SIGNAL = 1095;

    public static final int VIM_SEAT_BELT_STATUS_DRIVER_SIGNAL = 1096;

    public static final int VIM_SEAT_BELT_STATUS_PSNGR_SIGNAL = 1097;

    public static final int VIM_SEAT_BELT_STATUS_REAR_LEFT_SIGNAL = 1098;

    public static final int VIM_SEAT_BELT_STATUS_REAR_RIGHT_SIGNAL = 1099;

    public static final int VIM_SEAT_BELT_STATUS_REAR_CENTER_SIGNAL = 1100;

    public static final int VIM_WINDOW_LOCK_STATUS_DRIVER_SIGNAL = 1101;

    public static final int VIM_WINDOW_LOCK_STATUS_PSNGR_SIGNAL = 1102;

    public static final int VIM_WINDOW_LOCK_STATUS_REAR_LEFT_SIGNAL = 1103;

    public static final int VIM_WINDOW_LOCK_STATUS_REAR_RIGHT_SIGNAL = 1104;

    public static final int VIM_OBSTACLE_DISTANCE_SENSOR_STATUS_SIGNAL = 1105;

    public static final int VIM_OBSTACLE_DISTANCE_FRONT_CENTER_SIGNAL = 1106;

    public static final int VIM_OBSTACLE_DISTANCE_FRONT_LEFT_SIGNAL = 1107;

    public static final int VIM_OBSTACLE_DISTANCE_FRONT_RIGHT_SIGNAL = 1108;

    public static final int VIM_OBSTACLE_DISTANCE_REAR_CENTER_SIGNAL = 1109;

    public static final int VIM_OBSTACLE_DISTANCE_REAR_LEFT_SIGNAL = 1110;

    public static final int VIM_OBSTACLE_DISTANCE_REAR_RIGHT_SIGNAL = 1111;

    public static final int VIM_OBSTACLE_DISTANCE_MIDDLE_LEFT_SIGNAL = 1112;

    public static final int VIM_OBSTACLE_DISTANCE_MIDDLE_RIGHT_SIGNAL = 1113;

    public static final int VIM_FRONT_COLLISION_DETECTION_STATUS_SIGNAL = 1114;

    public static final int VIM_FRONT_COLLISION_DETECTION_DISTANCE_SIGNAL = 1115;

    public static final int VIM_FRONT_COLLISION_DETECTION_TIME_SIGNAL = 1116;

    public static final int VIM_HEAD_LIGHTS_SIGNAL = 1117;

    public static final int VIM_AUTOMATIC_HEAD_LIGHTS_SIGNAL = 1118;

    public static final int VIM_DYNAMIC_HIGH_BEAM_SIGNAL = 1119;

    public static final int VIM_HEAD_LIGHTS_HIGH_BEAM_SIGNAL = 1120;

    public static final int VIM_LEFT_TURN_LIGHT_SIGNAL = 1121;

    public static final int VIM_RIGHT_TURN_LIGHT_SIGNAL = 1122;

    public static final int VIM_BRAKE_LIGHT_SIGNAL = 1123;

    public static final int VIM_LIGHT_STATUS_FOG_FRONT_SIGNAL = 1124;

    public static final int VIM_LIGHT_STATUS_FOG_REAR_SIGNAL = 1125;

    public static final int VIM_HAZARD_LIGHT_STATUS_SIGNAL = 1126;

    public static final int VIM_PARKING_LIGHT_SIGNAL = 1127;

    public static final int VIM_INTR_LIGHT_DRIVER_SIGNAL = 1128;

    public static final int VIM_INTR_LIGHT_PSNGR_SIGNAL = 1129;

    public static final int VIM_INTR_LIGHT_CENTER_SIGNAL = 1130;

    public static final int VIM_INTR_LIGHT_3RD_ROW_SIGNAL = 1131;

    /**
    *When CAN message is received with this Signal,
    *depending on the value, platform's volume key event (up/down) will be injected.
    *Key injection is configured via vnwmappings.xml.
    */
    public static final int VIM_VOLUME_MAPPING_SIGNAL = 1132;

    public static final int VIM_FADE_LVL_SIGNAL = 1133;

    public static final int VIM_TREBLE_LVL_SIGNAL = 1134;

    public static final int VIM_BASS_LVL_SIGNAL = 1135;

    public static final int VIM_BRIGHTNESS_LVL_SIGNAL = 1136;

    public static final int VIM_REAR_LEFT_HEAD_PHONE_STATUS_SIGNAL = 1137;

    public static final int VIM_REAR_LEFT_HEAD_PHONE_VOLUME_SIGNAL = 1138;

    public static final int VIM_REAR_RIGHT_HEAD_PHONE_STATUS_SIGNAL = 1139;

    public static final int VIM_REAR_RIGHT_HEAD_PHONE_VOLUME_SIGNAL = 1140;

    public static final int VIM_RADIO_STATUS_SIGNAL = 1141;

    public static final int VIM_RADIO_STATION_SIGNAL = 1142;

    public static final int VIM_CD_STATUS_SIGNAL = 1143;

    public static final int VIM_DYNAMIC_ADAPT_SOUND_SIGNAL = 1144;

    public static final int VIM_DYNAMIC_ADAPT_DISPLAY_SIGNAL = 1145;

    public static final int VIM_BT_STATUS_SIGNAL = 1146;

    public static final int VIM_USB_MEDIA_STATUS_SIGNAL = 1147;

    public static final int VIM_AUDIO_SOURCE_STATUS_SIGNAL = 1148;

    public static final int VIM_KEY_SIGNAL = 1149;

    public static final int VIM_LANGUAGE_SIGNAL = 1150;

    public static final int VIM_MEASUREMENT_SYSTEM_SIGNAL = 1151;

    public static final int VIM_MEASUREMENT_FUEL_SIGNAL = 1152;

    public static final int VIM_MEASUREMENT_DISTANCE_SIGNAL = 1153;

    public static final int VIM_MEASUREMENT_SPEED_SIGNAL = 1154;

    public static final int VIM_MEASUREMENT_CONSUMPTION_SIGNAL = 1155;

    public static final int VIM_MIRROR_DRIVER_SIGNAL = 1156;

    public static final int VIM_MIRROR_PSNGR_SIGNAL = 1157;

    public static final int VIM_MIRROR_INTERIOR_SIGNAL = 1158;

    public static final int VIM_STEERING_WHEEL_POSN_SLIDE_SIGNAL = 1159;

    public static final int VIM_STEERING_WHEEL_POSN_TILT_SIGNAL = 1160;

    public static final int VIM_DRIVING_MODE_SIGNAL = 1161;

    public static final int VIM_DRIVER_SEAT_POSN_RECL_SEAT_BACK_SIGNAL = 1162;

    public static final int VIM_DRIVER_SEAT_POSN_SLIDE_SIGNAL = 1163;

    public static final int VIM_DRIVER_SEAT_POSN_CUSHION_HEIGHT_SIGNAL = 1164;

    public static final int VIM_DRIVER_SEAT_POSN_HEAD_REST_SIGNAL = 1165;

    public static final int VIM_DRIVER_SEAT_POSN_BACK_CUSHION_SIGNAL = 1166;

    public static final int VIM_DRIVER_SEAT_POSN_SIDE_CUSHION_SIGNAL = 1167;

    public static final int VIM_PSNGR_SEAT_POSN_RECL_SEAT_BACK_SIGNAL = 1168;

    public static final int VIM_PSNGR_SEAT_POSN_SLIDE_SIGNAL = 1169;

    public static final int VIM_PSNGR_SEAT_POSN_CUSHION_HEIGHT_SIGNAL = 1170;

    public static final int VIM_PSNGR_SEAT_POSN_HEAD_REST_SIGNAL = 1171;

    public static final int VIM_PSNGR_SEAT_POSN_BACK_CUSHION_SIGNAL = 1172;

    public static final int VIM_PSNGR_SEAT_POSN_SIDE_CUSHION_SIGNAL = 1173;

    public static final int VIM_DASH_BOARD_ILLUMINATION_SIGNAL = 1174;

    public static final int VIM_VEHICLE_SOUND_MODE_SIGNAL = 1175;

    public static final int VIM_HOME_MAPPING_SIGNAL = 1176;

    public static final int VIM_BACK_MAPPING_SIGNAL = 1177;

    public static final int VIM_MENU_MAPPING_SIGNAL = 1178;

    public static final int VIM_DPAD_DOWN_MAPPING_SIGNAL = 1179;

    public static final int VIM_DPAD_UP_MAPPING_SIGNAL = 1180;

    public static final int VIM_DPAD_CENTER_MAPPING_SIGNAL = 1181;

    public static final int VIM_DPAD_LEFT_MAPPING_SIGNAL = 1182;

    public static final int VIM_DPAD_RIGHT_MAPPING_SIGNAL = 1183;

    public static final int VIM_ENTER_MAPPING_SIGNAL = 1184;

    public static final int VIM_DRIVER_TEMP_SIGNAL = 1185;

    public static final int VIM_PASSENGER_TEMP_SIGNAL = 1186;

    public static final int VIM_REAR_TEMP_SIGNAL = 1187;

    public static final int VIM_VOLUME_UP_SIGNAL = 1188;
    public static final int VIM_VOLUME_DOWN_SIGNAL = 1189;
    public static final int VIM_MEDIA_NEXT_SIGNAL = 1190;
    public static final int VIM_MEDIA_PREV_SIGNAL = 1191;
    public static final int VIM_MEDIA_PAUSE_SIGNAL = 1192;
    public static final int VIM_MEDIA_PLAY_SIGNAL = 1193;
    public static final int VIM_MEDIA_FWD_SIGNAL = 1194;
    public static final int VIM_MEDIA_RWD_SIGNAL = 1195;
    public static final int VIM_IVI_POWER_SIGNAL = 1196;
    public static final int VIM_START_VOICE_CALL_SIGNAL = 1197;
    public static final int VIM_END_VOICE_CALL_SIGNAL = 1198;
    public static final int VIM_VOICE_ASSIST_SIGNAL = 1199;

    //If vendor needs to add more signals, the Ids should be >= VIM_VENDOR_SIGNAL_START and
    // <= VIM_VENDOR_SIGNAL_END;
    public static final int VIM_VENDOR_SIGNAL_START = 50000;
    public static final int VIM_VENDOR_SIGNAL_END = 65000;
}
