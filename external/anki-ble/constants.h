/**
 * File: constants.h
 *
 * Author: seichert
 * Created: 10/3/2017
 *
 * Description: Anki BLE Server constants
 *
 * Copyright: Anki, Inc. 2017
 *
 **/

#pragma once

#include <bluetooth/uuid.h>

// This header defines constants specific to the Anki BLE GATT Service.

namespace Anki {


const bluetooth::UUID kAnkiBLEService_128_BIT_UUID("D55E356B-59CC-4265-9D5F-3C61E9DFD70F");
const bluetooth::UUID kCentralToPeripheralCharacteristicUUID("7D2A4BDA-D29B-4152-B725-2491478C5CD7");
const bluetooth::UUID kPeripheralToCentralCharacteristicUUID("30619F2D-0F54-41BD-A65A-7588D8C85B45");
const bluetooth::UUID kDisconnectCharacteristicUUID("68F0FD05-B0D4-495A-8FD8-130179A137C0");
const bluetooth::UUID kCCCDescriptorUUID("2902");

const size_t kAnkiVictorMsgMaxSize=20;
const size_t kAnkiVictorMsgPayloadMaxSize=18;
const size_t kAnkiVictorMsgBaseSize=1;

const int kMinRssi = -100;
const int kMaxRssi = -55;

enum VictorMsg_Command : uint8_t {
  MSG_B2V_BTLE_DISCONNECT = 0x0D, // 13

  MSG_B2V_CORE_PING_REQUEST = 0x16, // 22
  MSG_V2B_CORE_PING_RESPONSE = 0x17, // 23
  MSG_B2V_HEARTBEAT = 0x18, // 24
  MSG_V2B_HEARTBEAT = 0x19, // 25

  MSG_B2V_WIFI_START = 0x1A, // 26
  MSG_B2V_WIFI_STOP = 0x1B, // 27
  MSG_B2V_WIFI_SET_CONFIG = 0x1C, // 28
  MSG_B2V_WIFI_SCAN = 0x1D, // 29
  MSG_V2B_WIFI_SCAN_RESULTS = 0x1E, // 30

  MSG_B2V_SSH_SET_AUTHORIZED_KEYS = 0x80, // 128

  MSG_B2V_DEV_PING_WITH_DATA_REQUEST = 0x91, // 145
  MSG_V2B_DEV_PING_WITH_DATA_RESPONSE = 0x92, // 146
  MSG_B2V_DEV_RESTART_ADBD = 0x93, // 147

  MSG_B2V_DEV_EXEC_CMD_LINE = 0x94, // 148
  MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE = 0x95, // 149

  MSG_B2V_MULTIPART_START = 0xF0, // 240
  MSG_B2V_MULTIPART_CONTINUE = 0xF1, // 241
  MSG_B2V_MULTIPART_FINAL = 0xF2, // 242

  MSG_V2B_MULTIPART_START = 0xF3, // 243
  MSG_V2B_MULTIPART_CONTINUE = 0xF4, // 244
  MSG_V2B_MULTIPART_FINAL = 0xF5, // 245
};

enum WiFiAuth : uint8_t {
  AUTH_NONE_OPEN = 0,
    AUTH_NONE_WEP = 1,
    AUTH_NONE_WEP_SHARED = 2,
    AUTH_IEEE8021X = 3,
    AUTH_WPA_PSK = 4,
    AUTH_WPA_EAP = 5,
    AUTH_WPA2_PSK = 6,
    AUTH_WPA2_EAP = 7
};

}  // Anki
