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

enum VictorMsg_Command : uint8_t {
  MSG_B2V_BTLE_DISCONNECT = 0x0D, // 13

  MSG_B2V_CORE_PING_REQUEST = 0x16, // 22
  MSG_V2B_CORE_PING_RESPONSE = 0x17, // 23
  MSG_B2V_HEARTBEAT = 0x18, // 24
  MSG_V2B_HEARTBEAT = 0x19, // 25

  MSG_B2V_WIFI_START = 0x1A, // 26
  MSG_B2V_WIFI_STOP = 0x1B, // 27
  MSG_B2V_WIFI_SET_CONFIG = 0x1C, // 28

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

typedef struct {
  uint8_t size;   // 1
  uint8_t msgID;  // MSG_B2V_BTLE_DISCONNECT
} VictorMsg_Disconnect;

typedef struct {
  uint8_t size;   // 1
  uint8_t msgID;  // MSG_B2V_CORE_PING_REQUEST
} VictorMsg_PingRequest;

typedef struct {
  uint8_t size;   // 1
  uint8_t msgID;  // MSG_V2B_CORE_PING_RESPONSE
} VictorMsg_PingResponse;

typedef struct {
  uint8_t size;     // 2
  uint8_t msgID;    // MSG_B2V_HEARTBEAT or MSG_V2B_HEARTBEAT
  uint8_t counter;  // Looping counter from 0 to 255
} VictorMsg_HeartBeat;

typedef struct {
  uint8_t size;     // 1
  uint8_t msgID;    // MSG_B2V_WIFI_START
} VictorMsg_WifiStart;

typedef struct {
  uint8_t size;     // 1
  uint8_t msgID;    // MSG_B2V_WIFI_STOP
} VictorMsg_WifiStop;

typedef struct {
  uint8_t size;     // 1 to 19
  uint8_t msgID;    // MSG_B2V_DEV_PING_WITH_DATA_REQUEST
  uint8_t data[18];
} VictorMsg_PingWithDataRequest;

typedef struct {
  uint8_t size;     // 1 to 19
  uint8_t msgID;    // MSG_V2B_DEV_PING_WITH_DATA_RESPONSE
  uint8_t data[18];
} VictorMsg_PingWithDataResponse;

}  // Anki
