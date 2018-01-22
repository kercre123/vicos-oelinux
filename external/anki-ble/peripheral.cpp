/**
 * File: peripheral.cpp
 *
 * Author: seichert
 * Created: 1/15/2018
 *
 * Description: BLE peripheral
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "peripheral.h"

#include "btstack.h"
#include "btutils.h"
#include "constants.h"
#include "exec_command.h"
#include "fileutils.h"
#include "gatt_constants.h"
#include "log.h"
#include "ssh.h"
#include "taskExecutor.h"
#include "wifi.h"

#include <algorithm>
#include <deque>
#include <map>
#include <mutex>
#include <sstream>

#include <string.h>

static std::mutex sMutex;
static Anki::TaskExecutor sHeartBeatTaskExecutor;

static BluetoothGattService sBluetoothGattService;
static int sCentralToPeripheralCharacteristicHandle = -1;
static int sPeripheralToCentralCharacteristicHandle = -1;
static int sCCCDescriptorHandle = -1;

static bool sConnected = false;
static int sConnectionId = -1;
static bool sCongested = false;
static uint8_t sCCCValue = 0x00;
static uint8_t sHeartBeatCount = 1;

static std::vector<uint8_t> sMultipartMessage;

static std::vector<uint8_t> sPeripheralToCentralValue;
static std::vector<uint8_t> sCentralToPeripheralValue;

std::deque<std::vector<uint8_t>> sNotificationQueue;

void TransmitNextNotification()
{
  bool transmitted = false;
  while (!sCongested && !transmitted && !sNotificationQueue.empty()) {
    const std::vector<uint8_t>& value = sNotificationQueue.front();
    if (sConnected && sConnectionId > 0
        && SendGattIndication(sPeripheralToCentralCharacteristicHandle,
                              sConnectionId,
                              false,
                              value)) {
      transmitted = true;
    } else {
      logi("Failed to send notification");
      if (sConnected && sConnectionId) {
        Anki::CancelBackgroundCommands();
        sConnected = false;
        sConnectionId = -1;
        return;
      } else {
        sNotificationQueue.pop_front();
      }
    }
  }
}

static void SendMessageToConnectedCentral(const std::vector<uint8_t>& value)
{
  if (!sConnected) {
    return;
  }
  if (value.size() <= Anki::kAnkiVictorMsgMaxSize) {
    sNotificationQueue.emplace_back(value);
    if (sNotificationQueue.size() == 1) {
      TransmitNextNotification();
    }
  } else {
    // Fragment large message into multipart messages to queue
    size_t offset = 0;
    while (offset < value.size()) {
      std::vector<uint8_t> message;
      uint8_t msgID;
      uint8_t msgSize = Anki::kAnkiVictorMsgBaseSize;
      if (offset == 0) {
        msgID = Anki::VictorMsg_Command::MSG_V2B_MULTIPART_START;
        msgSize += Anki::kAnkiVictorMsgPayloadMaxSize;
      } else if (value.size() - offset > Anki::kAnkiVictorMsgPayloadMaxSize) {
        msgID = Anki::VictorMsg_Command::MSG_V2B_MULTIPART_CONTINUE;
        msgSize += Anki::kAnkiVictorMsgPayloadMaxSize;
      } else {
        msgID = Anki::VictorMsg_Command::MSG_V2B_MULTIPART_FINAL;
        msgSize += (value.size() - offset);
      }
      message.push_back(msgSize);
      message.push_back(msgID);
      std::copy(value.begin() + offset, value.begin() + offset + msgSize - 1, std::back_inserter(message));
      SendMessageToConnectedCentral(message);
      offset += (msgSize - 1);
    }
  }
}

static void SendMessageToConnectedCentral(uint8_t msgID, const std::vector<uint8_t>& value)
{
  if (!sConnected) {
    return;
  }
  uint8_t msgSize = std::min(static_cast<unsigned int>(std::numeric_limits<uint8_t>::max()), value.size() + 1);
  std::vector<uint8_t> msg {msgSize, msgID};
  msg.insert(std::end(msg), std::begin(value), std::end(value));
  SendMessageToConnectedCentral(msg);
}

static void SendMessageToConnectedCentral(uint8_t msgID, const std::string& str)
{
  if (!sConnected) {
    return;
  }
  std::vector<uint8_t> value(str.begin(), str.end());
  value.push_back(0);
  SendMessageToConnectedCentral(msgID, value);
}

static void ScheduleNextHeartBeat();
static void SendHeartBeat() {
  std::lock_guard<std::mutex> lock(sMutex);
  if (!sConnected) {
    return;
  }

  if (sConnectionId < 0) {
    return;
  }

  if (!sCCCValue) {
    return;
  }


  std::vector<uint8_t> value;
  value.push_back(sHeartBeatCount++);
  SendMessageToConnectedCentral(Anki::VictorMsg_Command::MSG_V2B_HEARTBEAT, value);

  ScheduleNextHeartBeat();
}

static void ScheduleNextHeartBeat() {
  auto when = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  sHeartBeatTaskExecutor.WakeAfter(std::bind(SendHeartBeat), when);
};

static void SendOutputToConnectedCentral(int rc, const std::string& output) {
  if (!sConnected) {
    return;
  }
  std::ostringstream oss;
  if (rc) {
    oss << "Error: " << rc << std::endl;
  }
  oss << output;
  SendMessageToConnectedCentral(Anki::VictorMsg_Command::MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE, oss.str());
}

static void ExecCommandInBackgroundAndSendOutputToCentral(const std::vector<std::string>& args)
{
  if (!sConnected) {
    return;
  }
  if (args.empty()) {
    SendMessageToConnectedCentral(Anki::VictorMsg_Command::MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE,
                                  "Invalid argument: args in empty");
    return;
  }
  Anki::ExecCommandInBackground(args, SendOutputToConnectedCentral);
}

static void HandleIncomingMessageFromCentral(const std::vector<uint8_t>& message)
{
  if (message.size() < 2) {
    return;
  }

  uint8_t size = message[0];
  uint8_t msgID = message[1];

  switch (msgID) {
  case Anki::VictorMsg_Command::MSG_B2V_BTLE_DISCONNECT:
    logi("Received request to disconnect");
    DisconnectGattPeer(sConnectionId);
    break;
  case Anki::VictorMsg_Command::MSG_B2V_CORE_PING_REQUEST:
    logi("Received ping request");
    SendMessageToConnectedCentral({0x01, Anki::VictorMsg_Command::MSG_V2B_CORE_PING_RESPONSE});
    break;
  case Anki::VictorMsg_Command::MSG_B2V_HEARTBEAT:
    logi("Received heartbeat");
    // Nothing to do here, we already send a periodic heartbeat of our own back to the central
    break;
  case Anki::VictorMsg_Command::MSG_B2V_WIFI_START:
    logi("Received WiFi start");
    Anki::EnableWiFiInterface(true, SendOutputToConnectedCentral);
    break;
  case Anki::VictorMsg_Command::MSG_B2V_WIFI_STOP:
    logi("Received WiFi stop");
    Anki::EnableWiFiInterface(false, SendOutputToConnectedCentral);
    break;
  case Anki::VictorMsg_Command::MSG_B2V_WIFI_SCAN:
    {
      logi("Received WiFi scan");
      std::vector<Anki::WiFiScanResult> results = Anki::ScanForWiFiAccessPoints();
      std::vector<uint8_t> packed_results = Anki::PackWiFiScanResults(results);
      SendMessageToConnectedCentral(Anki::VictorMsg_Command::MSG_V2B_WIFI_SCAN_RESULTS,
                                    packed_results);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_WIFI_SET_CONFIG:
    {
      logi("Receive WiFi Set Config");
      std::vector<uint8_t> packedWiFiConfig(message.begin() + 2, message.end());
      std::map<std::string, std::string> networks = Anki::UnPackWiFiConfig(packedWiFiConfig);
      Anki::SetWiFiConfig(networks, SendOutputToConnectedCentral);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_SSH_SET_AUTHORIZED_KEYS:
    {
      logi("Receive SSH authorized keys");
      // Payload is text for the .ssh/authorized_keys file
      std::string keys(message.begin() + 2, message.end());
      int rc = Anki::SetSSHAuthorizedKeys(keys);
      std::string output;
      if (rc) {
        output = "Failed to set SSH authorized keys";
      } else {
        output = "SSH authorized keys set";
      }
      SendOutputToConnectedCentral(rc, output);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_DEV_PING_WITH_DATA_REQUEST:
    {
      logi("Received ping with data request");
      std::vector<uint8_t> data = message;
      data[1] = Anki::VictorMsg_Command::MSG_V2B_DEV_PING_WITH_DATA_RESPONSE;
      SendMessageToConnectedCentral(data);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_DEV_RESTART_ADBD:
    logi("Received restart adbd request");
    ExecCommandInBackgroundAndSendOutputToCentral({"/etc/initscripts/adbd", "stop"});
    ExecCommandInBackgroundAndSendOutputToCentral({"/etc/initscripts/adbd", "start"});
    break;
  case Anki::VictorMsg_Command::MSG_B2V_DEV_EXEC_CMD_LINE:
    {
      logi("Received command line to execute");
      std::vector<std::string> args;
      for (auto it = message.begin() + 2; it != message.end(); ) {
        auto terminator = std::find(it, message.end(), 0);
        if (terminator == message.end()) {
          break;
        }
        std::string arg(it, terminator);
        args.push_back(arg);
        it = terminator + 1;
      }
      ExecCommandInBackgroundAndSendOutputToCentral(args);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_MULTIPART_START:
    sMultipartMessage.clear();
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(sMultipartMessage));
    break;
  case Anki::VictorMsg_Command::MSG_B2V_MULTIPART_CONTINUE:
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(sMultipartMessage));
    break;
  case Anki::VictorMsg_Command::MSG_B2V_MULTIPART_FINAL:
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(sMultipartMessage));
    HandleIncomingMessageFromCentral(sMultipartMessage);
    break;
  default:
    break;
  }

}

static void PeripheralConnectionCallback(int conn_id, int connected) {
  std::lock_guard<std::mutex> lock(sMutex);
  sConnected = (bool) connected;
  sCCCValue = 0;
  sHeartBeatCount = 1;
  sPeripheralToCentralValue.clear();
  sCentralToPeripheralValue.clear();
  sNotificationQueue.clear();
  sCongested = false;
  Anki::CancelBackgroundCommands();
  if (connected) {
    sConnectionId = conn_id;
    StopAdvertisement();
  } else {
    sConnectionId = -1;
    StartAdvertisement(sBluetoothGattService.uuid);
  }
}

static void PeripheralReadCallback(int conn_id, int trans_id, int attr_handle, int offset) {
  std::lock_guard<std::mutex> lock(sMutex);
  std::vector<uint8_t> dummy;
  std::vector<uint8_t>& value = dummy;
  int error = kGattErrorReadNotPermitted;

  if (attr_handle == sPeripheralToCentralCharacteristicHandle) {
    if (offset != 0) {
      error = kGattErrorInvalidOffset;
    } else {
      value = sPeripheralToCentralValue;
      error = kGattErrorNone;
    }
  }

  (void) SendResponse(conn_id, trans_id, attr_handle, error, offset, value);
}

static void PeripheralWriteCallback(int conn_id, int trans_id, int attr_handle, int offset,
                                    bool need_rsp, const std::vector<uint8_t>& value) {
  std::lock_guard<std::mutex> lock(sMutex);
  std::vector<uint8_t> response;
  int error = kGattErrorNone;

  if (attr_handle == sPeripheralToCentralCharacteristicHandle) {
    error = kGattErrorWriteNotPermitted;
  } else if (attr_handle == sCentralToPeripheralCharacteristicHandle) {
    sCentralToPeripheralValue = value;
    HandleIncomingMessageFromCentral(sCentralToPeripheralValue);
  } else if (attr_handle == sCCCDescriptorHandle) {
    if (value.size() != 2 || value[1] != 0x00 || value[0] > 0x01) {
      error = kGattErrorCCCDImproperlyConfigured;
    } else {
      uint8_t previous_value = sCCCValue;
      sCCCValue = value[0];
      if (!previous_value && sCCCValue) {
        ScheduleNextHeartBeat();
      }
    }
  }

  if (need_rsp) {
    (void) SendResponse(conn_id, trans_id, attr_handle, error, offset, response);
  }
}

static void PeripheralIndicationSentCallback(int conn_id, int status) {
  std::lock_guard<std::mutex> lock(sMutex);
  if (conn_id == sConnectionId) {
    if (status == kGattErrorCongested || status == kGattErrorNone) {
      if (status == kGattErrorCongested) {
        sCongested = true;
      }
      sNotificationQueue.pop_front();
    } else {
      logw("Indication failed to send. Will Retry. error = %s",
           bt_gatt_error_to_string(status).c_str());
    }
    TransmitNextNotification();
  }
}

static void PeripheralCongestionCallback(int conn_id, bool congested) {
  std::lock_guard<std::mutex> lock(sMutex);
  if (conn_id == sConnectionId) {
    bool previous_congested_value = sCongested;
    sCongested = congested;
    if (previous_congested_value && !sCongested) {
      TransmitNextNotification();
    }
  }
}

bool StartBLEPeripheral() {
  sBluetoothGattService.uuid = Anki::kAnkiBLEService_128_BIT_UUID;
  sBluetoothGattService.connection_cb = PeripheralConnectionCallback;
  sBluetoothGattService.request_read_cb = PeripheralReadCallback;
  sBluetoothGattService.request_write_cb = PeripheralWriteCallback;
  sBluetoothGattService.indication_sent_cb = PeripheralIndicationSentCallback;
  sBluetoothGattService.congestion_cb = PeripheralCongestionCallback;
  sBluetoothGattService.service_handle = -1;

  BluetoothGattCharacteristic peripheralToCentralCharacteristic;
  peripheralToCentralCharacteristic.uuid = Anki::kPeripheralToCentralCharacteristicUUID;
  peripheralToCentralCharacteristic.properties =
      (kGattCharacteristicPropNotify | kGattCharacteristicPropRead);
  peripheralToCentralCharacteristic.permissions = kGattPermRead;
  peripheralToCentralCharacteristic.char_handle = -1;

  BluetoothGattDescriptor cccDescriptor;
  cccDescriptor.uuid = Anki::kCCCDescriptorUUID;
  cccDescriptor.permissions = (kGattPermRead | kGattPermWrite);
  cccDescriptor.desc_handle = -1;
  peripheralToCentralCharacteristic.descriptors.push_back(cccDescriptor);

  sBluetoothGattService.characteristics.push_back(peripheralToCentralCharacteristic);

  BluetoothGattCharacteristic centralToPeripheralCharacteristic;
  centralToPeripheralCharacteristic.uuid = Anki::kCentralToPeripheralCharacteristicUUID;
  centralToPeripheralCharacteristic.properties =
      (kGattCharacteristicPropWrite | kGattCharacteristicPropWriteNoResponse);
  centralToPeripheralCharacteristic.permissions = kGattPermWrite;
  centralToPeripheralCharacteristic.char_handle = -1;
  sBluetoothGattService.characteristics.push_back(centralToPeripheralCharacteristic);


  if (!AddGattService(&sBluetoothGattService)) {
    loge("Failed to add Anki BLE peripheral service");
    return false;
  }

  logv("Anki BLE peripheral service added.");

  for (auto const& cit : sBluetoothGattService.characteristics) {
    if (cit.uuid == Anki::kCentralToPeripheralCharacteristicUUID) {
      sCentralToPeripheralCharacteristicHandle = cit.char_handle;
    } else if (cit.uuid == Anki::kPeripheralToCentralCharacteristicUUID) {
      sPeripheralToCentralCharacteristicHandle = cit.char_handle;
    }
    for (auto const& dit: cit.descriptors) {
      if (dit.uuid == Anki::kCCCDescriptorUUID) {
        sCCCDescriptorHandle = dit.desc_handle;
      }
    }
  }

  if (!StartGattService(&sBluetoothGattService)) {
    loge("Failed to start Anki BLE peripheral service");
    return false;
  }

  logv("Anki BLE peripheral service started.");

  if (!StartAdvertisement(sBluetoothGattService.uuid)) {
    loge("Failed to start advertising Anki BLE peripheral service");
    return false;
  }

  logv("Started advertising Anki BLE peripheral service");

  return true;
}

bool StopBLEPeripheral() {
  return true;
}
