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

#include "anki_ble_uuids.h"
#include "ble_advertise_settings.h"
#include "btstack.h"
#include "btutils.h"
#include "gatt_constants.h"
#include "include_ev.h"
#include "log.h"

#include <algorithm>
#include <deque>
#include <mutex>

#include <string.h>

static Anki::BluetoothDaemon::Peripheral* sPeripheral = nullptr;

static std::mutex sMutex;

static Anki::BLEAdvertiseSettings sBLEAdvertiseSettings;
static BluetoothGattService sBluetoothGattService;
static int sCentralToPeripheralCharacteristicHandle = -1;
static int sPeripheralToCentralCharacteristicHandle = -1;
static int sCCCDescriptorHandle = -1;

static bool sAdvertising = false;
static bool sConnected = false;
static int sConnectionId = -1;
static bool sCongested = false;
static uint8_t sCCCValue = 0x00;

static std::vector<uint8_t> sPeripheralToCentralValue;

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
      sPeripheralToCentralValue = value;
    } else {
      logi("Failed to send notification");
      if (sConnected && sConnectionId) {
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

  sNotificationQueue.emplace_back(value);
  if (sNotificationQueue.size() == 1) {
    TransmitNextNotification();
  }
}

static void PeripheralConnectionCallback(int conn_id, int connected) {
  std::lock_guard<std::mutex> lock(sMutex);
  sConnected = (bool) connected;
  sCCCValue = 0;
  sPeripheralToCentralValue.clear();
  sNotificationQueue.clear();
  sCongested = false;
  if (connected) {
    sConnectionId = conn_id;
    sAdvertising = !StopAdvertisement();
  } else {
    sConnectionId = -1;
    sPeripheral->OnInboundConnectionChange(conn_id, connected);
    sAdvertising = StartAdvertisement(sBLEAdvertiseSettings);
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
    sPeripheral->OnReceiveMessage(conn_id, Anki::kCentralToPeripheralCharacteristicUUID, value);
  } else if (attr_handle == sCCCDescriptorHandle) {
    if (value.size() != 2 || value[1] != 0x00 || value[0] > 0x01) {
      error = kGattErrorCCCDImproperlyConfigured;
    } else {
      uint8_t previous_value = sCCCValue;
      sCCCValue = value[0];
      if (!previous_value && sCCCValue) {
        sPeripheral->OnInboundConnectionChange(conn_id, 1);
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
namespace Anki {
namespace BluetoothDaemon {
void Peripheral::OnNewIPCClient(const int sockfd)
{
  (void) sockfd; // not used for now
  OnPeripheralStateUpdate(sAdvertising, sConnectionId, sConnected, sCongested);
}

void Peripheral::OnSendMessage(const int connection_id,
                               const std::string& characteristic_uuid,
                               const bool reliable,
                               const std::vector<uint8_t>& value)
{
  if (sConnected
      && sConnectionId == connection_id
      && characteristic_uuid == Anki::kPeripheralToCentralCharacteristicUUID) {
    SendMessageToConnectedCentral(value);
  }
}

void Peripheral::OnDisconnect(const int connection_id)
{
  (void) DisconnectGattPeer(connection_id);
}

void Peripheral::OnStartAdvertising()
{
  sBLEAdvertiseSettings.GetAdvertisement().SetIncludeDeviceName(true);
  sBLEAdvertiseSettings.GetAdvertisement().SetIncludeTxPowerLevel(true);
  sBLEAdvertiseSettings.GetScanResponse().SetServiceUUID(Anki::kAnkiBLEService_128_BIT_UUID);
  sAdvertising = StartAdvertisement(sBLEAdvertiseSettings);
  if (sAdvertising) {
    logv("Started advertising Anki BLE peripheral service");
  } else {
    loge("Failed to start advertising Anki BLE peripheral service");
  }
}

void Peripheral::OnStopAdvertising()
{
  sAdvertising = !StopAdvertisement();
}

} // namespace BluetoothDaemon
} // namespace Anki

bool StartBLEPeripheral() {
  sPeripheral = new Anki::BluetoothDaemon::Peripheral(ev_default_loop(EVBACKEND_SELECT));
  sPeripheral->ListenForIPCConnections();

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
  return true;
}

bool StopBLEPeripheral() {
  return true;
}
