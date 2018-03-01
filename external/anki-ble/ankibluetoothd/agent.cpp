/**
 * File: agent.cpp
 *
 * Author: seichert
 * Created: 1/15/2018
 *
 * Description: BLE agent : peripheral and central
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "agent.h"

#include "anki_ble_uuids.h"
#include "ble_advertise_settings.h"
#include "btstack.h"
#include "btutils.h"
#include "gatt_constants.h"
#include "include_ev.h"
#include "log.h"
#include "stringutils.h"

#include <algorithm>
#include <deque>
#include <mutex>

#include <string.h>

static Anki::BluetoothDaemon::Agent* sAgent = nullptr;

namespace Anki {
namespace BluetoothDaemon {

Agent::Agent(struct ev_loop* loop)
    : IPCServer(loop)
{
  Anki::BluetoothStack::Callbacks cbs = {0};
  cbs.inbound_connection_cb = &Agent::StaticInboundConnectionCallback;
  cbs.request_read_cb = &Agent::StaticPeripheralReadCallback;
  cbs.request_write_cb = &Agent::StaticPeripheralWriteCallback;
  cbs.indication_sent_cb = &Agent::StaticPeripheralIndicationSentCallback;
  cbs.congestion_cb = &Agent::StaticPeripheralCongestionCallback;
  cbs.scan_result_cb = &Agent::StaticCentralScanResultCallback;
  cbs.outbound_connection_cb = &Agent::StaticOutboundConnectionCallback;
  cbs.notification_received_cb = &Agent::StaticNotificationReceivedCallback;
  Anki::BluetoothStack::SetCallbacks(&cbs);
}

void Agent::TransmitNextNotification()
{
  bool transmitted = false;
  while (!congested_ && !transmitted && !notification_queue_.empty()) {
    const Notification& notification = notification_queue_.front();
    uint16_t cccValue = kCCCDefaultValue;
    if (notification.characteristic_handle == app_read_characteristic_handle_) {
      cccValue = app_read_ccc_value_;
    } else if (notification.characteristic_handle == app_read_encrypted_characteristic_handle_) {
      cccValue = app_read_encrypted_ccc_value_;
    }
    int confirm = (cccValue == kCCCIndicationValue) ? notification.confirm : 0;
    if (connected_ && inbound_connection_id_ > 0 && (cccValue != kCCCDefaultValue)
        && BluetoothStack::SendGattIndication(notification.characteristic_handle,
                                              inbound_connection_id_,
                                              confirm,
                                              notification.value)) {
      transmitted = true;
      if (notification.characteristic_handle == app_read_characteristic_handle_) {
        app_read_value_ = notification.value;
      } else if (notification.characteristic_handle == app_read_encrypted_characteristic_handle_) {
        app_read_encrypted_value_ = notification.value;
      }
    } else {
      logi("Failed to send notification");
      if (connected_ && inbound_connection_id_) {
        connected_ = false;
        inbound_connection_id_ = -1;
        return;
      } else {
        notification_queue_.pop_front();
      }
    }
  }
}

void Agent::SendMessageToConnectedCentral(int characteristic_handle,
                                          int confirm,
                                          const std::vector<uint8_t>& value)
{
  if (!connected_) {
    return;
  }

  notification_queue_.push_back((Notification) {characteristic_handle, confirm, value});
  if (notification_queue_.size() == 1) {
    TransmitNextNotification();
  }
}

void Agent::InboundConnectionCallback(int conn_id, int connected) {
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = (bool) connected;
  app_read_ccc_value_ = kCCCDefaultValue;
  app_read_value_.clear();
  app_read_encrypted_ccc_value_ = kCCCDefaultValue;
  app_read_encrypted_value_.clear();
  notification_queue_.clear();
  congested_ = false;
  if (connected) {
    inbound_connection_id_ = conn_id;
    advertising_ = !BluetoothStack::StopAdvertisement();
  } else {
    inbound_connection_id_ = -1;
    OnInboundConnectionChange(conn_id, connected);
    advertising_ = BluetoothStack::StartAdvertisement(ble_advertise_settings_);
  }
}

void Agent::PeripheralReadCallback(int conn_id, int trans_id, int attr_handle, int offset) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> dummy;
  std::vector<uint8_t>& value = dummy;
  int error = kGattErrorReadNotPermitted;

  if (attr_handle == app_read_characteristic_handle_) {
    if (offset != 0) {
      error = kGattErrorInvalidOffset;
    } else {
      value = app_read_value_;
      error = kGattErrorNone;
    }
  } else if (attr_handle == app_read_encrypted_characteristic_handle_) {
    if (offset != 0) {
      error = kGattErrorInvalidOffset;
    } else {
      value = app_read_encrypted_value_;
      error = kGattErrorNone;
    }
  }

  (void) BluetoothStack::SendResponse(conn_id, trans_id, attr_handle, error, offset, value);
}

void Agent::PeripheralWriteCallback(int conn_id, int trans_id, int attr_handle, int offset,
                                    bool need_rsp, const std::vector<uint8_t>& value) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> response;
  int error = kGattErrorNone;

  if (attr_handle == app_read_characteristic_handle_
      || attr_handle == app_read_encrypted_characteristic_handle_) {
    error = kGattErrorWriteNotPermitted;
  } else if (attr_handle == app_write_characteristic_handle_) {
    OnReceiveMessage(conn_id, Anki::kAppWriteCharacteristicUUID, value);
  } else if (attr_handle == app_write_encrypted_characteristic_handle_) {
    OnReceiveMessage(conn_id, Anki::kAppWriteEncryptedCharacteristicUUID, value);
  } else if (attr_handle == app_read_ccc_descriptor_handle_) {
    error = kGattErrorCCCDImproperlyConfigured;
    if (value.size() == 2) {
      uint16_t ccc = ((value[1] << 8) | value[0]);
      if (ccc == kCCCDefaultValue || ccc == kCCCNotificationValue || ccc == kCCCIndicationValue) {
        uint16_t old_ccc = app_read_ccc_value_;
        app_read_ccc_value_ = ccc;
        if ((old_ccc == kCCCDefaultValue) && (app_read_ccc_value_ != kCCCDefaultValue)
            && (app_read_encrypted_ccc_value_ != kCCCDefaultValue)) {
          OnInboundConnectionChange(conn_id, 1);
        }
        error = kGattErrorNone;
      }
    }
  } else if (attr_handle == app_read_encrypted_ccc_descriptor_handle_) {
    error = kGattErrorCCCDImproperlyConfigured;
    if (value.size() == 2) {
      uint16_t ccc = ((value[1] << 8) | value[0]);
      if (ccc == kCCCDefaultValue || ccc == kCCCNotificationValue || ccc == kCCCIndicationValue) {
        uint16_t old_ccc = app_read_encrypted_ccc_value_;
        app_read_encrypted_ccc_value_ = ccc;
        if ((old_ccc == kCCCDefaultValue) && (app_read_ccc_value_ != kCCCDefaultValue)
            && (app_read_encrypted_ccc_value_ != kCCCDefaultValue)) {
          OnInboundConnectionChange(conn_id, 1);
        }
        error = kGattErrorNone;
      }
    }
  }

  if (need_rsp) {
    (void) BluetoothStack::SendResponse(conn_id, trans_id, attr_handle, error, offset, response);
  }
}

void Agent::PeripheralIndicationSentCallback(int conn_id, int status) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (conn_id == inbound_connection_id_) {
    if (status == kGattErrorCongested || status == kGattErrorNone) {
      if (status == kGattErrorCongested) {
        congested_ = true;
      }
      notification_queue_.pop_front();
    } else {
      logw("Indication failed to send. Will Retry. error = %s",
           bt_gatt_error_to_string(status).c_str());
    }
    TransmitNextNotification();
  }
}

void Agent::PeripheralCongestionCallback(int conn_id, bool congested) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (conn_id == inbound_connection_id_) {
    bool previous_congested_value = congested_;
    congested_ = congested;
    if (previous_congested_value && !congested_) {
      TransmitNextNotification();
    }
  }
}

void Agent::CentralScanResultCallback(const std::string& address,
                                      int rssi,
                                      const std::vector<uint8_t>& adv_data) {
  if (!scanning_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  bool passes_filter = scan_filter_service_uuid_.empty();
  ScanResultRecord record = {0};
  strncpy(record.address, address.c_str(), sizeof(record.address) - 1);
  record.rssi = rssi;

  auto it = adv_data.begin();
  while (it != adv_data.end()) {
    uint8_t length = *it;
    if (length == 0) {
      break;
    }
    it++;
    uint8_t type = *it;
    if (length > 1) {
      if (std::distance(adv_data.begin(), it) + length > adv_data.size()) {
        break;
      }
      std::vector<uint8_t> data(it + 1, it + length);
      switch(type) {
        case kADTypeFlags:
          // ignore for now
          break;
        case kADTypeCompleteListOf16bitServiceClassUUIDs:
          {
            // Check for the Device Information Service
            std::vector<std::string> uuids = bt_16bit_service_uuid_array_to_vector(data);
            for (auto const& u : uuids) {
              if (AreCaseInsensitiveStringsEqual(u, kDeviceInformationService_16_BIT_UUID)) {
                record.has_device_information_service = true;
              }
            }
          }
          break;
        case kADTypeCompleteListOf128bitServiceClassUUIDs:
          {
            // Check for Victor Cube Service and do filtering
            std::vector<std::string> uuids = bt_128bit_service_uuid_array_to_vector(data);
            for (auto const& u : uuids) {
              if (AreCaseInsensitiveStringsEqual(u, kCubeService_128_BIT_UUID)) {
                record.is_victor_cube = true;
              }
              if (AreCaseInsensitiveStringsEqual(u, scan_filter_service_uuid_)) {
                passes_filter = true;
              }
            }
          }
          break;
        case kADTypeCompleteLocalName:
          {
            strncpy(record.local_name,
                    (char *) data.data(),
                    std::min(sizeof(record.local_name) - 1, (size_t) length - 1));
          }
          break;
        case kADTypeManufacturerSpecificData:
          {
            record.manufacturer_data_len = length - 1;
            memcpy(record.manufacturer_data,
                   data.data(),
                   std::min(sizeof(record.manufacturer_data), (size_t) length - 1));
          }
          break;
        default:
          break;
      }
    }
    it += length;
  }
  if (!passes_filter) {
    return;
  }
  record.advertisement_length = adv_data.size();
  memcpy(record.advertisement_data,
         adv_data.data(),
         std::min(sizeof(record.advertisement_data), (size_t) record.advertisement_length));
  std::vector<ScanResultRecord> records;
  records.push_back(record);
  OnScanResults(0, records);
}

void Agent::OutboundConnectionCallback(const std::string& address,
                                       const int connected,
                                       const BluetoothGattConnection& connection)
{
  std::vector<GattDbRecord> records;
  for (auto & service : connection.services) {
    GattDbRecord service_record = {0};
    strncpy(service_record.uuid, service.uuid.c_str(), sizeof(service_record.uuid) - 1);
    service_record.type = GattDbRecordType::Service;
    service_record.handle = service.service_handle;
    service_record.start_handle = service.start_handle;
    service_record.end_handle = service.end_handle;
    records.push_back(service_record);
    for (auto & characteristic : service.characteristics) {
      GattDbRecord char_record = {0};
      strncpy(char_record.uuid, characteristic.uuid.c_str(), sizeof(char_record.uuid) - 1);
      char_record.type = GattDbRecordType::Characteristic;
      char_record.handle = characteristic.char_handle;
      char_record.properties = characteristic.properties;
      records.push_back(char_record);
      for (auto & descriptor : characteristic.descriptors) {
        GattDbRecord desc_record = {0};
        strncpy(desc_record.uuid, descriptor.uuid.c_str(), sizeof(desc_record.uuid) - 1);
        desc_record.type = GattDbRecordType::Descriptor;
        desc_record.handle = descriptor.desc_handle;
        records.push_back(desc_record);
      }
    }
  }
  OnOutboundConnectionChange(connection.address,
                             connected,
                             connection.conn_id,
                             records);
}

void Agent::StaticOutboundConnectionCallback(const std::string& address,
                                             const int connected,
                                             const BluetoothGattConnection& connection)
{
  if (sAgent) {
    sAgent->OutboundConnectionCallback(address, connected, connection);
  }
}

void Agent::NotificationReceivedCallback(const std::string& address,
                                         const int conn_id,
                                         const std::string& char_uuid,
                                         const std::vector<uint8_t>& value)
{
  OnReceiveMessage(conn_id, char_uuid, value);
}

void Agent::StaticNotificationReceivedCallback(const std::string& address,
                                               const int conn_id,
                                               const std::string& char_uuid,
                                               const std::vector<uint8_t>& value)
{
  if (sAgent) {
    sAgent->NotificationReceivedCallback(address, conn_id, char_uuid, value);
  }
}

void Agent::OnNewIPCClient(const int sockfd)
{
  (void) sockfd; // not used for now
  OnPeripheralStateUpdate(advertising_, inbound_connection_id_, connected_, congested_);
}

void Agent::SendMessage(const int connection_id,
                        const std::string& characteristic_uuid,
                        const bool reliable,
                        const std::vector<uint8_t>& value)
{
  if (connected_ && inbound_connection_id_ == connection_id) {
    int characteristic_handle;
    if (AreCaseInsensitiveStringsEqual(characteristic_uuid,Anki::kAppReadCharacteristicUUID)) {
      characteristic_handle = app_read_characteristic_handle_;
    } else if (AreCaseInsensitiveStringsEqual(characteristic_uuid,Anki::kAppReadEncryptedCharacteristicUUID)) {
      characteristic_handle = app_read_encrypted_characteristic_handle_;
    } else {
      return;
    }
    SendMessageToConnectedCentral(characteristic_handle, reliable ? 1 : 0, value);
  } else {
    (void) BluetoothStack::WriteGattCharacteristic(connection_id, characteristic_uuid, reliable, value);
  }
}

void Agent::Disconnect(const int connection_id)
{
  (void) BluetoothStack::DisconnectGattPeer(connection_id);
}

void Agent::StartAdvertising()
{
  ble_advertise_settings_.GetAdvertisement().SetIncludeDeviceName(true);
  ble_advertise_settings_.GetAdvertisement().SetIncludeTxPowerLevel(true);
  ble_advertise_settings_.GetScanResponse().SetServiceUUID(Anki::kAnkiBLEService_128_BIT_UUID);
  advertising_ = BluetoothStack::StartAdvertisement(ble_advertise_settings_);
  if (advertising_) {
    logv("Started advertising Anki BLE peripheral service");
  } else {
    loge("Failed to start advertising Anki BLE peripheral service");
  }
}

void Agent::StopAdvertising()
{
  advertising_ = !BluetoothStack::StopAdvertisement();
}

void Agent::StaticInboundConnectionCallback(int conn_id, int connected) {
  sAgent->InboundConnectionCallback(conn_id, connected);
}

void Agent::StaticPeripheralReadCallback(int conn_id, int trans_id, int attr_handle, int offset) {
  sAgent->PeripheralReadCallback(conn_id, trans_id, attr_handle, offset);
}

void Agent::StaticPeripheralWriteCallback(int conn_id, int trans_id, int attr_handle, int offset,
                                          bool need_rsp, const std::vector<uint8_t>& value) {
  sAgent->PeripheralWriteCallback(conn_id, trans_id, attr_handle, offset, need_rsp, value);
}

void Agent::StaticPeripheralIndicationSentCallback(int conn_id, int status) {
  sAgent->PeripheralIndicationSentCallback(conn_id, status);
}

void Agent::StaticPeripheralCongestionCallback(int conn_id, bool congested) {
  sAgent->PeripheralCongestionCallback(conn_id, congested);
}

void Agent::StaticCentralScanResultCallback(const std::string& address,
                                            int rssi,
                                            const std::vector<uint8_t>& adv_data) {
  sAgent->CentralScanResultCallback(address, rssi, adv_data);
}


bool Agent::StartPeripheral()
{
  bluetooth_gatt_service_ = BluetoothGattService(Anki::kAnkiBLEService_128_BIT_UUID);

  BluetoothGattCharacteristic
      appReadCharacteristic(Anki::kAppReadCharacteristicUUID,
                            (kGattCharacteristicPropNotify | kGattCharacteristicPropRead),
                            kGattPermRead);

  BluetoothGattDescriptor appReadCCCDescriptor(Anki::kCCCDescriptorUUID, (kGattPermRead | kGattPermWrite));
  appReadCharacteristic.descriptors.push_back(appReadCCCDescriptor);

  bluetooth_gatt_service_.characteristics.push_back(appReadCharacteristic);

  BluetoothGattCharacteristic
      appWriteCharacteristic(Anki::kAppWriteCharacteristicUUID,
                             (kGattCharacteristicPropWrite | kGattCharacteristicPropWriteNoResponse),
                             kGattPermWrite);

  bluetooth_gatt_service_.characteristics.push_back(appWriteCharacteristic);

  BluetoothGattCharacteristic
      appReadEncryptedCharacteristic(Anki::kAppReadEncryptedCharacteristicUUID,
                                     (kGattCharacteristicPropNotify | kGattCharacteristicPropRead),
                                     kGattPermRead);

  BluetoothGattDescriptor
      appReadEncryptedCCCDescriptor(Anki::kCCCDescriptorUUID, (kGattPermRead | kGattPermWrite));
  appReadEncryptedCharacteristic.descriptors.push_back(appReadEncryptedCCCDescriptor);

  bluetooth_gatt_service_.characteristics.push_back(appReadEncryptedCharacteristic);

  BluetoothGattCharacteristic
      appWriteEncryptedCharacteristic(Anki::kAppWriteEncryptedCharacteristicUUID,
                                      (kGattCharacteristicPropWrite | kGattCharacteristicPropWriteNoResponse),
                                      kGattPermWrite);
  bluetooth_gatt_service_.characteristics.push_back(appWriteEncryptedCharacteristic);


  if (!BluetoothStack::AddGattService(&bluetooth_gatt_service_)) {
    loge("Failed to add Anki BLE peripheral service");
    return false;
  }

  logv("Anki BLE peripheral service added.");

  for (auto const& cit : bluetooth_gatt_service_.characteristics) {
    if (AreCaseInsensitiveStringsEqual(cit.uuid, Anki::kAppWriteCharacteristicUUID)) {
      app_write_characteristic_handle_ = cit.char_handle;
    } else if (AreCaseInsensitiveStringsEqual(cit.uuid, Anki::kAppReadCharacteristicUUID)) {
      app_read_characteristic_handle_ = cit.char_handle;
      for (auto const& dit: cit.descriptors) {
        if (AreCaseInsensitiveStringsEqual(dit.uuid,Anki::kCCCDescriptorUUID)) {
          app_read_ccc_descriptor_handle_ = dit.desc_handle;
        }
      }
    } else if (AreCaseInsensitiveStringsEqual(cit.uuid, Anki::kAppWriteEncryptedCharacteristicUUID)) {
      app_write_encrypted_characteristic_handle_ = cit.char_handle;
    } else if (AreCaseInsensitiveStringsEqual(cit.uuid, Anki::kAppReadEncryptedCharacteristicUUID)) {
      app_read_encrypted_characteristic_handle_ = cit.char_handle;
      for (auto const& dit: cit.descriptors) {
        if (AreCaseInsensitiveStringsEqual(dit.uuid, Anki::kCCCDescriptorUUID)) {
          app_read_encrypted_ccc_descriptor_handle_ = dit.desc_handle;
        }
      }
    }
  }

  if (!BluetoothStack::StartGattService(&bluetooth_gatt_service_)) {
    loge("Failed to start Anki BLE peripheral service");
    return false;
  }

  logv("Anki BLE peripheral service started.");
  return true;
}

void Agent::StartScan(const std::string& serviceUUID) {
  scan_filter_service_uuid_ = serviceUUID;
  scanning_ = true;
  logv("Agent::StartScan(serviceUUID = '%s')", serviceUUID.c_str());
  (void) BluetoothStack::StartScan(true);
}

void Agent::StopScan() {
  scanning_ = false;
  (void) BluetoothStack::StartScan(false);
  scan_filter_service_uuid_.clear();
}

void Agent::ConnectToPeripheral(const std::string& address) {
  bool result = BluetoothStack::ConnectToBLEPeripheral(address, true);
  if (!result) {
    std::vector<GattDbRecord> records;
    OnOutboundConnectionChange(address, 0, 0, records);
  }
}

} // namespace BluetoothDaemon
} // namespace Anki

bool StartBLEAgent() {
  sAgent = new Anki::BluetoothDaemon::Agent(ev_default_loop(EVBACKEND_SELECT));
  sAgent->ListenForIPCConnections();
  return sAgent->StartPeripheral();
}

bool StopBLEAgent() {
  return true;
}
