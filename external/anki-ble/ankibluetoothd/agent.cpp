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
#include "strlcpy.h"

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
  cbs.inbound_connection_cb = &Agent::StaticPeripheralInboundConnectionCallback;
  cbs.request_read_cb = &Agent::StaticPeripheralReadCallback;
  cbs.request_write_cb = &Agent::StaticPeripheralWriteCallback;
  cbs.indication_sent_cb = &Agent::StaticPeripheralIndicationSentCallback;
  cbs.congestion_cb = &Agent::StaticPeripheralCongestionCallback;
  cbs.scan_result_cb = &Agent::StaticCentralScanResultCallback;
  cbs.outbound_connection_cb = &Agent::StaticCentralOutboundConnectionCallback;
  cbs.notification_received_cb = &Agent::StaticCentralNotificationReceivedCallback;
  cbs.characteristic_read_cb = &Agent::StaticCentralCharacteristicReadCallback;
  cbs.descriptor_read_cb = &Agent::StaticCentralDescriptorReadCallback;
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

void Agent::PeripheralInboundConnectionCallback(int conn_id, int connected) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected && (conn_id != inbound_connection_id_)) {
    // This is false disconnect notice that is really for an outbound connection.
    return;
  }
  connected_ = (bool) connected;
  app_read_ccc_value_ = kCCCDefaultValue;
  app_read_value_.clear();
  notification_queue_.clear();
  congested_ = false;
  if (connected) {
    inbound_connection_id_ = conn_id;
    advertising_ = !BluetoothStack::StopAdvertisement();
  } else {
    inbound_connection_id_ = -1;
    OnInboundConnectionChange(conn_id, connected);
    if (inbound_handler_sock_fd_ != -1) {
      advertising_ = BluetoothStack::StartAdvertisement(ble_advertise_settings_);
    }
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
  }

  (void) BluetoothStack::SendResponse(conn_id, trans_id, attr_handle, error, offset, value);
}

void Agent::PeripheralWriteCallback(int conn_id, int trans_id, int attr_handle, int offset,
                                    bool need_rsp, const std::vector<uint8_t>& value) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> response;
  int error = kGattErrorNone;

  if (attr_handle == app_read_characteristic_handle_) {
    error = kGattErrorWriteNotPermitted;
  } else if (attr_handle == app_write_characteristic_handle_) {
    OnReceiveMessage(conn_id, Anki::kAppWriteCharacteristicUUID, value);
  } else if (attr_handle == app_read_ccc_descriptor_handle_) {
    error = kGattErrorCCCDImproperlyConfigured;
    if (value.size() == 2) {
      uint16_t ccc = ((value[1] << 8) | value[0]);
      if (ccc == kCCCDefaultValue || ccc == kCCCNotificationValue || ccc == kCCCIndicationValue) {
        uint16_t old_ccc = app_read_ccc_value_;
        app_read_ccc_value_ = ccc;
        if ((old_ccc == kCCCDefaultValue) && (app_read_ccc_value_ != kCCCDefaultValue)) {
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
  ScanResultRecord record(address, rssi, adv_data);

  if (!scan_filter_service_uuid_.empty() && !record.HasServiceUUID(scan_filter_service_uuid_)) {
    return;
  }

  std::vector<ScanResultRecord> records;
  records.push_back(record);
  OnScanResults(0, records);
}

void Agent::CentralOutboundConnectionCallback(const std::string& address,
                                              const int connected,
                                              const BluetoothGattConnection& connection)
{
  std::vector<GattDbRecord> records;
  for (auto & service : connection.services) {
    GattDbRecord service_record = {0};
    (void) strlcpy(service_record.uuid, service.uuid.c_str(), sizeof(service_record.uuid));
    service_record.type = GattDbRecordType::Service;
    service_record.handle = service.service_handle;
    service_record.start_handle = service.start_handle;
    service_record.end_handle = service.end_handle;
    records.push_back(service_record);
    for (auto & characteristic : service.characteristics) {
      GattDbRecord char_record = {0};
      (void) strlcpy(char_record.uuid, characteristic.uuid.c_str(), sizeof(char_record.uuid));
      char_record.type = GattDbRecordType::Characteristic;
      char_record.handle = characteristic.char_handle;
      char_record.properties = characteristic.properties;
      records.push_back(char_record);
      for (auto & descriptor : characteristic.descriptors) {
        GattDbRecord desc_record = {0};
        (void) strlcpy(desc_record.uuid, descriptor.uuid.c_str(), sizeof(desc_record.uuid));
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

void Agent::StaticCentralOutboundConnectionCallback(const std::string& address,
                                                    const int connected,
                                                    const BluetoothGattConnection& connection)
{
  if (sAgent) {
    sAgent->CentralOutboundConnectionCallback(address, connected, connection);
  }
}

void Agent::CentralNotificationReceivedCallback(const std::string& address,
                                                const int conn_id,
                                                const std::string& char_uuid,
                                                const std::vector<uint8_t>& value)
{
  OnReceiveMessage(conn_id, char_uuid, value);
}

void Agent::StaticCentralNotificationReceivedCallback(const std::string& address,
                                                      const int conn_id,
                                                      const std::string& char_uuid,
                                                      const std::vector<uint8_t>& value)
{
  if (sAgent) {
    sAgent->CentralNotificationReceivedCallback(address, conn_id, char_uuid, value);
  }
}

void Agent::CentralCharacteristicReadCallback(const std::string& address,
                                              const int conn_id,
                                              const int error,
                                              const std::string& char_uuid,
                                              const std::vector<uint8_t>& value)
{
  OnCharacteristicReadResult(conn_id, error, char_uuid, value);
}

void Agent::StaticCentralCharacteristicReadCallback(const std::string& address,
                                                    const int conn_id,
                                                    const int error,
                                                    const std::string& char_uuid,
                                                    const std::vector<uint8_t>& value)
{
  if (sAgent) {
    sAgent->CentralCharacteristicReadCallback(address, conn_id, error, char_uuid, value);
  }
}

void Agent::CentralDescriptorReadCallback(const std::string& address,
                                          const int conn_id,
                                          const int error,
                                          const std::string& char_uuid,
                                          const std::string& desc_uuid,
                                          const std::vector<uint8_t>& value)
{
  OnDescriptorReadResult(conn_id, error, char_uuid, desc_uuid, value);
}

void Agent::StaticCentralDescriptorReadCallback(const std::string& address,
                                                const int conn_id,
                                                const int error,
                                                const std::string& char_uuid,
                                                const std::string& desc_uuid,
                                                const std::vector<uint8_t>& value)
{
  if (sAgent) {
    sAgent->CentralDescriptorReadCallback(address, conn_id, error, char_uuid, desc_uuid, value);
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
  if (inbound_connection_id_ == connection_id) {
    if (connected_) {
      int characteristic_handle;
      if (AreCaseInsensitiveStringsEqual(characteristic_uuid,Anki::kAppReadCharacteristicUUID)) {
        characteristic_handle = app_read_characteristic_handle_;
      } else {
        return;
      }
      SendMessageToConnectedCentral(characteristic_handle, reliable ? 1 : 0, value);
    }
  } else {
    (void) BluetoothStack::WriteGattCharacteristic(connection_id, characteristic_uuid, reliable, value);
  }
}

void Agent::ReadCharacteristic(const int connection_id,
                               const std::string& characteristic_uuid)
{
  bool result = BluetoothStack::ReadGattCharacteristic(connection_id, characteristic_uuid);
  if (!result) {
    CentralCharacteristicReadCallback(std::string(""),
                                      connection_id,
                                      -1,
                                      characteristic_uuid,
                                      std::vector<uint8_t>());
  }
}

void Agent::ReadDescriptor(const int connection_id,
                           const std::string& characteristic_uuid,
                           const std::string& descriptor_uuid)
{
  bool result = BluetoothStack::ReadGattDescriptor(connection_id, characteristic_uuid, descriptor_uuid);
  if (!result) {
    CentralDescriptorReadCallback(std::string(""),
                                  connection_id,
                                  -1,
                                  characteristic_uuid,
                                  descriptor_uuid,
                                  std::vector<uint8_t>());
  }
}

void Agent::Disconnect(const int connection_id)
{
  if (connection_id != -1) {
    (void) BluetoothStack::DisconnectGattPeer(connection_id);
  }
}

void Agent::StartAdvertising(const int sockfd, const BLEAdvertiseSettings& settings)
{
  inbound_handler_sock_fd_ = sockfd;
  ble_advertise_settings_ = settings;
  if (BluetoothStack::StartGattService(&bluetooth_gatt_service_)) {
    logv("Anki BLE Peripheral Service is started.");
  } else {
    loge("Failed to start GATT service.  Cannot start advertising.");
    return;
  }
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

void Agent::StaticPeripheralInboundConnectionCallback(int conn_id, int connected) {
  sAgent->PeripheralInboundConnectionCallback(conn_id, connected);
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
  bluetooth_gatt_service_ = BluetoothGattService(Anki::kAnkiSingleMessageService_128_BIT_UUID);

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
    }
  }

  return true;
}

void Agent::StartScan(const int sockfd, const std::string& serviceUUID) {
  logv("Agent::StartScan(sockfd = %d, serviceUUID = '%s')",
       sockfd, serviceUUID.c_str());
  scan_filter_service_uuid_ = serviceUUID;
  scanning_ = BluetoothStack::SetScanning(true);
  if (scanning_) {
    scanning_ipc_clients_.insert(sockfd);
  }
}

void Agent::StopScan(const int sockfd) {
  scanning_ipc_clients_.erase(sockfd);
  if (scanning_ipc_clients_.empty()) {
    scanning_ = !BluetoothStack::SetScanning(false);
    scan_filter_service_uuid_.clear();
  }
}

void Agent::ConnectToPeripheral(const int sockfd, const std::string& address) {
  bool result = BluetoothStack::ConnectToBLEPeripheral(address, true);
  if (result) {
    auto search = outbound_connection_addresses_.find(sockfd);
    if (search == outbound_connection_addresses_.end()) {
      std::set<std::string> addresses;
      outbound_connection_addresses_[sockfd] = addresses;
      search = outbound_connection_addresses_.find(sockfd);
    }
    search->second.insert(address);
  } else {
    std::vector<GattDbRecord> records;
    OnOutboundConnectionChange(address, 0, 0, records);
  }
}

void Agent::DisconnectByAddress(const std::string& address) {
  BluetoothStack::DisconnectGattPeerByAddress(address);
}

void Agent::RequestConnectionParameterUpdate(const int sockfd,
                                             const std::string& address,
                                             int min_interval,
                                             int max_interval,
                                             int latency,
                                             int timeout)
{
  int status = BluetoothStack::RequestConnectionParameterUpdate(address,
                                                                min_interval,
                                                                max_interval,
                                                                latency,
                                                                timeout);
  OnRequestConnectionParameterUpdateResult(sockfd, address, status);
}

void Agent::SetAdapterName(const std::string& name) {
  bool result = BluetoothStack::SetAdapterName(name);
  if (!result) {
    loge("Error setting adapter name to %s", name.c_str());
  }
}


void Agent::OnPeerClose(const int sockfd) {
  logv("agent - OnPeerClose(sockfd = %d)", sockfd);
  IPCServer::OnPeerClose(sockfd);
  auto search = outbound_connection_addresses_.find(sockfd);
  if (search != outbound_connection_addresses_.end()) {
    for (auto const& address : search->second) {
      BluetoothStack::DisconnectGattPeerByAddress(address);
    }
    search->second.clear();
    outbound_connection_addresses_.erase(sockfd);
  }
  StopScan(sockfd);
  if (sockfd == inbound_handler_sock_fd_) {
    inbound_handler_sock_fd_ = -1;
    Disconnect(inbound_connection_id_);
    StopAdvertising();
    (void) BluetoothStack::StopGattService(&bluetooth_gatt_service_);
  }
}

void Agent::OnOutboundConnectionChange(const std::string& address,
                                       const int connected,
                                       const int connection_id,
                                       const std::vector<GattDbRecord>& records) {
  if (!connected) {
    for (auto it = outbound_connection_addresses_.begin();
         it != outbound_connection_addresses_.end();
         it++) {
      it->second.erase(address);
    }
  }
  IPCServer::OnOutboundConnectionChange(address, connected, connection_id, records);
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
