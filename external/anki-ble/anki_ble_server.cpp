/**
 * File: anki_ble_server.cpp
 *
 * Author: seichert
 * Created: 10/3/2017
 *
 * Description: Anki BLE Server
 *
 * Copyright: Anki, Inc. 2017
 *
 **/

/**
 * This code is loosely based on the HeartRateServer example from Google's Fluoride project.
 *
 * That code is licensed under the Apache License, Version 2.0.  The license is below for reference.
 **/

//
//  Copyright (C) 2015 Google, Inc.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at:
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "anki_ble_server.h"

#include <base/bind.h>
#include <base/location.h>
#include <base/logging.h>
#include <base/rand_util.h>

#include <bluetooth/low_energy_constants.h>

#include <cutils/properties.h>

#include <sys/wait.h>
#include "constants.h"

namespace Anki {

class CLIBluetoothLowEnergyCallback
    : public ipc::binder::BnBluetoothLowEnergyCallback {
 public:
  CLIBluetoothLowEnergyCallback(android::sp<ipc::binder::IBluetooth> bt)
      : bt_(bt) {}

  // IBluetoothLowEnergyCallback overrides:
  void OnConnectionState(int /* status */,
                         int /* client_id */,
                         const char* /* address */,
                         bool /* connected */) override {}

  void OnMtuChanged(int /* status */,
                    const char * /* address */,
                    int /* mtu */) override {}

  void OnScanResult(const bluetooth::ScanResult& /* scan_result */) override {}

  void StartAdvertising() {
    /* Advertising data: 128-bit Service UUID: Anki BLE Service */
    std::vector<uint8_t> data;
    int uuid_size = kAnkiBLEService_128_BIT_UUID.GetShortestRepresentationSize();
    uint8_t type;
    if (uuid_size == bluetooth::UUID::kNumBytes128)
      type = bluetooth::kEIRTypeComplete128BitUUIDs;
    else if (uuid_size == bluetooth::UUID::kNumBytes32)
      type = bluetooth::kEIRTypeComplete32BitUUIDs;
    else if (uuid_size == bluetooth::UUID::kNumBytes16)
      type = bluetooth::kEIRTypeComplete16BitUUIDs;
    else
      NOTREACHED() << "Unexpected size: " << uuid_size;

    data.push_back(uuid_size + 1);
    data.push_back(type);

    auto uuid_bytes = kAnkiBLEService_128_BIT_UUID.GetFullLittleEndian();
    int index = (uuid_size == 16) ? 0 : 12;
    data.insert(data.end(), uuid_bytes.data() + index,
                uuid_bytes.data() + index + uuid_size);

    base::TimeDelta timeout;

    bluetooth::AdvertiseSettings settings(
        bluetooth::AdvertiseSettings::MODE_LOW_POWER,
        timeout,
        bluetooth::AdvertiseSettings::TX_POWER_LEVEL_MEDIUM,
        true);

    bluetooth::AdvertiseData adv_data;
    adv_data.set_include_device_name(true);
    adv_data.set_include_tx_power_level(true);

    bluetooth::AdvertiseData scan_rsp(data);

    bt_->GetLowEnergyInterface()->
        StartMultiAdvertising(client_id_, adv_data, scan_rsp, settings);
  }

  void StopAdvertising() {
    bt_->GetLowEnergyInterface()->StopMultiAdvertising(client_id_);
  }

  void OnClientRegistered(int status, int client_id){
    if (status != bluetooth::BLE_STATUS_SUCCESS) {
      LOG(ERROR) << "Failed to register BLE client, will not start advertising";
      return;
    }
    client_id_ = client_id;
    LOG(INFO) << "Registered BLE client with ID: " << client_id;

    StartAdvertising();
  }

  void OnMultiAdvertiseCallback(int /* status */, bool is_start,
      const bluetooth::AdvertiseSettings& /* settings */) {
    LOG(INFO) << "Advertising" << (is_start?" started":" stopped");
  };

 private:
  android::sp<ipc::binder::IBluetooth> bt_;
  int client_id_ = 0;
  DISALLOW_COPY_AND_ASSIGN(CLIBluetoothLowEnergyCallback);
};


BLEServer::BLEServer(
    android::sp<ipc::binder::IBluetooth> bluetooth,
    scoped_refptr<base::SingleThreadTaskRunner> main_task_runner,
    bool advertise)
  : bluetooth_(bluetooth),
      server_if_(-1),
      advertise_(advertise),
      main_task_runner_(main_task_runner),
      weak_ptr_factory_(this) {
  CHECK(bluetooth_.get());
  HandleDisconnect();
}

BLEServer::~BLEServer() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!gatt_.get() || server_if_ == -1)
    return;

  if (!android::IInterface::asBinder(gatt_.get())->isBinderAlive())
    return;

  // Manually unregister ourselves from the daemon. It's good practice to do
  // this, even though the daemon will automatically unregister us if this
  // process exits.
  gatt_->UnregisterServer(server_if_);
}

bool BLEServer::Run(const RunCallback& callback) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (pending_run_cb_) {
    LOG(ERROR) << "Already started";
    return false;
  }

  // Grab the IBluetoothGattServer binder from the Bluetooth daemon.
  gatt_ = bluetooth_->GetGattServerInterface();
  if (!gatt_.get()) {
    LOG(ERROR) << "Failed to obtain handle to IBluetoothGattServer interface";
    return false;
  }

  // Register this instance as a GATT server. If this call succeeds, we will
  // asynchronously receive a server ID via the OnServerRegistered callback.
  if (!gatt_->RegisterServer(this)) {
    LOG(ERROR) << "Failed to register with the server interface";
    return false;
  }

  pending_run_cb_ = callback;

  return true;
}

void BLEServer::ScheduleNextHeartBeat() {
  main_task_runner_->PostDelayedTask(
      FROM_HERE,
      base::Bind(&BLEServer::SendHeartBeat,
                 weak_ptr_factory_.GetWeakPtr()),
      base::TimeDelta::FromSeconds(10));
}

void BLEServer::SendHeartBeat() {
  std::lock_guard<std::mutex> lock(mutex_);

  // Only send a heartbeat if a central has connected to us
  if (connected_central_address_.empty()) {
    return;
  }

  // Only send a heartbeat if the CCC value is 1 (enabled)
  if (!peripheral_to_central_ccc_value_) {
    return;
  }

  std::vector<uint8_t> data;
  BuildHeartBeatMessage(&data);
  SendMessageToConnectedCentral(data);

  ScheduleNextHeartBeat();
}

void BLEServer::BuildHeartBeatMessage(
    std::vector<uint8_t>* out_value) {
  CHECK(out_value);  // Assert that |out_value| is not nullptr.

  out_value->clear();
  out_value->push_back(2); // message size
  out_value->push_back(VictorMsg_Command::MSG_V2B_HEARTBEAT);
  out_value->push_back(heart_beat_count_++);
}

void BLEServer::OnServerRegistered(int status, int server_if) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (status != bluetooth::BLE_STATUS_SUCCESS) {
    LOG(ERROR) << "Failed to register GATT server";
    pending_run_cb_(false);
    return;
  }

  // Registration succeeded. Store our ID, as we need it for GATT server
  // operations.
  server_if_ = server_if;

  LOG(INFO) << "Anki BLE server registered - server_if: " << server_if_;
  LOG(INFO) << "Populating attributes";

  // Start service declaration.
  std::unique_ptr<bluetooth::GattIdentifier> gatt_id;
  if (!gatt_->BeginServiceDeclaration(server_if_, true,
                                      kAnkiBLEService_128_BIT_UUID,
                                      &gatt_id)) {
    LOG(ERROR) << "Failed to begin service declaration";
    pending_run_cb_(false);
    return;
  }

  service_id_ = *gatt_id;

  // Add Peripheral to Central characteristic.
  if (!gatt_->AddCharacteristic(
      server_if_, kPeripheralToCentralCharacteristicUUID,
      bluetooth::kCharacteristicPropertyNotify | bluetooth::kCharacteristicPropertyRead,
      bluetooth::kAttributePermissionRead, &gatt_id)) {
    LOG(ERROR) << "Failed to add peripheral to central characteristic";
    pending_run_cb_(false);
    return;
  }

  peripheral_to_central_id_ = *gatt_id;

  // Add Client Characteristic Configuration descriptor for the Peripheral
  // to Central characteristic;
  if (!gatt_->AddDescriptor(
      server_if_, kCCCDescriptorUUID,
      bluetooth::kAttributePermissionRead|bluetooth::kAttributePermissionWrite,
      &gatt_id)) {
    LOG(ERROR) << "Failed to add CCC descriptor for Peripheral to Central characteristic";
    pending_run_cb_(false);
    return;
  }

  peripheral_to_central_cccd_id_ = *gatt_id;

  // Add Central to Peripheral characteristic
  if (!gatt_->AddCharacteristic(
      server_if_, kCentralToPeripheralCharacteristicUUID,
      bluetooth::kCharacteristicPropertyWrite | bluetooth::kCharacteristicPropertyWriteNoResponse,
      bluetooth::kAttributePermissionWrite,
      &gatt_id)) {
    LOG(ERROR) << "Failed to add central to peripheral characteristic";
    pending_run_cb_(false);
    return;
  }

  central_to_peripheral_id_ = *gatt_id;

  // Add Disconnect characteristic.
  if (!gatt_->AddCharacteristic(
      server_if_, kDisconnectCharacteristicUUID,
      bluetooth::kCharacteristicPropertyNotify | bluetooth::kCharacteristicPropertyRead,
      bluetooth::kAttributePermissionRead,
      &gatt_id)) {
    LOG(ERROR) << "Failed to add disconnect characteristic";
    pending_run_cb_(false);
    return;
  }

  disconnect_id_ = *gatt_id;
  // Add Client Characteristic Configuration descriptor for the Disconnect
  // characteristic;
  if (!gatt_->AddDescriptor(
      server_if_, kCCCDescriptorUUID,
      bluetooth::kAttributePermissionRead|bluetooth::kAttributePermissionWrite,
      &gatt_id)) {
    LOG(ERROR) << "Failed to add CCC descriptor for Disconnect characteristic";
    pending_run_cb_(false);
    return;
  }

  disconnect_cccd_id_ = *gatt_id;

  // End service declaration. We will be notified whether or not this succeeded
  // via the OnServiceAdded callback.
  if (!gatt_->EndServiceDeclaration(server_if_)) {
    LOG(ERROR) << "Failed to end service declaration";
    pending_run_cb_(false);
    return;
  }

  LOG(INFO) << "Initiated EndServiceDeclaration request";
}

void BLEServer::OnServiceAdded(
    int status,
    const bluetooth::GattIdentifier& service_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (status != bluetooth::BLE_STATUS_SUCCESS) {
    LOG(ERROR) << "Failed to add Anki BLE service";
    pending_run_cb_(false);
    return;
  }

  if (service_id != service_id_) {
    LOG(ERROR) << "Received callback for the wrong service ID";
    pending_run_cb_(false);
    return;
  }

  // EndServiceDeclaration succeeded! Our Anki BLE service is now discoverable
  // over GATT connections.

  LOG(INFO) << "Anki BLE service added";
  pending_run_cb_(true);

  if (advertise_) {
    auto ble = bluetooth_->GetLowEnergyInterface();
    if (!ble.get()) {
      LOG(ERROR) << "Failed to obtain handle to IBluetoothLowEnergy interface";
      return;
    }
    bool registered = ble->RegisterClient(new CLIBluetoothLowEnergyCallback(bluetooth_));
    LOG(INFO) << "RegisterClient result = " << registered;
  }

}

void BLEServer::OnCharacteristicReadRequest(
    const std::string& device_address,
    int request_id, int offset, bool /* is_long */,
    const bluetooth::GattIdentifier& characteristic_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Save the address as our connected central
  connected_central_address_ = device_address;

  // This is where we handle an incoming characteristic read. Only the
  // peripheral to central and disconnect characteristics are readable.
  CHECK((characteristic_id == peripheral_to_central_id_) || (characteristic_id == disconnect_id_));

  std::vector<uint8_t> dummy;
  std::vector<uint8_t>& value = dummy;
  bluetooth::GATTError error = bluetooth::GATT_ERROR_READ_NOT_PERMITTED;
  if (characteristic_id == peripheral_to_central_id_) {
    if (offset != 0) {
      error = bluetooth::GATT_ERROR_INVALID_OFFSET;
    } else {
      value = peripheral_to_central_value_;
      error = bluetooth::GATT_ERROR_NONE;
    }
  } else if (characteristic_id == disconnect_id_) {
    value = disconnect_value_;
  }

  gatt_->SendResponse(server_if_, device_address, request_id, error,
                      offset, value);
}

void BLEServer::OnDescriptorReadRequest(
    const std::string& device_address,
    int request_id, int offset, bool /* is_long */,
    const bluetooth::GattIdentifier& descriptor_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Save the address as our connected central
  connected_central_address_ = device_address;

  // This is where we handle an incoming characteristic descriptor read.
  if ((descriptor_id != peripheral_to_central_cccd_id_)
      && (descriptor_id != disconnect_cccd_id_)) {
    std::vector<uint8_t> value;
    gatt_->SendResponse(server_if_, device_address, request_id,
                        bluetooth::GATT_ERROR_ATTRIBUTE_NOT_FOUND,
                        offset, value);
    return;
  }

  // 16-bit value encoded as little-endian.
  uint8_t value_bytes[2];
  if (descriptor_id == peripheral_to_central_cccd_id_) {
    value_bytes[0] = peripheral_to_central_ccc_value_;
  } else if (descriptor_id == disconnect_cccd_id_) {
    value_bytes[0] = disconnect_ccc_value_;
  }
  value_bytes[1] = 0x00;

  std::vector<uint8_t> value;
  bluetooth::GATTError error = bluetooth::GATT_ERROR_NONE;
  if (offset > 2)
    error = bluetooth::GATT_ERROR_INVALID_OFFSET;
  else
    value.insert(value.begin(), value_bytes + offset, value_bytes + 2 - offset);

  gatt_->SendResponse(server_if_, device_address, request_id, error,
                      offset, value);
}

void BLEServer::OnCharacteristicWriteRequest(
    const std::string& device_address,
    int request_id, int offset, bool is_prepare_write, bool need_response,
    const std::vector<uint8_t>& value,
    const bluetooth::GattIdentifier& characteristic_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<uint8_t> dummy;

  // This is where we handle an incoming characteristic write. The Anki BLE
  // service doesn't really support prepared writes, so we just reject them to
  // keep things simple.
  if (is_prepare_write) {
    gatt_->SendResponse(server_if_, device_address, request_id,
                        bluetooth::GATT_ERROR_REQUEST_NOT_SUPPORTED,
                        offset, dummy);
    return;
  }

  // Central to Peripheral is the only writable characteristic.
  CHECK(characteristic_id == central_to_peripheral_id_);

  central_to_peripheral_value_ = value;

  HandleIncomingMessageFromCentral(central_to_peripheral_value_);

  if (!need_response)
    return;

  gatt_->SendResponse(server_if_, device_address, request_id,
                      bluetooth::GATT_ERROR_NONE, offset, dummy);
}

void BLEServer::EnableWiFiInterface(const bool enable)
{
  pid_t pid = fork();
  if (pid == 0) {
    if (enable) {
      char *const args[] = {(char* const) "/system/bin/ifconfig",
                            (char* const) "wlan0",
                            (char* const) "up", NULL};
      execve((char* const) "/system/bin/ifconfig", args, nullptr);
    } else {
      char *const args[] = {(char* const) "/system/bin/ifconfig",
                            (char* const) "wlan0",
                            (char* const) "down", NULL};
      execve((char* const) "/system/bin/ifconfig", args, nullptr);
    }
  } else if (pid > 0) {
    wait(NULL);
  } else {
    LOG(ERROR) << "EnableWiFiInterface fork() failed";
  }
}


void BLEServer::HandleIncomingMessageFromCentral(const std::vector<uint8_t>& message)
{
  if (message.size() < 2) {
    return;
  }

  uint8_t size = message[0];
  uint8_t msgID = message[1];

  switch (msgID) {
  case VictorMsg_Command::MSG_B2V_BTLE_DISCONNECT:
    LOG(INFO) << "Received request to disconnect";
    // TODO:  Not sure how to do this, may need to power down and power up adapter?
    break;
  case VictorMsg_Command::MSG_B2V_CORE_PING_REQUEST:
    LOG(INFO) << "Received ping request";
    SendMessageToConnectedCentral({0x01, VictorMsg_Command::MSG_V2B_CORE_PING_RESPONSE});
    break;
  case VictorMsg_Command::MSG_B2V_HEARTBEAT:
    LOG(INFO) << "Received heartbeat";
    // Nothing to do here, we already send a periodic heartbeat of our own back to the central
    break;
  case VictorMsg_Command::MSG_B2V_WIFI_START:
    LOG(INFO) << "Received WiFi start";
    EnableWiFiInterface(true);
    break;
  case VictorMsg_Command::MSG_B2V_WIFI_STOP:
    LOG(INFO) << "Received WiFi stop";
    EnableWiFiInterface(false);
    break;
  case VictorMsg_Command::MSG_B2V_DEV_PING_WITH_DATA_REQUEST:
    {
      LOG(INFO) << "Received ping with data request";
      std::vector<uint8_t> data = message;
      data[1] = VictorMsg_Command::MSG_V2B_DEV_PING_WITH_DATA_RESPONSE;
      SendMessageToConnectedCentral(data);
    }
    break;
  case VictorMsg_Command::MSG_B2V_DEV_RESTART_ADBD:
    LOG(INFO) << "Received restart adbd request";
    property_set("ctl.restart", "adbd");
    break;
  default:
    break;
  }
}

void BLEServer::HandleDisconnect() {
  connected_central_address_.clear();
  peripheral_to_central_ccc_value_ = 0;
  disconnect_ccc_value_ = 0;
  heart_beat_count_ = 0;
  peripheral_to_central_value_.clear();
  central_to_peripheral_value_.clear();
  disconnect_value_.clear();
  notifications_.clear();
}

void BLEServer::OnDescriptorWriteRequest(
    const std::string& device_address,
    int request_id, int offset, bool is_prepare_write, bool need_response,
    const std::vector<uint8_t>& value,
    const bluetooth::GattIdentifier& descriptor_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<uint8_t> dummy;
  // This is where we handle an incoming characteristic write. The Anki BLE
  // service doesn't really support prepared writes, so we just reject them to
  // keep things simple.
  if (is_prepare_write) {
    gatt_->SendResponse(server_if_, device_address, request_id,
                        bluetooth::GATT_ERROR_REQUEST_NOT_SUPPORTED,
                        offset, dummy);
    return;
  }

  // CCC is the only descriptor we have.
  CHECK((descriptor_id == peripheral_to_central_cccd_id_) || (descriptor_id == disconnect_cccd_id_));

  // CCC must contain 2 bytes for a 16-bit value in little-endian. The only
  // allowed values here are 0x0000 and 0x0001.
  if (value.size() != 2 || value[1] != 0x00 || value[0] > 0x01) {
    gatt_->SendResponse(server_if_, device_address, request_id,
                        bluetooth::GATT_ERROR_CCCD_IMPROPERLY_CONFIGURED,
                        offset, dummy);
    return;
  }

  // Save the address as our connected central
  connected_central_address_ = device_address;

  if (descriptor_id == peripheral_to_central_cccd_id_) {
    uint8_t previous_value = peripheral_to_central_ccc_value_;
    peripheral_to_central_ccc_value_ = value[0];
    LOG(INFO) << "Peripheral to Central CCC written - device: "
              << device_address << " value: " << (int)value[0];

    // Start sending heartbeats
    if (!previous_value && peripheral_to_central_ccc_value_) {
      ScheduleNextHeartBeat();
    }
  } else if (descriptor_id == disconnect_cccd_id_) {
    disconnect_ccc_value_ = value[0];
    LOG(INFO) << "Disconnect CCC written - device: "
              << device_address << " value: " << (int)value[0];
  }

  if (!need_response)
    return;

  gatt_->SendResponse(server_if_, device_address, request_id,
                      bluetooth::GATT_ERROR_NONE, offset, dummy);
}

void BLEServer::OnExecuteWriteRequest(
    const std::string& device_address,
    int request_id,
    bool /* is_execute */) {
  // We don't support Prepared Writes so, simply return Not Supported error.
  std::vector<uint8_t> dummy;
  gatt_->SendResponse(server_if_, device_address, request_id,
                      bluetooth::GATT_ERROR_REQUEST_NOT_SUPPORTED, 0, dummy);
}

void BLEServer::SendMessageToConnectedCentral(const std::vector<uint8_t>& value)
{
  QueueNotification(connected_central_address_, peripheral_to_central_id_, false, value);
}

void BLEServer::QueueNotification(const std::string& device_address,
                                  const bluetooth::GattIdentifier& characteristic_id,
                                  bool confirm,
                                  const std::vector<uint8_t>& value)
{
  notifications_.emplace_back(device_address, characteristic_id, confirm, value);
  if (notifications_.size() == 1) {
    TransmitNextNotification();
  }

}

void BLEServer::TransmitNextNotification()
{
  bool transmitted = false;
  while (!transmitted && !notifications_.empty()) {
    const NotificationInfo& notification = notifications_.front();
    if ((notification.GetDeviceAddress() == connected_central_address_)
        && (gatt_->SendNotification(server_if_,
                                    notification.GetDeviceAddress(),
                                    notification.GetCharacteristicId(),
                                    notification.GetConfirm(),
                                    notification.GetValue()))) {
      transmitted = true;
    } else {
      LOG(INFO) << "Failed to send notification to " << notification.GetDeviceAddress();
      if (connected_central_address_ == notification.GetDeviceAddress()) {
        HandleDisconnect();
        return;
      } else {
        notifications_.pop_front();
      }
    }
  }

}

void BLEServer::OnNotificationSent(
    const std::string& device_address, int status) {
  LOG(INFO) << "Notification was sent - device: " << device_address
            << " status: " << status;
  std::lock_guard<std::mutex> lock(mutex_);
  notifications_.pop_front();
  TransmitNextNotification();
}

}  // namespace Anki
