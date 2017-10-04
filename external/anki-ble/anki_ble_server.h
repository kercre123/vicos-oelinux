/**
 * File: anki_ble_server.h
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


#pragma once

#include <unordered_map>

#include <base/macros.h>
#include <base/memory/ref_counted.h>
#include <base/memory/weak_ptr.h>
#include <base/single_thread_task_runner.h>

#include <bluetooth/binder/IBluetooth.h>
#include <bluetooth/binder/IBluetoothGattServerCallback.h>
#include <bluetooth/gatt_identifier.h>

namespace Anki {

class BLEServer : public ipc::binder::BnBluetoothGattServerCallback {
 public:
  BLEServer(
      android::sp<ipc::binder::IBluetooth> bluetooth,
      scoped_refptr<base::SingleThreadTaskRunner> main_task_runner,
      bool advertise);
  ~BLEServer() override;

  // Set up the server and register the GATT services with the stack. This
  // initiates a set of asynchronous procedures. Invokes |callback|
  // asynchronously with the result of the operation.
  using RunCallback = std::function<void(bool success)>;
  bool Run(const RunCallback& callback);

 private:
  void ScheduleNextHeartBeat();
  void SendHeartBeat();
  void BuildHeartBeatMessage(std::vector<uint8_t>* out_value);

  // ipc::binder::IBluetoothGattServerCallback override:
  void OnServerRegistered(int status, int server_if) override;
  void OnServiceAdded(
      int status,
      const bluetooth::GattIdentifier& service_id) override;
  void OnCharacteristicReadRequest(
      const std::string& device_address,
      int request_id, int offset, bool is_long,
      const bluetooth::GattIdentifier& characteristic_id) override;
  void OnDescriptorReadRequest(
      const std::string& device_address,
      int request_id, int offset, bool is_long,
      const bluetooth::GattIdentifier& descriptor_id) override;
  void OnCharacteristicWriteRequest(
      const std::string& device_address,
      int request_id, int offset, bool is_prepare_write, bool need_response,
      const std::vector<uint8_t>& value,
      const bluetooth::GattIdentifier& characteristic_id) override;
  void OnDescriptorWriteRequest(
      const std::string& device_address,
      int request_id, int offset, bool is_prepare_write, bool need_response,
      const std::vector<uint8_t>& value,
      const bluetooth::GattIdentifier& descriptor_id) override;
  void OnExecuteWriteRequest(
      const std::string& device_address,
      int request_id, bool is_execute) override;
  void OnNotificationSent(const std::string& device_address,
                          int status) override;

  void HandleIncomingMessageFromCentral(const std::vector<uint8_t>& message);
  // Single mutex to protect all variables below.
  std::mutex mutex_;

  // This stores whether or not at least one remote device has written to the
  // CCC descriptor.
  bool peripheral_to_central_ccc_descriptor_written_;
  bool disconnect_ccc_descriptor_written_;

  // The IBluetooth and IBluetoothGattServer binders that we use to communicate
  // with the Bluetooth daemon's GATT server features.
  android::sp<ipc::binder::IBluetooth> bluetooth_;
  android::sp<ipc::binder::IBluetoothGattServer> gatt_;

  // ID assigned to us by the daemon to operate on our dedicated GATT server
  // instance.
  int server_if_;

  // Callback passed to Run(). We use this to tell main that all attributes have
  // been registered with the daemon.
  RunCallback pending_run_cb_;

  // Stores whether or not an outgoing notification is still pending. We use
  // this to throttle notifications so that we don't accidentally congest the
  // connection.
  std::unordered_map<std::string, bool> pending_notification_map_;

  uint8_t heart_beat_count_;

  std::vector<uint8_t> peripheral_to_central_value_;
  std::vector<uint8_t> central_to_peripheral_value_;
  std::vector<uint8_t> disconnect_value_;

  // The unique IDs that refer to each of the BLE Service GATT objects.
  // These are returned to us from the Bluetooth daemon as we populate the database.
  bluetooth::GattIdentifier service_id_;
  bluetooth::GattIdentifier central_to_peripheral_id_;
  bluetooth::GattIdentifier peripheral_to_central_id_;
  bluetooth::GattIdentifier peripheral_to_central_cccd_id_;
  bluetooth::GattIdentifier disconnect_id_;
  bluetooth::GattIdentifier disconnect_cccd_id_;

  // The daemon itself doesn't maintain a Client Characteristic Configuration
  // mapping, so we do it ourselves here.
  std::unordered_map<std::string, uint8_t> device_peripheral_to_central_ccc_map_;
  std::unordered_map<std::string, uint8_t> device_disconnect_ccc_map_;

  // Wether we should also start advertising
  bool advertise_;

  // libchrome task runner that we use to post
  // notifications on the main thread.
  scoped_refptr<base::SingleThreadTaskRunner> main_task_runner_;

  // We use this to pass weak_ptr's to base::Bind, which won't execute if the
  // BLEServer object gets deleted. This is a convenience utility from
  // libchrome and we use it here since base::TaskRunner uses base::Callback.
  // Note: This should remain the last member so that it'll be destroyed and
  // invalidate its weak pointers before any other members are destroyed.
  base::WeakPtrFactory<BLEServer> weak_ptr_factory_;

  DISALLOW_COPY_AND_ASSIGN(BLEServer);
};

}  // namespace Anki
