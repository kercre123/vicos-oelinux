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

#include <deque>
#include <unordered_map>
#include <map>

#include <base/macros.h>
#include <base/memory/ref_counted.h>
#include <base/memory/weak_ptr.h>
#include <base/single_thread_task_runner.h>
#include <base/threading/thread.h>

#include <bluetooth/binder/IBluetooth.h>
#include <bluetooth/binder/IBluetoothGattServerCallback.h>
#include <bluetooth/gatt_identifier.h>

namespace Anki {
class CLIBluetoothLowEnergyCallback;

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
  class NotificationInfo {
  public:
    NotificationInfo(const std::string& device_address,
                     const bluetooth::GattIdentifier& characteristic_id,
                     bool confirm,
                     const std::vector<uint8_t>& value)
      : device_address_(device_address)
      , characteristic_id_(characteristic_id)
      , confirm_(confirm)
      , value_(value) { }

    const std::string& GetDeviceAddress() const { return device_address_; }
    const bluetooth::GattIdentifier& GetCharacteristicId() const { return characteristic_id_; }
    bool GetConfirm() const { return confirm_; }
    const std::vector<uint8_t>& GetValue() const { return value_; }
  private:
    std::string device_address_;
    bluetooth::GattIdentifier characteristic_id_;
    bool confirm_;
    std::vector<uint8_t> value_;
  };

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
  void SendMessageToConnectedCentral(const std::vector<uint8_t>& value);
  void SendMessageToConnectedCentral(uint8_t msgID, const std::vector<uint8_t>& value);
  void SendMessageToConnectedCentral(uint8_t msgID, const std::string& str);
  void QueueNotification(const std::string& device_address,
                         const bluetooth::GattIdentifier& characteristic_id,
                         bool confirm,
                         const std::vector<uint8_t>& value);
  void TransmitNextNotification();
  void OnNotificationSent(const std::string& device_address,
                          int status) override;

  void HandleIncomingMessageFromCentral(const std::vector<uint8_t>& message);
  void ResetConnectionState();
  void HandleConnect(const std::string& device_address);
  void HandleDisconnect();
  void EnableWiFiInterface(const bool enable);
  std::string GetPathToWiFiConfigFile();
  std::string GetPathToWiFiDefaultConfigFile();
  std::string GetWiFiDefaultConfig();
  void SetWiFiConfig(const std::map<std::string, std::string> networks);
  std::string GetPathToSSHAuthorizedKeys();
  void SetSSHAuthorizedKeys(const std::string& keys);
  void ExecCommand(const std::vector<std::string>& args);
  void ExecCommandInBackground(const std::vector<std::string>& args);
  int ExecCommandEx(const std::vector<std::string>& args, std::string& output);


  CLIBluetoothLowEnergyCallback* low_energy_callback_;
  // Single mutex to protect all variables below.
  std::mutex mutex_;

  // The address of the central that is connected to us.  Only allow 1 central
  // at a time to be connected.
  std::string connected_central_address_;

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

  uint8_t heart_beat_count_;

  std::vector<uint8_t> peripheral_to_central_value_;
  std::vector<uint8_t> central_to_peripheral_value_;
  std::vector<uint8_t> disconnect_value_;

  // The daemon itself doesn't maintain a Client Characteristic Configuration
  // mapping, so we do it ourselves here.
  uint8_t peripheral_to_central_ccc_value_;
  uint8_t disconnect_ccc_value_;

  // The unique IDs that refer to each of the BLE Service GATT objects.
  // These are returned to us from the Bluetooth daemon as we populate the database.
  bluetooth::GattIdentifier service_id_;
  bluetooth::GattIdentifier central_to_peripheral_id_;
  bluetooth::GattIdentifier peripheral_to_central_id_;
  bluetooth::GattIdentifier peripheral_to_central_cccd_id_;
  bluetooth::GattIdentifier disconnect_id_;
  bluetooth::GattIdentifier disconnect_cccd_id_;

  // Wether we should also start advertising
  bool advertise_;

  // libchrome task runner that we use to post
  // notifications on the main thread.
  scoped_refptr<base::SingleThreadTaskRunner> main_task_runner_;

  base::Thread* bg_thread_;
  // libchrome task runner that we use to post
  // notifications on the background thread
  scoped_refptr<base::SingleThreadTaskRunner> bg_task_runner_;

  // We use this to pass weak_ptr's to base::Bind, which won't execute if the
  // BLEServer object gets deleted. This is a convenience utility from
  // libchrome and we use it here since base::TaskRunner uses base::Callback.
  // Note: This should remain the last member so that it'll be destroyed and
  // invalidate its weak pointers before any other members are destroyed.
  base::WeakPtrFactory<BLEServer> weak_ptr_factory_;


  std::deque<NotificationInfo> notifications_;

  std::vector<uint8_t> multipart_message_;

  DISALLOW_COPY_AND_ASSIGN(BLEServer);
};

}  // namespace Anki
