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

#include <logwrap/logwrap.h>

#include "private/android_filesystem_config.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <sstream>

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
  : low_energy_callback_(nullptr),
    bluetooth_(bluetooth),
    server_if_(-1),
    advertise_(advertise),
    main_task_runner_(main_task_runner),
    weak_ptr_factory_(this) {
  CHECK(bluetooth_.get());
  bg_thread_ = new base::Thread("BackgroundTaskThread");
  bg_thread_->Start();
  bg_task_runner_ = bg_thread_->task_runner();
  HandleDisconnect();
}

BLEServer::~BLEServer() {
  std::lock_guard<std::mutex> lock(mutex_);
  bg_thread_->Stop();
  delete bg_thread_; bg_thread_ = nullptr;
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
    low_energy_callback_ = new CLIBluetoothLowEnergyCallback(bluetooth_);
    bool registered = ble->RegisterClient(low_energy_callback_);
    LOG(INFO) << "RegisterClient result = " << registered;
  }

}

void BLEServer::OnCharacteristicReadRequest(
    const std::string& device_address,
    int request_id, int offset, bool /* is_long */,
    const bluetooth::GattIdentifier& characteristic_id) {
  std::lock_guard<std::mutex> lock(mutex_);

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
  if (enable) {
    ExecCommandInBackground({"ifconfig", "wlan0", "up"});
    ExecCommandInBackground({"dhcptool", "wlan0"});
  } else {
    ExecCommandInBackground({"ifconfig", "wlan0", "down"});
  }
  ExecCommandInBackground({"ifconfig", "wlan0"});
}

std::string BLEServer::GetPathToWiFiConfigFile()
{
  return "/data/misc/wifi/wpa_supplicant.conf";
}

std::string BLEServer::GetPathToWiFiDefaultConfigFile()
{
  return "/system/etc/wpa_supplicant_default.conf";
}

std::string BLEServer::GetWiFiDefaultConfig()
{
  std::string defaultConfigPath = GetPathToWiFiDefaultConfigFile();
  std::ifstream istrm(defaultConfigPath, std::ios::binary | std::ios::in);
  if (istrm.is_open()) {
    auto ss = std::ostringstream{};
    ss << istrm.rdbuf();
    return ss.str();
  } else {
    const char* defaultConfig = R"foo(ctrl_interface=/data/misc/wifi/sockets
update_config=1
p2p_no_group_iface=1

)foo";
    return defaultConfig;
  }
}

void BLEServer::SetWiFiConfig(const std::map<std::string, std::string> networks)
{
  std::string wifiConfigPath = GetPathToWiFiConfigFile();
  std::string wifiConfigTmpPath = wifiConfigPath + ".tmp";

  (void) unlink(wifiConfigTmpPath.c_str());

  auto wifiConfigStream = std::ofstream(wifiConfigTmpPath, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!wifiConfigStream.is_open()) {
    LOG(INFO) << "Failed to open '" << wifiConfigTmpPath << "'";
    return;
  }
  wifiConfigStream << GetWiFiDefaultConfig();

  for (auto const& kv : networks) {
    wifiConfigStream << "network={" << std::endl
                     << "\tssid=\"" << kv.first << "\"" << std::endl
                     << "\tpsk=\"" << kv.second << "\"" << std::endl
                     << "}" << std::endl << std::endl;
  }

  wifiConfigStream.close();
  int rc = chmod(wifiConfigTmpPath.c_str(), (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP));
  if (rc) {
    LOG(INFO) << "Error chmoding '" << wifiConfigTmpPath << "' errno = " << errno;
    return;
  }
  rc = chown(wifiConfigTmpPath.c_str(), AID_WIFI, AID_WIFI);
  if (rc) {
    LOG(INFO) << "Error chowning '" << wifiConfigTmpPath << "' errno = " << errno;
    return;
  }
  rc = rename(wifiConfigTmpPath.c_str(), wifiConfigPath.c_str());
  if (rc) {
    LOG(INFO) << "Error renaming '" << wifiConfigTmpPath << "' to '"
              << wifiConfigPath << "' errno = " << errno;
    return;
  }
  ExecCommandInBackground({"wpa_cli", "reconfigure"});
  ExecCommandInBackground({"dhcptool", "wlan0"});
  ExecCommandInBackground({"wpa_cli", "status"});
}

std::string BLEServer::GetPathToSSHAuthorizedKeys()
{
  return "/data/root/.ssh/authorized_keys";
}

void BLEServer::SetSSHAuthorizedKeys(const std::string& keys)
{
  std::string authorizedKeysPath = GetPathToSSHAuthorizedKeys();
  std::string authorizedKeysTmpPath = authorizedKeysPath + ".tmp";

  (void) unlink(authorizedKeysTmpPath.c_str());

  auto authKeysStream = std::ofstream(authorizedKeysTmpPath, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!authKeysStream.is_open()) {
    LOG(INFO) << "Failed to open '" << authorizedKeysTmpPath << "'";
    return;
  }

  authKeysStream << keys;

  authKeysStream.close();
  int rc = chmod(authorizedKeysTmpPath.c_str(), (S_IRUSR | S_IWUSR));
  if (rc) {
    LOG(INFO) << "Error chmoding '" << authorizedKeysTmpPath << "' errno = " << errno;
    return;
  }
  rc = chown(authorizedKeysTmpPath.c_str(), AID_ROOT, AID_ROOT);
  if (rc) {
    LOG(INFO) << "Error chowning '" << authorizedKeysTmpPath << "' errno = " << errno;
    return;
  }
  rc = rename(authorizedKeysTmpPath.c_str(), authorizedKeysPath.c_str());
  if (rc) {
    LOG(INFO) << "Error renaming '" << authorizedKeysTmpPath << "' to '"
              << authorizedKeysPath << "' errno = " << errno;
    return;
  }
  SendMessageToConnectedCentral(MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE,
                                "SSH authorized keys set");

}

void BLEServer::ExecCommandInBackground(const std::vector<std::string>& args)
{
  bg_task_runner_->PostTask(FROM_HERE, base::Bind(&BLEServer::ExecCommand,
                                                  weak_ptr_factory_.GetWeakPtr(),
                                                  args));
}

void BLEServer::ExecCommand(const std::vector<std::string>& args)
{
  static const std::vector<std::string> allowedCommands =
    { "ifconfig", "wpa_cli", "dhcptool", "reboot"};

  if (args.empty()) {
    SendMessageToConnectedCentral(MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE,
                                  "Invalid argument: args in empty");
    return;
  }

  LOG(INFO) << "ExecCommand. args[0] = " << args[0];

  if (std::find(allowedCommands.begin(), allowedCommands.end(), args[0]) == allowedCommands.end()) {
    std::string errMessage = "Operation not permitted. Valid commands are ";
    bool first = true;
    for (auto const& s : allowedCommands) {
      if (first) {
        first = false;
      } else {
        errMessage += ", ";
      }
      errMessage += s;
    }
    SendMessageToConnectedCentral(MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE, errMessage);
    return;
  }

  int argc = args.size();
  char* argv[argc];
  int status = 0;
  bool ignore_int_quit = false;
  int log_target = LOG_FILE;
  bool abbreviated = true;
  char *file_path = (char *) "/data/local/tmp/cmd_results.txt";
  struct AndroidForkExecvpOption* opts = NULL;
  size_t opts_len = 0;
  std::string commandLine;

  for (unsigned int j = 0 ; j < args.size(); ++j) {
    argv[j] = (char *) malloc(args[j].size() + 1);
    strcpy(argv[j], args[j].c_str());
    if (j != 0) {
      commandLine += " ";
    }
    commandLine += std::string(argv[j]);
  }

  (void) unlink(file_path);

  LOG(INFO) << "About fork and exec '" << commandLine << "'";

  int rc = android_fork_execvp_ext(argc, argv, &status, ignore_int_quit,
                                   log_target, abbreviated, file_path,
                                   opts, opts_len);
  LOG(INFO) << "Fork results. rc = " << rc << " and status = " << status;
  for (unsigned int j = 0; j < args.size(); ++j) {
    free(argv[j]);
  }

  int results_fd = open(file_path, O_RDONLY);
  if (results_fd > 0) {
    std::vector<uint8_t> value;
    size_t count = 128;
    uint8_t data[count];
    ssize_t bytesRead;
    bytesRead = read(results_fd, data, count);
    while (bytesRead > 0) {
      std::copy(&data[0], &data[bytesRead], std::back_inserter(value));
      bytesRead = read(results_fd, data, count);
    }
    value.push_back(0);
    std::vector<uint8_t> clean_value;
    size_t k = 0;
    while (k < value.size()) {
      clean_value.push_back(value[k]);
      if (value[k] == '\n') {
        k++;
      }
      k++;
    }
    SendMessageToConnectedCentral(MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE, clean_value);
    close(results_fd); results_fd = -1;
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
  case VictorMsg_Command::MSG_B2V_WIFI_SET_CONFIG:
    {
      LOG(INFO) << "Receive WiFi Set Config";
      std::map<std::string, std::string> networks;
      std::string arg;
      std::string ssid;
      for (auto it = message.begin() + 2 ; it != message.end(); it++) {
        if (((char) *it) == '\0') {
          if (ssid.empty()) {
            ssid = arg;
            arg.clear();
          } else {
            networks.emplace(ssid, arg);
            ssid.clear();
            arg.clear();
          }
        } else {
          arg.push_back(*it);
        }
      }
      if (!arg.empty() && !ssid.empty()) {
        networks.emplace(ssid, arg);
      }
      SetWiFiConfig(networks);
    }
    break;
  case VictorMsg_Command::MSG_B2V_SSH_SET_AUTHORIZED_KEYS:
    {
      LOG(INFO) << "Receive SSH authorized keys";
      std::string keys;
      for (auto it = message.begin() + 2; it != message.end(); it++) {
        keys.push_back(*it);
      }
      SetSSHAuthorizedKeys(keys);
    }
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
  case VictorMsg_Command::MSG_B2V_DEV_EXEC_CMD_LINE:
    {
      LOG(INFO) << "Received command line to execute";
      std::vector<std::string> args;
      std::string arg;
      for (auto it = message.begin() + 2; it != message.end(); it++) {
        if (((char) *it) == '\0') {
          args.push_back(arg);
          arg.clear();
        } else {
          arg.push_back((char) *it);
        }
      }
      if (!arg.empty()) {
        args.push_back(arg);
      }
      ExecCommandInBackground(args);
    }
    break;
  case VictorMsg_Command::MSG_B2V_MULTIPART_START:
    LOG(INFO) << "Received multipart message start";
    multipart_message_.clear();
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(multipart_message_));
    break;
  case VictorMsg_Command::MSG_B2V_MULTIPART_CONTINUE:
    LOG(INFO) << "Received multipart message continue";
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(multipart_message_));
    break;
  case VictorMsg_Command::MSG_B2V_MULTIPART_FINAL:
    LOG(INFO) << "Received multipart message final";
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(multipart_message_));
    HandleIncomingMessageFromCentral(multipart_message_);
    break;
  default:
    break;
  }
}

void BLEServer::ResetConnectionState() {
  connected_central_address_.clear();
  peripheral_to_central_ccc_value_ = 0;
  disconnect_ccc_value_ = 0;
  heart_beat_count_ = 0;
  peripheral_to_central_value_.clear();
  central_to_peripheral_value_.clear();
  disconnect_value_.clear();
  notifications_.clear();
}

void BLEServer::HandleConnect(const std::string& device_address) {
  if (low_energy_callback_) {
    low_energy_callback_->StopAdvertising();
  }
  connected_central_address_ = device_address;
}

void BLEServer::HandleDisconnect() {
  ResetConnectionState();
  if (low_energy_callback_) {
    low_energy_callback_->StartAdvertising();
  }
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

  HandleConnect(device_address);

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
  if (value.size() <= kAnkiVictorMsgMaxSize) {
    QueueNotification(connected_central_address_, peripheral_to_central_id_, false, value);
  } else {
    // Fragment large message into multipart messages to queue
    size_t offset = 0;
    while (offset < value.size()) {
      std::vector<uint8_t> message;
      uint8_t msgID;
      uint8_t msgSize = kAnkiVictorMsgBaseSize;
      if (offset == 0) {
        msgID = MSG_V2B_MULTIPART_START;
        msgSize += kAnkiVictorMsgPayloadMaxSize;
      } else if (value.size() - offset > kAnkiVictorMsgPayloadMaxSize) {
        msgID = MSG_V2B_MULTIPART_CONTINUE;
        msgSize += kAnkiVictorMsgPayloadMaxSize;
      } else {
        msgID = MSG_V2B_MULTIPART_FINAL;
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

void BLEServer::SendMessageToConnectedCentral(uint8_t msgID, const std::vector<uint8_t>& value)
{
  std::vector<uint8_t> msg;
  msg.push_back(value.size() + 1);
  msg.push_back(msgID);
  msg.insert(std::end(msg), std::begin(value), std::end(value));
  SendMessageToConnectedCentral(msg);
}

void BLEServer::SendMessageToConnectedCentral(uint8_t msgID, const std::string& str)
{
  std::vector<uint8_t> value(str.begin(), str.end());
  value.push_back(0);
  SendMessageToConnectedCentral(msgID, value);
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
  if (status == bluetooth::GATT_ERROR_NONE) {
    LOG(INFO) << "Notification was sent - device: " << device_address;
    (void) usleep(1000 * 5);
  } else {
    LOG(WARNING) << "Notification failed to send - device: " << device_address
                 << " status: " << status;
    (void) usleep(1000 * 30);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (status == bluetooth::GATT_ERROR_NONE) {
    notifications_.pop_front();
  }
  TransmitNextNotification();
}

}  // namespace Anki
