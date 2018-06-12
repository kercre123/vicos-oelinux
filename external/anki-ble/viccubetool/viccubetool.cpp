/**
 * File: viccubetool.cpp
 *
 * Author: seichert
 * Created: 2/15/2018
 *
 * Description: IPC client for VicCubeTool
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "viccubetool.h"
#include "anki_ble_uuids.h"
#include "byte_vector.h"
#include "crc32.h"
#include "fileutils.h"
#include "gatt_constants.h"
#include "include_ev.h"
#include "log.h"
#include "stringutils.h"

#include <unistd.h>

#include <algorithm>
#include <iostream>

VicCubeTool::~VicCubeTool() {
  delete connect_retry_timer_; connect_retry_timer_ = nullptr;
  delete stop_scan_timer_; stop_scan_timer_ = nullptr;
  delete task_executor_; task_executor_ = nullptr;
}

void VicCubeTool::Execute() {
  if (!connect_retry_timer_) {
    connect_retry_timer_ = new ev::timer(loop_);
  }
  connect_retry_timer_->set <VicCubeTool, &VicCubeTool::ConnectRetryTimerCallback> (this);
  connect_retry_timer_->start(0.1);
}

void VicCubeTool::ConnectRetryTimerCallback(ev::timer& w, int revents) {
  if (!IsConnected()) {
    (void) Connect();
  }
  if (IsConnected()) {
    bool result = SendIPCMessageToServer(Anki::BluetoothDaemon::IPCMessageType::Ping);
    if (!result) {
        loge("Failed to send IPC message to server");
        _exit(2);
    }
    w.stop();
    OnConnectedToDaemon();
  } else {
    w.start(2.);
  }
}

void VicCubeTool::OnConnectedToDaemon() {
  if (args_.empty()) {
    std::cerr << "No commands provided" << std::endl;
    _exit(1);
  }

  std::string command = args_[0];
  if (AreCaseInsensitiveStringsEqual(command, "scan")) {
    ScanForCubes();
  } else if (AreCaseInsensitiveStringsEqual(command, "connect")) {
    if (args_.size() > 1) {
      address_ = args_[1];
    }
    if (address_.empty()) {
      connect_to_first_cube_found_ = true;
      ScanForCubes();
    } else {
      ConnectToCube();
    }
  } else if (AreCaseInsensitiveStringsEqual(command, "flash")
             || AreCaseInsensitiveStringsEqual(command, "flashdvt1")
             || AreCaseInsensitiveStringsEqual(command, "cube-test")) {
    if (args_.size() < 2) {
      std::cerr << "Path to firmware required" << std::endl;
      _exit(1);
    }
    use_dvt1_flasher_ = AreCaseInsensitiveStringsEqual(command, "flashdvt1");
    path_to_firmware_ = args_[1];
    flash_cube_after_connect_ = true;
    if (AreCaseInsensitiveStringsEqual(command, "cube-test")) {
      cube_test_mode_ = true;
    }
    if (address_.empty()) {
      connect_to_first_cube_found_ = true;
      ScanForCubes();
    } else {
      ConnectToCube();
    }
  } else {
    std::cerr << command << " is not a supported command" << std::endl;
  }
}


void VicCubeTool::ScanTimerCallback(ev::timer& w, int revents) {
  scanning_ = false;
  StopScan();
  if (cube_test_mode_) {
      std::cerr << "No cubes found. Restarting Scan..." << std::endl;
      ScanForCubes();
  }
  else {
    if (scan_records_.empty()) {
      std::cerr << "No cubes found. Exiting..." << std::endl;
      _exit(1);
    }
  }
}

void VicCubeTool::ScanForCubes() {
  std::cout << "Scanning for cubes...." << std::endl;
  scanning_ = true;
  StartScan(Anki::kCubeService_128_BIT_UUID);
  if (!stop_scan_timer_) {
    stop_scan_timer_ = new ev::timer(loop_);
  }
  stop_scan_timer_->stop();
  stop_scan_timer_->set <VicCubeTool, &VicCubeTool::ScanTimerCallback> (this);
  stop_scan_timer_->start(cube_test_mode_ ? 300 : 2.0);
}


void VicCubeTool::OnScanResults(int error,
                          const std::vector<Anki::BluetoothDaemon::ScanResultRecord>& records)
{
  if (!scanning_) {
    std::cerr << "Got scan results when not scanning. error = " << error << std::endl;
    return;
  }
  if (error) {
    std::cerr << "Error scanning for cubes. error = " << error << std::endl;
    return;
  }
  for (auto const& r : records) {
    scan_records_[r.address] = r;
    std::cout << r.address << " '" << r.local_name << "' " << "rssi = " << r.rssi << std::endl;
    if (connect_to_first_cube_found_) {
      if (cube_test_mode_) {
        if (r.rssi < rssi_min_) {
          continue; //reject far away cubes
        }
      }
      else {  //normal mode only connects to one cube
        connect_to_first_cube_found_ = false;
      }
      scanning_ = false;
      StopScan();
      address_ = r.address;
      ConnectToCube();
    }
  }
}

void VicCubeTool::ConnectToCube() {
  std::cout << "Connecting to " << address_ << "....." << std::endl;
  ConnectToPeripheral(address_);
}

void VicCubeTool::OnOutboundConnectionChange(const std::string& address,
                                             const int connected,
                                             const int connection_id,
                                             const std::vector<Anki::BluetoothDaemon::GattDbRecord>& records) {
  if (address != address_) {
    std::cout << "Unexpected "
              << std::string(connected ? "Connection to " : "Disconnection from ")
              << address << std::endl;
    return;
  }
  std::cout << std::string(connected ? "Connected to " : "Disconnected from ")
            << address << std::endl;

  if (!connected) {
    if (cube_test_mode_) {
      //Go back to scanning
      ScanForCubes();
      return;
    }
    else {
      //normal mode quits right away
      _exit(0);
    }
  }
  connection_id_ = connection_id;
  ReadCharacteristic(connection_id_, Anki::kModelNumberString_128_BIT_UUID);
}

void VicCubeTool::OnReceiveMessage(const int connection_id,
                                   const std::string& characteristic_uuid,
                                   const std::vector<uint8_t>& value)
{
  logv("OnReceiveMessage(id = %d, char_uuid = '%s', value.size = %d)",
       connection_id, characteristic_uuid.c_str(), value.size());
  if (connection_id == -1 || connection_id != connection_id_) {
    std::cerr << "Got message from wrong connection" << connection_id << std::endl;
    return;
  }
  if (AreCaseInsensitiveStringsEqual(characteristic_uuid,Anki::kCubeAppVersion_128_BIT_UUID)) {
    if (!value.empty()) {
      std::string firmware_version(value.begin(), value.end());
      if (!new_firmware_version_.empty() && new_firmware_version_ != firmware_version) {
        std::cerr << "Expected firmware version : '" << new_firmware_version_ << "' "
                  << ", but received version '" << firmware_version << "'" << std::endl;
      } else {
        std::cout << "APP VERSION: " << std::string(value.begin(), value.end()) << std::endl;
        std::cout << "Firmware upload complete" << std::endl;
      }
      if (!cube_test_mode_) {
        task_executor_->WakeAfter(std::bind(&IPCClient::Disconnect, this, connection_id_),
                                std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 * 5));
      }
    }
  } else if (AreCaseInsensitiveStringsEqual(characteristic_uuid,Anki::kCubeAppRead_128_BIT_UUID)) {
    std::cout << "APP DATA: " << byteVectorToHexString(value, 1) << std::endl;
  }
}

void VicCubeTool::OnCharacteristicReadResult(const int connection_id,
                                             const int error,
                                             const std::string& characteristic_uuid,
                                             const std::vector<uint8_t>& data)
{
  logv("OnCharacteristicReadResult(connection_id = %d, error = %d, characteristic_uuid = %s, data.size = %d)",
       connection_id, error, characteristic_uuid.c_str(), data.size());
  if (connection_id == -1 || connection_id != connection_id_) {
    return;
  }

  if (error) {
    return;
  }

  int connectionTimeout = kGattConnectionTimeoutDefault;
  if (cube_test_mode_) { connectionTimeout /= 80; } //from 20 seconds to .25

  if (AreCaseInsensitiveStringsEqual(characteristic_uuid, Anki::kModelNumberString_128_BIT_UUID)) {
    cube_model_number_ = std::string(data.begin(), data.end());
    std::cout << "Cube Model Number : " << cube_model_number_ << std::endl;
    RequestConnectionParameterUpdate(address_,
                                     kGattConnectionIntervalHighPriorityMinimum,
                                     kGattConnectionIntervalHighPriorityMaximum,
                                     kGattConnectionLatencyDefault,
                                     connectionTimeout);

    if (flash_cube_after_connect_) {
      if (use_dvt1_flasher_) {
        FlashCubeDVT1(path_to_firmware_);
      } else {
        FlashCube(path_to_firmware_);
      }
    }
  }
}

void VicCubeTool::OnDescriptorReadResult(const int connection_id,
                                         const int error,
                                         const std::string& characteristic_uuid,
                                         const std::string& descriptor_uuid,
                                         const std::vector<uint8_t>& data)
{
  /* do nothing */
}

#define DVT1_MAX_BYTES_PER_PACKET 18
void VicCubeTool::FlashCubeDVT1(const std::string& pathToFirmware) {
  std::vector<uint8_t> firmware;
  if (!Anki::ReadFileIntoVector(pathToFirmware, firmware)) {
    std::cerr << "Error reading firmware.  Exiting..." << std::endl;
    _exit(1);
  }
  std::cout << "Flashing firmware....." << std::endl;
  size_t offset = 0;
  while (offset < firmware.size()) {
    Anki::ByteVector packet;
    packet.push_back_le((uint16_t) offset);
    size_t chunk_length = std::min(DVT1_MAX_BYTES_PER_PACKET, (int) (firmware.size() - offset));
    packet.push_back(firmware, offset, chunk_length);
    SendMessage(connection_id_, Anki::kCubeOTATarget_128_BIT_UUID, true, packet.GetStdVector());
    offset += chunk_length;
  }
  Anki::ByteVector final_packet({(uint8_t) 0xff, (uint8_t) 0xff});
  final_packet.push_back_le((uint16_t) firmware.size());
  final_packet.push_back_le(Anki::Crc32(firmware));
  SendMessage(connection_id_, Anki::kCubeOTATarget_128_BIT_UUID, true, final_packet.GetStdVector());
}

#define MAX_BYTES_PER_PACKET 20
void VicCubeTool::FlashCube(const std::string& pathToFirmware) {
  std::vector<uint8_t> firmware;
  if (!Anki::ReadFileIntoVector(pathToFirmware, firmware)) {
    std::cerr << "Error reading firmware.  Exiting..." << std::endl;
    _exit(1);
  }
  size_t offset = 0x10; // The first 16 bytes of the firmware data are the version string
  new_firmware_version_ = std::string(firmware.begin(), firmware.begin() + offset);
  std::cout << "Flashing firmware version : " << new_firmware_version_ << std::endl;
  while (offset < firmware.size()) {
    Anki::ByteVector packet;
    size_t chunk_length = std::min(MAX_BYTES_PER_PACKET, (int) (firmware.size() - offset));
    packet.push_back(firmware, offset, chunk_length);
    SendMessage(connection_id_, Anki::kCubeOTATarget_128_BIT_UUID, true, packet.GetStdVector());
    offset += chunk_length;
  }
}
