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
    if (interactive_) {
      std::cerr << "Interactive mode not yet supported" << std::endl;
    }
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
  } else if (AreCaseInsensitiveStringsEqual(command, "flash")) {
    if (args_.size() < 2) {
      std::cerr << "Path to firmware required" << std::endl;
      _exit(1);
    }
    path_to_firmware_ = args_[1];
    flash_cube_after_connect_ = true;
    if (address_.empty()) {
      connect_to_first_cube_found_ = true;
      ScanForCubes();
    } else {
      ConnectToCube();
    }
  } else if (AreCaseInsensitiveStringsEqual(command, "disconnect")) {
    DisconnectFromCube();
  } else {
    std::cerr << command << " is not a supported command" << std::endl;
  }
}

void VicCubeTool::ScanTimerCallback(ev::timer& w, int revents) {
  scanning_ = false;
  StopScan();
  if (scan_records_.empty()) {
    std::cerr << "No cubes found.  Exiting..." << std::endl;
    _exit(1);
  }
}

void VicCubeTool::ScanForCubes() {
  std::cout << "Scanning for cubes...." << std::endl;
  scanning_ = true;
  StartScan(Anki::kCubeService_128_BIT_UUID);
  if (!stop_scan_timer_) {
    stop_scan_timer_ = new ev::timer(loop_);
  }
  stop_scan_timer_->set <VicCubeTool, &VicCubeTool::ScanTimerCallback> (this);
  stop_scan_timer_->start(2.0);
}

void VicCubeTool::OnScanResults(int error,
                          const std::vector<Anki::BluetoothDaemon::ScanResultRecord>& records)
{
  if (!scanning_) {
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
      connect_to_first_cube_found_ = false;
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
    return;
  }
  std::cout << std::string(connected ? "Connected to " : "Disconnected from ")
            << address << std::endl;

  if (!connected) {
    _exit(0);
  }
  connection_id_ = connection_id;
  if (flash_cube_after_connect_) {
    FlashCube(path_to_firmware_);
  }
}

void VicCubeTool::OnReceiveMessage(const int connection_id,
                                   const std::string& characteristic_uuid,
                                   const std::vector<uint8_t>& value)
{
  logv("OnReceiveMessage(id = %d, char_uuid = '%s', value.size = %d)",
       connection_id, characteristic_uuid.c_str(), value.size());
  if (connection_id == -1 || connection_id != connection_id_) {
    return;
  }
  if (AreCaseInsensitiveStringsEqual(characteristic_uuid,Anki::kCubeAppVersion_128_BIT_UUID)) {
    if (!value.empty()) {
      std::cout << "APP VERSION: " << std::string(value.begin(), value.end()) << std::endl;
      std::cout << "Done flashing firmware" << std::endl;
      task_executor_->WakeAfter(std::bind(&IPCClient::Disconnect, this, connection_id_),
                                std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 * 5));
    }
  } else if (AreCaseInsensitiveStringsEqual(characteristic_uuid,Anki::kCubeAppRead_128_BIT_UUID)) {
    std::cout << "APP DATA: " << std::string(value.begin(), value.end()) << std::endl;
  }
}

#define MAX_BYTES_PER_PACKET 18
void VicCubeTool::FlashCube(const std::string& pathToFirmware) {
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
    size_t chunk_length = std::min(MAX_BYTES_PER_PACKET, (int) (firmware.size() - offset));
    packet.push_back(firmware, offset, chunk_length);
    SendMessage(connection_id_, Anki::kCubeOTATarget_128_BIT_UUID, true, packet.GetStdVector());
    offset += chunk_length;
  }
  Anki::ByteVector final_packet({(uint8_t) 0xff, (uint8_t) 0xff});
  final_packet.push_back_le((uint16_t) firmware.size());
  final_packet.push_back_le(Anki::Crc32(firmware));
  SendMessage(connection_id_, Anki::kCubeOTATarget_128_BIT_UUID, true, final_packet.GetStdVector());
}

void VicCubeTool::DisconnectFromCube() {
  std::cerr << "Sorry, disconnect is not yet implemented" << std::endl;
}
