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
#include "include_ev.h"
#include "log.h"
#include "stringutils.h"

#include <unistd.h>

#include <iostream>

VicCubeTool::~VicCubeTool() {
  delete connect_retry_timer_; connect_retry_timer_ = nullptr;
  delete stop_scan_timer_; stop_scan_timer_ = nullptr;
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
    ConnectToCube(address_);
  } else if (AreCaseInsensitiveStringsEqual(command, "flash")) {
    FlashCube(address_, args_[1]);
  } else if (AreCaseInsensitiveStringsEqual(command, "disconnect")) {
    DisconnectFromCube();
  } else {
    std::cerr << command << " is not a supported command" << std::endl;
  }
}

void VicCubeTool::ScanTimerCallback(ev::timer& w, int revents) {
  StopScan();
}

void VicCubeTool::ScanForCubes() {
  std::cout << "Scanning for cubes...." << std::endl;
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
  if (error) {
    std::cerr << "Error scanning for cubes. error = " << error << std::endl;
    return;
  }
  for (auto const& r : records) {
    scan_records_[r.address] = r;
    std::cout << r.address << " '" << r.local_name << "' " << "rssi = " << r.rssi << std::endl;
  }
}

void VicCubeTool::ConnectToCube(const std::string& address) {
  ConnectToPeripheral(address);
}

void VicCubeTool::OnOutboundConnectionChange(const std::string& address,
                                             const int connected,
                                             const int connection_id,
                                             const std::vector<Anki::BluetoothDaemon::GattDbRecord>& records) {
  std::cout << "OnOutboundConnectionChange - address = " << address
            << ", connected = " << connected
            << ", connection_id = " << connection_id << std::endl;
  for (auto const& r : records) {
    std::cout << std::string(r.uuid) << " " << static_cast<int>(r.type);
    if (r.handle) {
      std::cout << ", handle = " << r.handle;
    }
    if (r.start_handle) {
      std::cout << ", start_handle = " << r.start_handle;
    }
    if (r.end_handle) {
      std::cout << ", end_handle = " << r.end_handle;
    }
    if (r.properties) {
      std::cout << ", properties = " << r.properties;
    }
    std::cout << std::endl;
  }
}


void VicCubeTool::FlashCube(const std::string& address, const std::string& pathToFirmware) {
  std::cerr << "Sorry, flash is not yet implemented" << std::endl;
}

void VicCubeTool::DisconnectFromCube() {
  std::cerr << "Sorry, disconnect is not yet implemented" << std::endl;
}
