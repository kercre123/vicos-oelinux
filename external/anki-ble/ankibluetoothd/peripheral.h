/**
 * File: peripheral.h
 *
 * Author: seichert
 * Created: 1/15/2018
 *
 * Description: BLE peripheral
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include "ipc-server.h"

namespace Anki {
namespace BluetoothDaemon {

class Peripheral : public IPCServer {
 public:
  Peripheral(struct ev_loop* loop)
      : IPCServer(loop) {}

 protected:
  virtual void OnNewIPCClient(const int sockfd);
  virtual void OnSendMessage(const int connection_id,
                             const std::string& characteristic_uuid,
                             const bool reliable,
                             const std::vector<uint8_t>& value);
  virtual void OnDisconnect(const int connection_id);
  virtual void OnStartAdvertising();
  virtual void OnStopAdvertising();
};

} // namespace BluetoothDaemon
} // namespace Anki

bool StartBLEPeripheral();
bool StopBLEPeripheral();

