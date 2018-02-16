/**
 * File: ipc-server.h
 *
 * Author: seichert
 * Created: 2/5/2018
 *
 * Description: IPC Server for ankibluetoothd
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include "ipc.h"
#include <vector>

namespace Anki {
namespace BluetoothDaemon {

class IPCServer : public IPCEndpoint {
 public:
  IPCServer(struct ev_loop* loop);
  ~IPCServer();
  bool ListenForIPCConnections();

 protected:
  void OnPeripheralStateUpdate(const bool advertising,
                               const int connection_id,
                               const int connected,
                               const bool congested);
  void OnInboundConnectionChange(const int connection_id, const int connected);
  void OnReceiveMessage(const int connection_id,
                        const std::string& characteristic_uuid,
                        const std::vector<uint8_t>& data);

  virtual void OnNewIPCClient(const int sockfd) {}
  virtual void OnReceiveIPCMessage(const int sockfd,
                                   const IPCMessageType type,
                                   const std::vector<uint8_t>& data);
  virtual void SendMessage(const int connection_id,
                           const std::string& characteristic_uuid,
                           const bool reliable,
                           const std::vector<uint8_t>& value) {}
  virtual void Disconnect(const int connection_id) {}
  virtual void StartAdvertising() {}
  virtual void StopAdvertising() {}

 private:
  void AcceptWatcherCallback(ev::io& w, int revents);
  ev::io* accept_watcher_;
};

} // namespace BluetoothDaemon
} // namespace Anki

