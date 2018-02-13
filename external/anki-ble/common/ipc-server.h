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
  void OnPeripheralStateUpdate(const bool advertising,
                               const int connection_id,
                               const int connected,
                               const bool congested);
  void OnInboundConnectionChange(const int connection_id, const int connected);
  void OnReceiveMessage(const int connection_id,
                        const std::string& characteristic_uuid,
                        const std::vector<uint8_t>& data);

 protected:
  virtual void OnNewIPCClient(const int sockfd) {}
  virtual void OnReceiveIPCMessage(const int sockfd,
                                   const IPCMessageType type,
                                   const std::vector<uint8_t>& data);
  virtual void OnSendMessage(const int connection_id,
                             const std::string& characteristic_uuid,
                             const bool reliable,
                             const std::vector<uint8_t>& value) {}
  virtual void OnDisconnect(const int connection_id) {}
  virtual void OnStartAdvertising() {}
  virtual void OnStopAdvertising() {}

 private:
  void AcceptWatcherCallback(ev::io& w, int revents);
  ev::io* accept_watcher_;
};

} // namespace BluetoothDaemon
} // namespace Anki

