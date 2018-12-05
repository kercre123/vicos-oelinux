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
#include "ble_advertise_settings.h"
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
  void OnCharacteristicReadResult(const int connection_id,
                                  const int error,
                                  const std::string& characteristic_uuid,
                                  const std::vector<uint8_t>& data);
  void OnDescriptorReadResult(const int connection_id,
                              const int error,
                              const std::string& characteristic_uuid,
                              const std::string& descriptor_uuid,
                              const std::vector<uint8_t>& data);
  void OnScanResults(int error, const std::vector<ScanResultRecord>& records);
  void OnRequestConnectionParameterUpdateResult(const int sockfd,
                                                const std::string& address,
                                                const int status);
  virtual void OnOutboundConnectionChange(const std::string& address,
                                          const int connected,
                                          const int connection_id,
                                          const std::vector<GattDbRecord>& records);

  virtual void OnNewIPCClient(const int sockfd) {}
  virtual void OnReceiveIPCMessage(const int sockfd,
                                   const IPCMessageType type,
                                   const std::vector<uint8_t>& data);
  virtual void OnPeerClose(const int sockfd);
  virtual void SendMessage(const int connection_id,
                           const std::string& characteristic_uuid,
                           const bool reliable,
                           const std::vector<uint8_t>& value) {}
  virtual void ReadCharacteristic(const int connection_id,
                                  const std::string& characteristic_uuid) {}
  virtual void ReadDescriptor(const int connection_id,
                              const std::string& characteristic_uuid,
                              const std::string& descriptor_uuid) {}
  virtual void Disconnect(const int connection_id) {}
  virtual void StartAdvertising(const int sockfd, const BLEAdvertiseSettings& settings) {}
  virtual void StopAdvertising() {}
  virtual void StartScan(const int sockfd, const std::string& serviceUUID) {}
  virtual void StopScan(const int sockfd) {}
  virtual void ConnectToPeripheral(const int sockfd, const std::string& address) {}
  virtual void RequestConnectionParameterUpdate(const int sockfd,
                                                const std::string& address,
                                                int min_interval,
                                                int max_interval,
                                                int latency,
                                                int timeout) {}
  virtual void SetAdapterName(const std::string& name) {}
  virtual void DisconnectByAddress(const std::string& address) {}
 private:
  void AcceptWatcherCallback(ev::io& w, int revents);
  ev::io* accept_watcher_;
};

} // namespace BluetoothDaemon
} // namespace Anki

