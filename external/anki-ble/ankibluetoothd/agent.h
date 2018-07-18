/**
 * File: agent.h
 *
 * Author: seichert
 * Created: 1/15/2018
 *
 * Description: BLE agent
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include "ipc-server.h"
#include "ble_advertise_settings.h"
#include "bluetooth_gatt.h"
#include "gatt_constants.h"

#include <deque>
#include <map>
#include <set>

namespace Anki {
namespace BluetoothDaemon {

class Agent : public IPCServer {
 public:
  Agent(struct ev_loop* loop);
  bool StartPeripheral();

 protected:
  virtual void OnOutboundConnectionChange(const std::string& address,
                                          const int connected,
                                          const int connection_id,
                                          const std::vector<GattDbRecord>& records);
  virtual void OnNewIPCClient(const int sockfd);
  virtual void SendMessage(const int connection_id,
                           const std::string& characteristic_uuid,
                           const bool reliable,
                           const std::vector<uint8_t>& value);
  virtual void ReadCharacteristic(const int connection_id,
                                  const std::string& characteristic_uuid);
  virtual void ReadDescriptor(const int connection_id,
                              const std::string& characteristic_uuid,
                              const std::string& descriptor_uuid);
  virtual void Disconnect(const int connection_id);
  virtual void StartAdvertising(const int sockfd, const BLEAdvertiseSettings& settings);
  virtual void StopAdvertising();
  virtual void StartScan(const int sockfd, const std::string& serviceUUID);
  virtual void StopScan(const int sockfd);
  virtual void ConnectToPeripheral(const int sockfd, const std::string& address);
  virtual void OnPeerClose(const int sockfd);
  virtual void RequestConnectionParameterUpdate(const int sockfd,
                                                const std::string& address,
                                                int min_interval,
                                                int max_interval,
                                                int latency,
                                                int timeout);
  virtual void SetAdapterName(const std::string& name);
  virtual void DisconnectByAddress(const std::string& address);


 private:
  void TransmitNextNotification();
  void SendMessageToConnectedCentral(int characteristic_handle,
                                     int confirm, const std::vector<uint8_t>& value);
  void PeripheralInboundConnectionCallback(int conn_id, int connected);
  void PeripheralReadCallback(int conn_id, int trans_id, int attr_handle, int offset);
  void PeripheralWriteCallback(int conn_id, int trans_id, int attr_handle, int offset,
                               bool need_rsp, const std::vector<uint8_t>& value);
  void PeripheralIndicationSentCallback(int conn_id, int status);
  void PeripheralCongestionCallback(int conn_id, bool congested);
  void CentralScanResultCallback(const std::string& address,
                                 int rssi,
                                 const std::vector<uint8_t>& adv_data);
  void CentralOutboundConnectionCallback(const std::string& address,
                                         const int connected,
                                         const BluetoothGattConnection& connection);
  void CentralNotificationReceivedCallback(const std::string& address,
                                           const int conn_id,
                                           const std::string& char_uuid,
                                           const std::vector<uint8_t>& value);
  void CentralCharacteristicReadCallback(const std::string& address,
                                         const int conn_id,
                                         const int error,
                                         const std::string& char_uuid,
                                         const std::vector<uint8_t>& value);
  void CentralDescriptorReadCallback(const std::string& address,
                                     const int conn_id,
                                     const int error,
                                     const std::string& char_uuid,
                                     const std::string& desc_uuid,
                                     const std::vector<uint8_t>& value);


  static void StaticPeripheralInboundConnectionCallback(int conn_id, int connected);
  static void StaticPeripheralReadCallback(int conn_id, int trans_id, int attr_handle, int offset);
  static void StaticPeripheralWriteCallback(int conn_id, int trans_id, int attr_handle, int offset,
                                      bool need_rsp, const std::vector<uint8_t>& value);
  static void StaticPeripheralIndicationSentCallback(int conn_id, int status);
  static void StaticPeripheralCongestionCallback(int conn_id, bool congested);
  static void StaticCentralScanResultCallback(const std::string& address,
                                              int rssi,
                                              const std::vector<uint8_t>& adv_data);
  static void StaticCentralOutboundConnectionCallback(const std::string& address,
                                                      const int connected,
                                                      const BluetoothGattConnection& connection);
  static void StaticCentralNotificationReceivedCallback(const std::string& address,
                                                        const int conn_id,
                                                        const std::string& char_uuid,
                                                        const std::vector<uint8_t>& value);
  static void StaticCentralCharacteristicReadCallback(const std::string& address,
                                                      const int conn_id,
                                                      const int error,
                                                      const std::string& char_uuid,
                                                      const std::vector<uint8_t>& value);
  static void StaticCentralDescriptorReadCallback(const std::string& address,
                                                  const int conn_id,
                                                  const int error,
                                                  const std::string& char_uuid,
                                                  const std::string& desc_uuid,
                                                  const std::vector<uint8_t>& value);

  std::mutex mutex_;
  BLEAdvertiseSettings ble_advertise_settings_;
  BluetoothGattService bluetooth_gatt_service_;
  int app_write_characteristic_handle_ = -1;
  int app_read_characteristic_handle_ = -1;
  int app_read_ccc_descriptor_handle_ = -1;
  bool advertising_ = false;
  bool connected_ = false;
  int inbound_connection_id_ = -1;
  int inbound_handler_sock_fd_ = -1;
  bool congested_ = false;
  uint16_t app_read_ccc_value_ = kCCCDefaultValue;
  std::vector<uint8_t> app_read_value_;

  typedef struct Notification {
    int characteristic_handle;
    int confirm;
    std::vector<uint8_t> value;
  } Notification;

  std::deque<Notification> notification_queue_;
  bool scanning_;
  std::string scan_filter_service_uuid_;
  std::map<int, std::set<std::string>> outbound_connection_addresses_;
  std::set<int> scanning_ipc_clients_;
};

} // namespace BluetoothDaemon
} // namespace Anki

bool StartBLEAgent();
bool StopBLEAgent();

