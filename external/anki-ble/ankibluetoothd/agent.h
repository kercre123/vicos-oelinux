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

namespace Anki {
namespace BluetoothDaemon {

class Agent : public IPCServer {
 public:
  Agent(struct ev_loop* loop)
      : IPCServer(loop) {}

  bool StartPeripheral();

 protected:
  virtual void OnNewIPCClient(const int sockfd);
  virtual void SendMessage(const int connection_id,
                           const std::string& characteristic_uuid,
                           const bool reliable,
                           const std::vector<uint8_t>& value);
  virtual void Disconnect(const int connection_id);
  virtual void StartAdvertising();
  virtual void StopAdvertising();
  virtual void StartScan(const std::string& serviceUUID);
  virtual void StopScan();

 private:
  void TransmitNextNotification();
  void SendMessageToConnectedCentral(int characteristic_handle,
                                     int confirm, const std::vector<uint8_t>& value);
  void PeripheralConnectionCallback(int conn_id, int connected);
  void PeripheralReadCallback(int conn_id, int trans_id, int attr_handle, int offset);
  void PeripheralWriteCallback(int conn_id, int trans_id, int attr_handle, int offset,
                               bool need_rsp, const std::vector<uint8_t>& value);
  void PeripheralIndicationSentCallback(int conn_id, int status);
  void PeripheralCongestionCallback(int conn_id, bool congested);
  void CentralScanResultCallback(const std::string& address,
                                 int rssi,
                                 const std::vector<uint8_t>& adv_data);

  static void StaticPeripheralConnectionCallback(int conn_id, int connected);
  static void StaticPeripheralReadCallback(int conn_id, int trans_id, int attr_handle, int offset);
  static void StaticPeripheralWriteCallback(int conn_id, int trans_id, int attr_handle, int offset,
                                      bool need_rsp, const std::vector<uint8_t>& value);
  static void StaticPeripheralIndicationSentCallback(int conn_id, int status);
  static void StaticPeripheralCongestionCallback(int conn_id, bool congested);
  static void StaticCentralScanResultCallback(const std::string& address,
                                              int rssi,
                                              const std::vector<uint8_t>& adv_data);

  std::mutex mutex_;
  BLEAdvertiseSettings ble_advertise_settings_;
  BluetoothGattService bluetooth_gatt_service_;
  int app_write_characteristic_handle_ = -1;
  int app_read_characteristic_handle_ = -1;
  int app_read_ccc_descriptor_handle_ = -1;
  int app_write_encrypted_characteristic_handle_ = -1;
  int app_read_encrypted_characteristic_handle_ = -1;
  int app_read_encrypted_ccc_descriptor_handle_ = -1;
  bool advertising_ = false;
  bool connected_ = false;
  int inbound_connection_id_ = -1;
  bool congested_ = false;
  uint16_t app_read_ccc_value_ = kCCCDefaultValue;
  std::vector<uint8_t> app_read_value_;
  uint16_t app_read_encrypted_ccc_value_ = kCCCDefaultValue;
  std::vector<uint8_t> app_read_encrypted_value_;

  typedef struct Notification {
    int characteristic_handle;
    int confirm;
    std::vector<uint8_t> value;
  } Notification;

  std::deque<Notification> notification_queue_;
  bool scanning_;
  std::string scan_filter_service_uuid_;
};

} // namespace BluetoothDaemon
} // namespace Anki

bool StartBLEAgent();
bool StopBLEAgent();

