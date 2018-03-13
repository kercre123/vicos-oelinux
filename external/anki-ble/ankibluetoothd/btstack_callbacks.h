/**
 * File: btstack_callbacks.h
 *
 * Author: seichert
 * Created: 2/16/2018
 *
 * Description: Bluetooth Stack callbacks
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include "bluetooth_gatt.h"

#include <vector>

namespace Anki {
namespace BluetoothStack {

typedef void (*InboundConnectionCallback)(int conn_id, int connected);
typedef void (*RequestReadCallback)(int conn_id, int trans_id, int attr_handle, int offset);
typedef void (*RequestWriteCallback)(int conn_id, int trans_id, int attr_handle, int offset,
                                     bool need_rsp, const std::vector<uint8_t>& value);
typedef void (*IndicationSentCallback)(int conn_id, int status);
typedef void (*CongestionCallback)(int conn_id, bool congested);
typedef void (*ScanResultCallback)(const std::string& address,
                                   const int rssi,
                                   const std::vector<uint8_t>& adv_data);
typedef void (*OutboundConnectionCallback)(const std::string& address,
                                           const int connected,
                                           const BluetoothGattConnection& connection);
typedef void (*NotificationReceivedCallback)(const std::string& address,
                                             const int conn_id,
                                             const std::string& char_uuid,
                                             const std::vector<uint8_t>& value);
typedef void (*CharacteristicReadCallback)(const std::string& address,
                                           const int conn_id,
                                           const int error,
                                           const std::string& char_uuid,
                                           const std::vector<uint8_t>& value);
typedef void (*DescriptorReadCallback)(const std::string& address,
                                       const int conn_id,
                                       const int error,
                                       const std::string& char_uuid,
                                       const std::string& desc_uuid,
                                       const std::vector<uint8_t>& value);

typedef struct Callbacks {
  InboundConnectionCallback inbound_connection_cb;
  RequestReadCallback request_read_cb;
  RequestWriteCallback request_write_cb;
  IndicationSentCallback indication_sent_cb;
  CongestionCallback congestion_cb;
  ScanResultCallback scan_result_cb;
  OutboundConnectionCallback outbound_connection_cb;
  NotificationReceivedCallback notification_received_cb;
  CharacteristicReadCallback characteristic_read_cb;
  DescriptorReadCallback descriptor_read_cb;
} Callbacks;

} // namespace BluetoothStack
} // namespace Anki
