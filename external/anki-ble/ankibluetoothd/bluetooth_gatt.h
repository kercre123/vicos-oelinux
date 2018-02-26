/**
 * File: bluetooth_gatt.h
 *
 * Author: seichert
 * Created: 2/16/2018
 *
 * Description: Bluetooth Gatt structures
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include "btstack_callbacks.h"

#include <string>

typedef struct {
  std::string uuid;
  int permissions;
  int desc_handle;
} BluetoothGattDescriptor;

typedef struct {
  std::string uuid;
  int properties;
  int permissions;
  int char_handle;
  std::vector<BluetoothGattDescriptor> descriptors;
} BluetoothGattCharacteristic;

typedef struct {
  std::string uuid;
  std::vector<BluetoothGattCharacteristic> characteristics;
  ConnectionCallback connection_cb;
  RequestReadCallback request_read_cb;
  RequestWriteCallback request_write_cb;
  IndicationSentCallback indication_sent_cb;
  CongestionCallback congestion_cb;
  int service_handle;
} BluetoothGattService;

