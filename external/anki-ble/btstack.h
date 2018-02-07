/**
 * File: btstack.h
 *
 * Author: seichert
 * Created: 1/10/2018
 *
 * Description: Bluetooth Stack functions for Anki Bluetooth Daemon
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include "ble_advertise_settings.h"
#include <string>
#include <vector>

bool LoadBtStack();
void UnLoadBtStack();
bool EnableAdapter();
std::string GetAdapterName();
bool SetAdapterName(const std::string& name);
bool RegisterGattClient();
bool RegisterGattServer();

typedef void (*ConnectionCallback)(int conn_id, int connected);
typedef void (*RequestReadCallback)(int conn_id, int trans_id, int attr_handle, int offset);
typedef void (*RequestWriteCallback)(int conn_id, int trans_id, int attr_handle, int offset,
                                     bool need_rsp, const std::vector<uint8_t>& value);
typedef void (*IndicationSentCallback)(int conn_id, int status);
typedef void (*CongestionCallback)(int conn_id, bool congested);


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

bool AddGattService(BluetoothGattService* service);
bool StartGattService(BluetoothGattService* service);
bool StopGattService(BluetoothGattService* service);
bool DisconnectGattPeer(int conn_id);
bool RemoveGattService(BluetoothGattService* service);
bool SendGattIndication(int attribute_handle, int conn_id, int confirm,
                        const std::vector<uint8_t>& value);
bool SendResponse(int conn_id, int trans_id, int handle, int error, int offset,
                  const std::vector<uint8_t>& value);
bool StartAdvertisement(const Anki::BLEAdvertiseSettings& settings);
bool StopAdvertisement();
