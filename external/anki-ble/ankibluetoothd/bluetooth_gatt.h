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

#include <string>

typedef struct BluetoothGattDescriptor {
  std::string uuid;
  int permissions;
  int desc_handle;
  bool descriptor_written;
  BluetoothGattDescriptor()
      : BluetoothGattDescriptor("", 0) { }
  BluetoothGattDescriptor(const std::string& uuid,
                          const int permissions)
      : BluetoothGattDescriptor(uuid, permissions, -1) {}
  BluetoothGattDescriptor(const std::string& uuid,
                          const int permissions,
                          const int desc_handle)
      : uuid(uuid)
      , permissions(permissions)
      , desc_handle(desc_handle)
      , descriptor_written(false) {}
} BluetoothGattDescriptor;

typedef struct BluetoothGattCharacteristic {
  std::string uuid;
  int properties;
  int permissions;
  int char_handle;
  bool registered_for_notifications;
  std::vector<BluetoothGattDescriptor> descriptors;
  BluetoothGattCharacteristic()
      : BluetoothGattCharacteristic("", 0, 0) {}
  BluetoothGattCharacteristic(const std::string& uuid,
                              const int properties,
                              const int permissions)
      : BluetoothGattCharacteristic(uuid, properties, permissions, -1) {}
  BluetoothGattCharacteristic(const std::string& uuid,
                              const int properties,
                              const int permissions,
                              const int char_handle)
      : uuid(uuid)
      , properties(properties)
      , permissions(permissions)
      , char_handle(char_handle)
      , registered_for_notifications(false) {}
} BluetoothGattCharacteristic;

typedef struct BluetoothGattService {
  std::string uuid;
  std::vector<BluetoothGattCharacteristic> characteristics;
  int service_handle;
  int start_handle;
  int end_handle;
  BluetoothGattService()
      : BluetoothGattService("") {}
  BluetoothGattService(const std::string& uuid)
      : BluetoothGattService(uuid, -1, -1, -1) {}
  BluetoothGattService(const std::string& uuid, int service_handle, int start_handle, int end_handle)
      : uuid(uuid)
      , service_handle(service_handle)
      , start_handle(start_handle)
      , end_handle(end_handle) {}
} BluetoothGattService;

typedef struct BluetoothGattConnection {
  int conn_id;
  std::string address;
  std::vector<BluetoothGattService> services;
  BluetoothGattConnection()
      : BluetoothGattConnection("") {}
  BluetoothGattConnection(const std::string& address)
      : conn_id(-1)
      , address(address) {}
} BluetoothGattConnection;
