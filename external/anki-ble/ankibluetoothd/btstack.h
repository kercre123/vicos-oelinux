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
#include "bluetooth_gatt.h"
#include "btstack_callbacks.h"

#include <string>

namespace Anki {
namespace BluetoothStack {

bool LoadBtStack();
void UnLoadBtStack();
bool EnableAdapter();
std::string GetAdapterName();
bool SetAdapterName(const std::string& name);
bool RegisterGattClient();
bool RegisterGattServer();

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

bool StartScan(const bool enable, ScanResultCallback callback);

} // namespace BluetoothStack
} // namespace Anki
