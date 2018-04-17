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


void SetCallbacks(Callbacks* cbs);
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
void DisconnectGattPeerByAddress(const std::string& address);
bool RemoveGattService(BluetoothGattService* service);
bool SendGattIndication(int attribute_handle, int conn_id, int confirm,
                        const std::vector<uint8_t>& value);
bool WriteGattCharacteristic(const int conn_id,
                             const std::string& uuid,
                             const bool reliable,
                             const std::vector<uint8_t>& value);
bool ReadGattCharacteristic(const int conn_id,
                            const std::string& uuid);
bool ReadGattDescriptor(const int conn_id,
                        const std::string& char_uuid,
                        const std::string& desc_uuid);
bool SendResponse(int conn_id, int trans_id, int handle, int error, int offset,
                  const std::vector<uint8_t>& value);
bool StartAdvertisement(const Anki::BLEAdvertiseSettings& settings);
bool StopAdvertisement();
void RestartAdvertisement();

bool SetScanning(const bool enable);
bool ConnectToBLEPeripheral(const std::string& address, const bool is_direct);

int RequestConnectionParameterUpdate(const std::string& address,
                                     int min_interval,
                                     int max_interval,
                                     int latency,
                                     int timeout);


} // namespace BluetoothStack
} // namespace Anki
