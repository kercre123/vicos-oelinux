/**
 * File: btutils.h
 *
 * Author: seichert
 * Created: 1/11/2018
 *
 * Description: Bluetooth utility functions
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once
#include <hardware/bluetooth.h>
#include <string>
#include <vector>

std::string bt_value_to_string(int length, const uint8_t* value);
std::string bt_bdaddr_t_to_string(const bt_bdaddr_t* addr);
void bt_bdaddr_t_from_string(const std::string& address, bt_bdaddr_t* bda);
std::string bt_uuid_t_to_string(const bt_uuid_t* uuid);
void bt_uuid_t_from_string(const std::string& uuidStr, bt_uuid_t* uuid);
std::vector<uint8_t> byte_vector_from_uuid_string(const std::string& uuidStr);
bool bt_uuid_t_equals(const bt_uuid_t* uuid1, const bt_uuid_t* uuid2);
bool bt_uuid_string_equals(const std::string& uuidStr1, const std::string& uuidStr2);
std::string bt_status_t_to_string(const bt_status_t status);
std::string bt_gatt_error_to_string(const int error);
const char* bt_scan_mode_t_to_string(const bt_scan_mode_t mode);
const char* bt_state_t_to_string(const bt_state_t state);
const char* bt_cb_thread_evt_to_string(const bt_cb_thread_evt evt);
const char* bt_property_type_t_to_string(const bt_property_type_t property_type);
const char* bt_device_type_t_to_string(const bt_device_type_t type);
std::string bt_acl_state_t_to_string(const bt_acl_state_t state);
std::string bt_bond_state_t_to_string(const bt_bond_state_t state);
