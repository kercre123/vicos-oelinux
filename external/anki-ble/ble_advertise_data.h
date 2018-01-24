/**
 * File: ble_advertise_data.h
 *
 * Author: seichert
 * Created: 1/23/2018
 *
 * Description: Bluetooth Low Energy Advertising Settings
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include <string>
#include <vector>

namespace Anki {

class BLEAdvertiseData {
 public:
  BLEAdvertiseData()
      : include_name_(false)
      , include_tx_power_(false)
  { }
  ~BLEAdvertiseData() = default;

  void SetIncludeName(const bool enable) { include_name_ = enable; }
  bool GetIncludeName() const { return include_name_; }

  void SetIncludeTxPower(const bool enable) { include_tx_power_ = enable; }
  bool GetIncludeTxPower() const { return include_tx_power_; }

  void SetManufacturerData(const std::vector<uint8_t>& data) { manufacturer_data_ = data; }
  const std::vector<uint8_t>& GetManufacturerData() const { return manufacturer_data_; }
  std::vector<uint8_t>& GetManufacturerData() { return manufacturer_data_; }

  void SetServiceData(const std::vector<uint8_t>& data) { service_data_ = data; }
  const std::vector<uint8_t>& GetServiceData() const { return service_data_; }
  std::vector<uint8_t>& GetServiceData() { return service_data_; }

  void SetServiceUUID(const std::string uuid) {service_uuid_ = uuid;}
  const std::string& GetServiceUUID() const { return service_uuid_; }

 private:
  bool include_name_;
  bool include_tx_power_;
  std::vector<uint8_t> manufacturer_data_;
  std::vector<uint8_t> service_data_;
  std::string service_uuid_;
};

} // namespace Anki
