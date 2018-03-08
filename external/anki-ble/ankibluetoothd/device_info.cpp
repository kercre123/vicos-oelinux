/**
 * File: device_info.cpp
 *
 * Author: seichert
 * Created: 1/11/2018
 *
 * Description: Device Info functions for Anki Bluetooth Daemon
 *
 * Copyright: Anki, Inc. 2018
 *
 **/


#include <fstream>
#include <sstream>
#include <string>

#include <cutils/properties.h>

std::string GetDeviceSerialNumber()
{
  char value[PROPERTY_VALUE_MAX] = {0};
  (void) property_get("ro.serialno", value, "");

  if (value[0]) {
    return std::string(value);
  }

  std::ifstream in("/sys/devices/virtual/android_usb/android0/iSerial", std::ios::binary);
  if (!in.is_open()) {
    return std::string("");
  }
  std::ostringstream oss;
  oss << in.rdbuf();
  return oss.str();
}
