/**
 * File: wifi.h
 *
 * Author: seichert
 * Created: 1/22/2018
 *
 * Description: Routines for scanning and configuring WiFi
 *
 * Copyright: Anki, Inc. 2018
 *
 **/


#pragma once

#include "exec_command.h"

#include <map>
#include <string>
#include <vector>


namespace Anki {

enum WiFiAuth : uint8_t {
      AUTH_NONE_OPEN       = 0,
      AUTH_NONE_WEP        = 1,
      AUTH_NONE_WEP_SHARED = 2,
      AUTH_IEEE8021X       = 3,
      AUTH_WPA_PSK         = 4,
      AUTH_WPA_EAP         = 5,
      AUTH_WPA2_PSK        = 6,
      AUTH_WPA2_EAP        = 7
};

class WiFiScanResult {
 public:
  WiFiAuth    auth;
  bool        encrypted;
  bool        wps;
  uint8_t     signal_level;
  std::string ssid;
};

class WiFiConfig {
 public:
  WiFiAuth auth;
  bool     hidden;
  std::string ssid; /* hexadecimal representation of ssid name */
  std::string passphrase;
};

std::vector<WiFiScanResult> ScanForWiFiAccessPoints();
std::vector<uint8_t> PackWiFiScanResults(const std::vector<WiFiScanResult>& results);
void EnableWiFiInterface(const bool enable, ExecCommandCallback callback);
std::vector<WiFiConfig> UnPackWiFiConfig(const std::vector<uint8_t>& packed);
void SetWiFiConfig(const std::vector<WiFiConfig>& networks, ExecCommandCallback);

} // namespace Anki
