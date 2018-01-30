/**
 * File: wifi.cpp
 *
 * Author: seichert
 * Created: 1/22/2018
 *
 * Description: Routines for scanning and configuring WiFi
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "wifi.h"
#include "exec_command.h"
#include "fileutils.h"
#include "log.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace Anki {

static int CalculateSignalLevel(int rssi, int numLevels)
{
  if (rssi <= kMinRssi) {
    return 0;
  } else if (rssi >= kMaxRssi) {
    return (numLevels - 1);
  } else {
    float inputRange = (kMaxRssi - kMinRssi);
    float outputRange = (numLevels - 1);
    return (int)((float)(rssi - kMinRssi) * outputRange / inputRange);
  }
}

static void ParseWiFiScanResults(const std::string& in, std::vector<WiFiScanResult>& outResults)
{
  outResults.clear();
  std::istringstream input(in);
  std::string header1;
  std::string header2;

  std::getline(input, header1);
  std::getline(input, header2);

  for (std::string line; std::getline(input, line); ) {
    std::stringstream linestream(line);
    std::string bssid;
    std::string frequency;
    std::string signal_level;
    std::string flags;
    std::string ssid;

    std::getline(linestream, bssid, '\t');
    if (!linestream.good()) break;
    std::getline(linestream, frequency, '\t');
    if (!linestream.good()) break;
    std::getline(linestream, signal_level, '\t');
    if (!linestream.good()) break;
    std::getline(linestream, flags, '\t');
    if (!linestream.good()) break;
    std::getline(linestream, ssid, '\t');
    if (!linestream.good() && !linestream.eof()) break;

    WiFiScanResult result;
    result.ssid = ssid;
    if (flags.find("[WPA2-EAP") != std::string::npos) {
      result.auth = WiFiAuth::AUTH_WPA2_EAP;
    } else if (flags.find("[WPA-EAP") != std::string::npos) {
      result.auth = WiFiAuth::AUTH_WPA_EAP;
    } else if (flags.find("[WPA2-PSK") != std::string::npos) {
      result.auth = WiFiAuth::AUTH_WPA2_PSK;
    } else if (flags.find("[WPA-PSK") != std::string::npos) {
      result.auth = WiFiAuth::AUTH_WPA_PSK;
    } else {
      result.auth = WiFiAuth::AUTH_NONE_OPEN;
    }

    if (flags.find("-CCMP") != std::string::npos) {
      result.encrypted = true;
    } else if (flags.find("-TKIP") != std::string::npos) {
      result.encrypted = false;
    } else if (flags.find("WEP") != std::string::npos) {
      result.encrypted = true;
      if (result.auth == WiFiAuth::AUTH_NONE_OPEN) {
        result.auth = WiFiAuth::AUTH_NONE_WEP;
      }
    } else {
      result.encrypted = false;
    }

    if (flags.find("[WPS") != std::string::npos) {
      result.wps = true;
    } else {
      result.wps = false;
    }

    result.signal_level = (uint8_t) CalculateSignalLevel(std::stoi(signal_level), 4);
    outResults.push_back(result);
  }

}

std::vector<WiFiScanResult> ScanForWiFiAccessPoints() {
  std::vector<WiFiScanResult> results;
  std::string output;
  int rc = ExecCommand({"wpa_cli", "scan"}, output);
  if (rc) {
    loge("Error scanning for WiFi access points. rc = %d", rc);
    return results;
  }
  rc = ExecCommand({"wpa_cli", "scan_results"}, output);
  if (rc) {
    loge("Error getting WiFi scan results. rc = %d", rc);
    return results;
  }
  ParseWiFiScanResults(output, results);
  return results;
}

std::vector<uint8_t> PackWiFiScanResults(const std::vector<WiFiScanResult>& results) {
  std::vector<uint8_t> packed_results;
  // Payload is (<auth><encrypted><wps><signal_level><ssid>\0)*
  for (auto const& r : results) {
    packed_results.push_back(r.auth);
    packed_results.push_back(r.encrypted);
    packed_results.push_back(r.wps);
    packed_results.push_back(r.signal_level);
    std::copy(r.ssid.begin(), r.ssid.end(), std::back_inserter(packed_results));
    packed_results.push_back(0);
  }
  return packed_results;
}

void EnableWiFiInterface(const bool enable, ExecCommandCallback callback) {
  if (enable) {
    ExecCommandInBackground({"ifconfig", "wlan0", "up"}, callback);
  } else {
    ExecCommandInBackground({"ifconfig", "wlan0", "down"}, callback);
  }
  ExecCommandInBackground({"ifconfig", "wlan0"}, callback);
}

static std::string GetPathToWiFiConfigFile()
{
  return "/data/misc/wifi/wpa_supplicant.conf";
}

static std::string GetPathToWiFiDefaultConfigFile()
{
  return "/system/etc/wpa_supplicant_default.conf";
}

static std::string GetWiFiDefaultConfig()
{
  std::string defaultConfigPath = GetPathToWiFiDefaultConfigFile();
  std::ifstream istrm(defaultConfigPath, std::ios::binary | std::ios::in);
  if (istrm.is_open()) {
    std::ostringstream ss;
    ss << istrm.rdbuf();
    return ss.str();
  } else {
    const char* defaultConfig = R"foo(ctrl_interface=/var/run/wpa_supplicant
update_config=1
p2p_no_group_iface=1

)foo";
    return defaultConfig;
  }
}
std::map<std::string, std::string> UnPackWiFiConfig(const std::vector<uint8_t>& packed) {
  std::map<std::string, std::string> networks;
  // The payload is (<SSID>\0<PSK>\0)*
  for (auto it = packed.begin(); it != packed.end(); ) {
    auto terminator = std::find(it, packed.end(), 0);
    if (terminator == packed.end()) {
      break;
    }
    std::string ssid(it, terminator);
    it = terminator + 1;
    terminator = std::find(it, packed.end(), 0);
    if (terminator == packed.end()) {
      break;
    }
    std::string psk(it, terminator);
    networks.emplace(ssid, psk);
    it = terminator + 1;
  }
  return networks;
}

void SetWiFiConfig(const std::map<std::string, std::string> networks, ExecCommandCallback callback) {
  std::ostringstream wifiConfigStream;

  wifiConfigStream << GetWiFiDefaultConfig();

  for (auto const& kv : networks) {
    wifiConfigStream << "network={" << std::endl
                     << "\tssid=\"" << kv.first << "\"" << std::endl
                     << "\tpsk=\"" << kv.second << "\"" << std::endl
                     << "}" << std::endl << std::endl;
  }

  int rc = WriteFileAtomically(GetPathToWiFiConfigFile(), wifiConfigStream.str());

  if (rc) {
    std::string error = "Failed to write wifi config. rc = " + std::to_string(rc);
    callback(rc, "Failed to write wifi config.");
    return;
  }

  ExecCommandInBackground({"wpa_cli", "reconfigure"}, callback);
  ExecCommandInBackground({"wpa_cli", "enable", "0"}, callback);
  ExecCommandInBackground({"echo", "Give me about 10 seconds to setup WiFi...."}, callback);
  ExecCommandInBackground({"wpa_cli", "status"}, callback, 10L * 1000L);
}


} // namespace Anki
