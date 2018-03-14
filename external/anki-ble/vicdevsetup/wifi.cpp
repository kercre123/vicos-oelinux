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
#include "connmanbus.h"
#include "exec_command.h"
#include "fileutils.h"
#include "log.h"
#include "stringutils.h"

#include <glib.h>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace Anki {

static int CalculateSignalLevel(const int strength, const int min, const int max, const int numLevels) {
  if (strength < min) {
    return 0;
  } else if (strength >= max) {
    return (numLevels - 1);
  } else {
    float inputRange = (max - min);
    float outputRange = (numLevels - 1);
    return (int)((float)(strength - min) * outputRange / inputRange);
  }
}

std::vector<WiFiScanResult> ScanForWiFiAccessPoints() {
  std::vector<WiFiScanResult> results;
  ConnManBusTechnology* tech_proxy;
  GError* error;

  error = nullptr;
  tech_proxy = conn_man_bus_technology_proxy_new_for_bus_sync(G_BUS_TYPE_SYSTEM,
                                                              G_DBUS_PROXY_FLAGS_NONE,
                                                              "net.connman",
                                                              "/net/connman/technology/wifi",
                                                              nullptr,
                                                              &error);
  if (error) {
    loge("error getting proxy for net.connman /net/connman/technology/wifi");
    return results;
  }

  gboolean success = conn_man_bus_technology_call_scan_sync(tech_proxy,
                                                            nullptr,
                                                            &error);
  g_object_unref(tech_proxy);
  if (error) {
    loge("error asking connman to scan for wifi access points");
    return results;
  }

  if (!success) {
    loge("connman failed to scan for wifi access points");
    return results;
  }

  ConnManBusManager* manager_proxy;
  manager_proxy = conn_man_bus_manager_proxy_new_for_bus_sync(G_BUS_TYPE_SYSTEM,
                                                              G_DBUS_PROXY_FLAGS_NONE,
                                                              "net.connman",
                                                              "/",
                                                              nullptr,
                                                              &error);
  if (error) {
    loge("error getting proxy for net.connman /");
    return results;
  }

  GVariant* services = nullptr;
  success = conn_man_bus_manager_call_get_services_sync(manager_proxy,
                                                        &services,
                                                        nullptr,
                                                        &error);
  g_object_unref(manager_proxy);
  if (error) {
    loge("Error getting services from connman");
    return results;
  }

  if (!success) {
    loge("connman failed to get list of services");
    return results;
  }

  for (gsize i = 0 ; i < g_variant_n_children(services); i++) {
    WiFiScanResult result{WiFiAuth::AUTH_NONE_OPEN, false, false, 0, ""};
    GVariant* child = g_variant_get_child_value(services, i);
    GVariant* attrs = g_variant_get_child_value(child, 1);
    bool type_is_wifi = false;
    bool iface_is_wlan0 = false;

    for (gsize j = 0 ; j < g_variant_n_children(attrs); j++) {
      GVariant* attr = g_variant_get_child_value(attrs, j);
      GVariant* key_v = g_variant_get_child_value(attr, 0);
      GVariant* val_v = g_variant_get_child_value(attr, 1);
      GVariant* val = g_variant_get_variant(val_v);
      const char* key = g_variant_get_string(key_v, nullptr);

      // Make sure this is a wifi service and not something else
      if (g_str_equal(key, "Type")) {
        if (g_str_equal(g_variant_get_string(val, nullptr), "wifi")) {
          type_is_wifi = true;
        } else {
          type_is_wifi = false;
          break;
        }
      }

      // Make sure this is for the wlan0 interface and not p2p0
      if (g_str_equal(key, "Ethernet")) {
        for (gsize k = 0 ; k < g_variant_n_children(val); k++) {
          GVariant* ethernet_attr = g_variant_get_child_value(val, k);
          GVariant* ethernet_key_v = g_variant_get_child_value(ethernet_attr, 0);
          GVariant* ethernet_val_v = g_variant_get_child_value(ethernet_attr, 1);
          GVariant* ethernet_val = g_variant_get_variant(ethernet_val_v);
          const char* ethernet_key = g_variant_get_string(ethernet_key_v, nullptr);
          if (g_str_equal(ethernet_key, "Interface")) {
            if (g_str_equal(g_variant_get_string(ethernet_val, nullptr), "wlan0")) {
              iface_is_wlan0 = true;
            } else {
              iface_is_wlan0 = false;
              break;
            }
          }
        }
      }

      if (g_str_equal(key, "Name")) {
        result.ssid = std::string(g_variant_get_string(val, nullptr));
      }

      if (g_str_equal(key, "Strength")) {
        result.signal_level =
            (uint8_t) CalculateSignalLevel((int) g_variant_get_byte(val),
                                           0, // min
                                           100, // max
                                           4 /* number of levels */);
      }

      if (g_str_equal(key, "Security")) {
        for (gsize k = 0 ; k < g_variant_n_children(val); k++) {
          GVariant* security_val = g_variant_get_child_value(val, k);
          const char* security_val_str = g_variant_get_string(security_val, nullptr);
          if (g_str_equal(security_val_str, "wps")) {
            result.wps = true;
          }
          if (g_str_equal(security_val_str, "none")) {
            result.auth = WiFiAuth::AUTH_NONE_OPEN;
            result.encrypted = false;
          }
          if (g_str_equal(security_val_str, "wep")) {
            result.auth = WiFiAuth::AUTH_NONE_WEP;
            result.encrypted = true;
          }
          if (g_str_equal(security_val_str, "ieee8021x")) {
            result.auth = WiFiAuth::AUTH_IEEE8021X;
            result.encrypted = true;
          }
          if (g_str_equal(security_val_str, "psk")) {
            result.auth = WiFiAuth::AUTH_WPA2_PSK;
            result.encrypted = true;
          }
        }
      }

    }

    if (type_is_wifi && iface_is_wlan0) {
      results.push_back(result);
    }
  }
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
    ExecCommandInBackground({"connmanctl", "enable", "wifi"}, callback);
    ExecCommandInBackground({"echo", "Give me about 15 seconds to start WiFi and get an IP...."}, callback);
    ExecCommandInBackground({"ifconfig", "wlan0"}, callback, 15L * 1000L);
  } else {
    ExecCommandInBackground({"connmanctl", "disable", "wifi"}, callback);
    ExecCommandInBackground({"ifconfig", "wlan0"}, callback);
  }
}

static std::string GetPathToWiFiConfigFile()
{
  return "/data/lib/connman/wifi.config";
}

std::vector<WiFiConfig> UnPackWiFiConfig(const std::vector<uint8_t>& packed) {
  std::vector<WiFiConfig> networks;
  // The payload is (<auth><hidden><ssid>\0<psk>\0)*
  for (auto it = packed.begin(); it != packed.end(); ) {
    WiFiConfig config;
    config.auth = (WiFiAuth) *it;
    it++;
    config.hidden = (bool) *it;
    it++;
    auto terminator = std::find(it, packed.end(), 0);
    if (terminator == packed.end()) {
      break;
    }
    config.ssid = std::string(it, terminator);
    it = terminator + 1;
    terminator = std::find(it, packed.end(), 0);
    if (terminator == packed.end()) {
      break;
    }
    config.passphrase = std::string(it, terminator);
    networks.push_back(config);
    it = terminator + 1;
  }
  return networks;
}

void SetWiFiConfig(const std::vector<WiFiConfig>& networks, ExecCommandCallback callback) {
  std::ostringstream wifiConfigStream;

  int count = 0;
  for (auto const& config : networks) {
    if (count > 0) {
      wifiConfigStream << std::endl;
    }
    // Exclude networks with ssids that are not in hex format
    if (!IsHexString(config.ssid)) {
      loge("SetWiFiConfig. '%s' is NOT a hexadecimal string.", config.ssid.c_str());
      continue;
    }
    // Exclude networks with unsupported auth types
    if ((config.auth != WiFiAuth::AUTH_NONE_OPEN)
        && (config.auth != WiFiAuth::AUTH_NONE_WEP)
        && (config.auth != WiFiAuth::AUTH_NONE_WEP_SHARED)
        && (config.auth != WiFiAuth::AUTH_WPA_PSK)
        && (config.auth != WiFiAuth::AUTH_WPA2_PSK)) {
      loge("SetWiFiConfig. Unsupported auth type : %d for '%s'",
           config.auth,
           hexStringToAsciiString(config.ssid).c_str());
      continue;
    }

    std::string security;
    switch (config.auth) {
      case WiFiAuth::AUTH_NONE_WEP:
        /* fall through */
      case WiFiAuth::AUTH_NONE_WEP_SHARED:
        security = "wep";
        break;
      case WiFiAuth::AUTH_WPA_PSK:
        /* fall through */
      case WiFiAuth::AUTH_WPA2_PSK:
        security = "psk";
        break;
      case WiFiAuth::AUTH_NONE_OPEN:
        /* fall through */
      default:
        security = "none";
        break;
    }

    std::string hidden(config.hidden ? "true" : "false");
    wifiConfigStream << "[service_wifi_" << count++ << "]" << std::endl
                     << "Type = wifi" << std::endl
                     << "IPv4 = dhcp" << std::endl
                     << "IPv6 = auto" << std::endl
                     << "SSID=" << config.ssid << std::endl
                     << "Security=" << security << std::endl
                     << "Hidden=" << hidden << std::endl;
    if (!config.passphrase.empty()) {
      wifiConfigStream << "Passphrase=" << config.passphrase << std::endl;
    }
  }

  int rc = WriteFileAtomically(GetPathToWiFiConfigFile(), wifiConfigStream.str());

  if (rc) {
    std::string error = "Failed to write wifi config. rc = " + std::to_string(rc);
    callback(rc, "Failed to write wifi config.");
    return;
  }

  ExecCommandInBackground({"connmanctl", "enable", "wifi"}, callback);
  ExecCommandInBackground({"echo", "Give me about 15 seconds to setup WiFi...."}, callback);
  ExecCommandInBackground({"ifconfig", "wlan0"}, callback, 15L * 1000L);
}


} // namespace Anki
