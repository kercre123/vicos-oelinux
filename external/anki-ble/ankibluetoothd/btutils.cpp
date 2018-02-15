/**
 * File: btutils.cpp
 *
 * Author: seichert
 * Created: 1/11/2018
 *
 * Description: Bluetooth utility functions
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "btutils.h"
#include "gatt_constants.h"
#include "stringutils.h"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>

static uint8_t hex_char_to_byte(char input) {
  if (input >= '0' && input <= '9') {
    return input - '0';
  }
  if (input >= 'A' && input <= 'F') {
    return input - 'A' + 10;
  }
  if (input >= 'a' && input <= 'f') {
    return input - 'a' + 10;
  }
  return 0;
}

std::string bt_value_to_string(int length, uint8_t* value) {
  if (!value) {
    return "<null>";
  }
  if (length < 1) {
    return "";
  }
  std::vector<uint8_t> v(value, value + length);
  return byteVectorToHexString(v, 1, true);
}

std::string bt_bdaddr_t_to_string(const bt_bdaddr_t* addr) {
  char str[18];

  const uint8_t* ptr = addr->address;
  snprintf(str, sizeof(str), "%02x:%02x:%02x:%02x:%02x:%02x",
           ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);

  return std::string(str);
}

std::string bt_uuid_t_to_string(const bt_uuid_t* uuid) {
  char str[37];

  const uint8_t* u = uuid->uu;
  snprintf(str, sizeof(str),
           "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
           u[15], u[14], u[13], u[12],
           u[11], u[10],
           u[9], u[8],
           u[7], u[6],
           u[5], u[4], u[3], u[2], u[1], u[0]);

  return std::string(str);
}

void bt_uuid_t_from_string(const std::string& uuidStr, bt_uuid_t* uuid) {
  memset(uuid, 0, sizeof(*uuid));
  int index = 15;
  int offset = 0;
  while (index >= 0 && offset < (uuidStr.length() - 1)) {
    char high = uuidStr[offset++];
    if (std::isxdigit(high)) {
      uint8_t val = hex_char_to_byte(high) * 16;
      char low = uuidStr[offset++];
      val += hex_char_to_byte(low);
      uuid->uu[index--] = val;
    }
  }
}

bool bt_uuid_t_equals(const bt_uuid_t* uuid1, const bt_uuid_t* uuid2) {
  return (0 == memcmp(uuid1->uu, uuid2->uu, sizeof(uuid1->uu)));
}

std::string bt_status_t_to_string(const bt_status_t status) {
  switch(status) {
    case BT_STATUS_SUCCESS:
      return "BT_STATUS_SUCCESS";
    case BT_STATUS_FAIL:
      return "BT_STATUS_FAIL";
    case BT_STATUS_NOT_READY:
      return "BT_STATUS_NOT_READY";
    case BT_STATUS_NOMEM:
      return "BT_STATUS_NOMEM";
    case BT_STATUS_BUSY:
      return "BT_STATUS_BUSY";
    case BT_STATUS_DONE:
      return "BT_STATUS_DONE";
    case BT_STATUS_UNSUPPORTED:
      return "BT_STATUS_UNSUPPORTED";
    case BT_STATUS_PARM_INVALID:
      return "BT_STATUS_PARM_INVALID";
    case BT_STATUS_UNHANDLED:
      return "BT_STATUS_UNHANDLED";
    case BT_STATUS_AUTH_FAILURE:
      return "BT_STATUS_AUTH_FAILURE";
    case BT_STATUS_RMT_DEV_DOWN:
      return "BT_STATUS_RMT_DEV_DOWN";
    case BT_STATUS_AUTH_REJECTED:
      return "BT_STATUS_AUTH_REJECTED";
    case BT_STATUS_JNI_ENVIRONMENT_ERROR:
      return "BT_STATUS_JNI_ENVIRONMENT_ERROR";
    case BT_STATUS_JNI_THREAD_ATTACH_ERROR:
      return "BT_STATUS_JNI_THREAD_ATTACH_ERROR";
    case BT_STATUS_WAKELOCK_ERROR:
      return "BT_STATUS_WAKELOCK_ERROR";
    default:
      {
        if ((int) status == 0x85) {
          return "GATT_ERROR";
        }
        std::ostringstream oss;
        oss << "Unknown (" << (int) status << ")";
        return oss.str();
      }
  }
}


std::string bt_gatt_error_to_string(const int error)
{
  switch(error)
  {
    case kGattErrorNone:
      return "kGattErrorNone";
    case kGattErrorInvalidHandle:
      return "kGattErrorInvalidHandle";
    case kGattErrorReadNotPermitted:
      return "kGattErrorReadNotPermitted";
    case kGattErrorWriteNotPermitted:
      return "kGattErrorWriteNotPermitted";
    case kGattErrorInvalidPDU:
      return "kGattErrorInvalidPDU";
    case kGattErrorInsufficientAuthen:
      return "kGattErrorInsufficientAuthen";
    case kGattErrorRequestNotSupported:
      return "kGattErrorRequestNotSupported";
    case kGattErrorInvalidOffset:
      return "kGattErrorInvalidOffset";
    case kGattErrorInsufficientAuthor:
      return "kGattErrorInsufficientAuthor";
    case kGattErrorPrepQueueFull:
      return "kGattErrorPrepQueueFull";
    case kGattErrorAttributeNotFound:
      return "kGattErrorAttributeNotFound";
    case kGattErrorAttributeNotLong:
      return "kGattErrorAttributeNotLong";
    case kGattErrorInsufficientKeySize:
      return "kGattErrorInsufficientKeySize";
    case kGattErrorInvalidAttributeLength:
      return "kGattErrorInvalidAttributeLength";
    case kGattErrorUnlikely:
      return "kGattErrorUnlikely";
    case kGattErrorInsufficientEncr:
      return "kGattErrorInsufficientEncr";
    case kGattErrorUnsupportedGrpType:
      return "kGattErrorUnsupportedGrpType";
    case kGattErrorInsufficientResources:
      return "kGattErrorInsufficientResources";
    case kGattErrorIllegalParameter:
      return "kGattErrorIllegalParameter";
    case kGattErrorNoResources:
      return "kGattErrorNoResources";
    case kGattErrorInternalError:
      return "kGattErrorInternalError";
    case kGattErrorWrongState:
      return "kGattErrorWrongState";
    case kGattErrorDbFull:
      return "kGattErrorDbFull";
    case kGattErrorBusy:
      return "kGattErrorBusy";
    case kGattErrorError:
      return "kGattErrorError";
    case kGattErrorCmdStarted:
      return "kGattErrorCmdStarted";
    case kGattErrorPending:
      return "kGattErrorPending";
    case kGattErrorAuthFail:
      return "kGattErrorAuthFail";
    case kGattErrorMore:
      return "kGattErrorMore";
    case kGattErrorInvalidConfig:
      return "kGattErrorInvalidConfig";
    case kGattErrorServiceStarted:
      return "kGattErrorServiceStarted";
    case kGattErrorEncryptedNoMITM:
      return "kGattErrorEncryptedNoMITM";
    case kGattErrorNotEncrypted:
      return "kGattErrorNotEncrypted";
    case kGattErrorCongested:
      return "kGattErrorCongested";
    case kGattErrorCCCDImproperlyConfigured:
      return "kGattErrorCCCDImproperlyConfigured";
    case kGattErrorProcedureInProgress:
      return "kGattErrorProcedureInProgress";
    case kGattErrorOutOfRange:
      return "kGattErrorOutOfRange";
    default:
      {
        std::ostringstream oss;
        oss << "Unknown (" << error << ")";
        return oss.str();
      }
  }
}

const char* bt_scan_mode_t_to_string(const bt_scan_mode_t mode) {
  switch(mode) {
    case BT_SCAN_MODE_NONE:
      return "BT_SCAN_MODE_NONE";
    case BT_SCAN_MODE_CONNECTABLE:
      return "BT_SCAN_MODE_CONNECTABLE";
    case BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE:
      return "BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE";
    default:
      return "Unknown";
  }
}

const char* bt_state_t_to_string(const bt_state_t state) {
  switch(state) {
    case BT_STATE_OFF:
      return "BT_STATE_OFF";
    case BT_STATE_ON:
      return "BT_STATE_ON";
    default:
      return "Unknown";
  }
}

const char* bt_cb_thread_evt_to_string(const bt_cb_thread_evt evt) {
  switch(evt) {
    case ASSOCIATE_JVM:
      return "ASSOCIATE_JVM";
    case DISASSOCIATE_JVM:
      return "DISASSOCIATE_JVM";
    default:
      return "Unknown";
  }
}

const char* bt_property_type_t_to_string(const bt_property_type_t property_type) {
  switch(property_type) {
    case BT_PROPERTY_BDNAME:
      return "BT_PROPERTY_BDNAME";
    case BT_PROPERTY_BDADDR:
      return "BT_PROPERTY_BDADDR";
    case BT_PROPERTY_UUIDS:
      return "BT_PROPERTY_UUIDS";
    case BT_PROPERTY_CLASS_OF_DEVICE:
      return "BT_PROPERTY_CLASS_OF_DEVICE";
    case BT_PROPERTY_TYPE_OF_DEVICE:
      return "BT_PROPERTY_TYPE_OF_DEVICE";
    case BT_PROPERTY_SERVICE_RECORD:
      return "BT_PROPERTY_SERVICE_RECORD";
    case BT_PROPERTY_ADAPTER_SCAN_MODE:
      return "BT_PROPERTY_ADAPTER_SCAN_MODE";
    case BT_PROPERTY_ADAPTER_BONDED_DEVICES:
      return "BT_PROPERTY_ADAPTER_BONDED_DEVICES";
    case BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT:
      return "BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT";
    case BT_PROPERTY_REMOTE_FRIENDLY_NAME:
      return "BT_PROPERTY_REMOTE_FRIENDLY_NAME";
    case BT_PROPERTY_REMOTE_RSSI:
      return "BT_PROPERTY_REMOTE_RSSI";
    case BT_PROPERTY_REMOTE_VERSION_INFO:
      return "BT_PROPERTY_REMOTE_VERSION_INFO";
    case BT_PROPERTY_LOCAL_LE_FEATURES:
      return "BT_PROPERTY_LOCAL_LE_FEATURES";
    case BT_PROPERTY_REMOTE_DEVICE_TIMESTAMP:
      return "BT_PROPERTY_REMOTE_DEVICE_TIMESTAMP";
    default:
      return "Unknown";
  }
}

const char* bt_device_type_t_to_string(const bt_device_type_t type) {
  switch(type) {
    case BT_DEVICE_DEVTYPE_BREDR:
      return "BT_DEVICE_DEVTYPE_BREDR";
    case BT_DEVICE_DEVTYPE_BLE:
      return "BT_DEVICE_DEVTYPE_BLE";
    case BT_DEVICE_DEVTYPE_DUAL:
      return "BT_DEVICE_DEVTYPE_DUAL";
    default:
      return "Unknown";
  }
}

std::string bt_acl_state_t_to_string(const bt_acl_state_t state) {
  switch(state) {
    case BT_ACL_STATE_CONNECTED:
      return "BT_ACL_STATE_CONNECTED";
    case BT_ACL_STATE_DISCONNECTED:
      return "BT_ACL_STATE_DISCONNECTED";
    default:
      {
        std::ostringstream oss;
        oss << "Unknown (" << (int) state << ")";
        return oss.str();
      }
  }

}

std::vector<std::string> bt_16bit_service_uuid_array_to_vector(const std::vector<uint8_t>& data)
{
  std::vector<std::string> uuids;

  size_t sz = data.size();
  size_t offset = 0;
  while ((offset + 2) <= sz) {
    std::vector<uint8_t> v(data.begin() + offset, data.begin() + offset + 2);
    std::reverse(std::begin(v), std::end(v));
    uuids.push_back(byteVectorToHexString(v));
    offset += 2;
  }
  return uuids;
}

std::vector<std::string> bt_128bit_service_uuid_array_to_vector(const std::vector<uint8_t>& data)
{
  std::vector<std::string> uuids;

  size_t sz = data.size();
  size_t offset = 0;
  while ((offset + 16) <= sz) {
    std::vector<uint8_t> v(data.begin() + offset, data.begin() + offset + 16);
    std::reverse(std::begin(v), std::end(v));
    auto it = v.begin();
    std::vector<int> lengths = {4,2,2,2,6};
    std::string uuidString;
    for (auto l : lengths) {
      if (it != v.begin()) {
        uuidString.push_back('-');
      }
      std::vector<uint8_t> part(it, it + l);
      uuidString += byteVectorToHexString(part);
      it += l;
    }
    uuids.push_back(uuidString);
    offset += 16;
  }
  return uuids;
}
