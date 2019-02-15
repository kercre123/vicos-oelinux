/**
 * File: btstack.cpp
 *
 * Author: seichert
 * Created: 1/10/2018
 *
 * Description: Bluetooth Stack functions for Anki Bluetooth Daemon
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "btstack.h"

#include "anki_ble_uuids.h"
#include "codeTimer.h"
#include "log.h"
#include "btutils.h"
#include "gatt_constants.h"
#include "strlcpy.h"
#include "taskExecutor.h"
#include <hardware/bluetooth.h>
#include <hardware/bt_gatt.h>
#include <hardware/bt_gatt_client.h>
#include <hardware/bt_gatt_server.h>
#include <hardware/hardware.h>
#include <hardware/vendor.h>
#include <string.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

namespace Anki {
namespace BluetoothStack {

#ifdef __cplusplus
extern "C" {
#endif

static struct ev_loop* sDefaultLoop = ev_default_loop(EVBACKEND_SELECT);
static TaskExecutor sTaskExecutor(sDefaultLoop);

static struct Callbacks sCallbacks = {0};

/* To register for the client and server GATT interface, we need to provide uuids.
 * These uuids were generated with uuidgen on the desktop to avoid having to generate
 * them in code.
 */
static bt_uuid_t sGattClientUUID = {0x11, 0xd3, 0xc1, 0xd7, 0xac, 0x93, 0x38, 0xa1,
                                    0x6c, 0x47, 0x87, 0xfc, 0x54, 0x16, 0xd0, 0x03};
static bt_uuid_t sGattServerUUID = {0x25, 0x3b, 0xbe, 0x5a, 0x29, 0x45, 0xe9, 0x84,
                                    0xd0, 0x48, 0xfc, 0xb6, 0xeb, 0xd2, 0x90, 0xb6};

static struct hw_device_t *sDevice;
static bluetooth_device_t *sBtDevice;
static const bt_interface_t *sBtInterface;
static const btvendor_interface_t *sBtVendorInterface;
static btgatt_interface_t *sBtGattInterface;

static std::string sBtAdapterName;

static std::mutex sBtStackCallbackMutex;
static std::condition_variable sBtStackCallbackCV;
static bool sBtStackCallbackReceived = false;
static bool sBtGattServiceAdded = false;
static bool sBtGattServiceStarted = false;
static bool sBtGattOpError = false;
static bool sBtAdvertisingEnabled = false;


static bool sBtAdapterEnabled = false;
static int sBtGattClientIf;
static int sBtGattServerIf;

static BluetoothGattService* sBluetoothGattService;

typedef struct BluetoothGattWriteItem {
  bool is_descriptor;
  int conn_id;
  int handle;
  int write_type;
  std::vector<uint8_t> value;
  BluetoothGattWriteItem(bool is_descriptor,
                         int conn_id,
                         int handle,
                         int write_type,
                         const std::vector<uint8_t>& value)
      : is_descriptor(is_descriptor)
      , conn_id(conn_id)
      , handle(handle)
      , write_type(write_type)
      , value(value) {}
} BluetoothGattWriteItem;

typedef struct BluetoothGattReadItem {
  bool is_descriptor;
  int conn_id;
  int handle;
  BluetoothGattReadItem(bool is_descriptor,
                        int conn_id,
                        int handle)
      : is_descriptor(is_descriptor)
      , conn_id(conn_id)
      , handle(handle) {}
} BluetoothGattReadItem;

static std::deque<BluetoothGattWriteItem> sGattWriteQueue;
static bool sGattWriteClearToSend = true;
static std::deque<BluetoothGattReadItem> sGattReadQueue;
static bool sGattReadClearToSend = true;
static bool sCongested = false;

enum class BluetoothGattConnectionStatus {
  Invalid = 0,
    Connecting,
    DiscoveringServices,
    GettingGattDb,
    RegisteringForNotifications,
    Connected,
    Disconnecting
};

static std::string BluetoothGattConnectionStatusToString(const BluetoothGattConnectionStatus status)
{
  switch(status) {
    case BluetoothGattConnectionStatus::Invalid:
      return "Invalid";
      break;
    case BluetoothGattConnectionStatus::Connecting:
      return "Connecting";
      break;
    case BluetoothGattConnectionStatus::DiscoveringServices:
      return "DiscoveringServices";
      break;
    case BluetoothGattConnectionStatus::GettingGattDb:
      return "GettingGattDb";
      break;
    case BluetoothGattConnectionStatus::RegisteringForNotifications:
      return "RegisteringForNotifications";
      break;
    case BluetoothGattConnectionStatus::Connected:
      return "Connected";
      break;
    case BluetoothGattConnectionStatus::Disconnecting:
      return "Disconnecting";
      break;
    default:
      return std::to_string(static_cast<int>(status));
      break;
  }

}

typedef struct BluetoothGattConnectionInfo {
  Anki::Util::CodeTimer::TimePoint start;
  BluetoothGattConnectionStatus status;
  BluetoothGattConnection connection;
  BluetoothGattConnectionInfo()
      : BluetoothGattConnectionInfo("") {}
  BluetoothGattConnectionInfo(const std::string &address)
      : connection(BluetoothGattConnection(address))
      , status(BluetoothGattConnectionStatus::Connecting)
      , start(Anki::Util::CodeTimer::Start()) {}
} BluetoothGattConnectionInfo;

static std::map<std::string, BluetoothGattConnectionInfo> sOutboundConnections;

static std::map<std::string, BluetoothGattConnectionInfo>::iterator
FindOutboundConnectionById(const int conn_id)
{
  auto search =
      std::find_if(sOutboundConnections.begin(),
                   sOutboundConnections.end(),
           [conn_id] (const std::pair<std::string, BluetoothGattConnectionInfo>& p) -> bool {
                     return (p.second.connection.conn_id == conn_id);
                   });
  return search;
}

static BluetoothGattDescriptor*
FindBluetoothGattDescriptorByConnectionAndHandle(BluetoothGattConnection& connection,
                                                 const int handle)
{
  for (auto & service : connection.services) {
    for (auto & characteristic : service.characteristics) {
      for (auto & descriptor : characteristic.descriptors) {
        if (descriptor.desc_handle == handle) {
          return &descriptor;
        }
      }
    }
  }
  return nullptr;
}

static void EraseGattQueueItemsByConnId(const int conn_id) {
  sGattWriteQueue.erase(std::remove_if(sGattWriteQueue.begin(),
                                       sGattWriteQueue.end(),
                                       [conn_id](const BluetoothGattWriteItem &i){return conn_id == i.conn_id;}),
                        sGattWriteQueue.end());
  if (sGattWriteQueue.empty()) {
    sGattWriteClearToSend = true;
  }
  sGattReadQueue.erase(std::remove_if(sGattReadQueue.begin(),
                                      sGattReadQueue.end(),
                                      [conn_id](const BluetoothGattReadItem &i){return conn_id == i.conn_id;}),
                       sGattReadQueue.end());
  if (sGattReadQueue.empty()) {
    sGattReadClearToSend = true;
  }
}

static int sConnIdRegisterForNotification = 0;

static void DeregisterForNotifications(const int conn_id, BluetoothGattConnection& connection) {
  for (auto & service : connection.services) {
    for (auto & characteristic : service.characteristics) {
      if (characteristic.registered_for_notifications) {
        bt_bdaddr_t bda = {0};
        bt_bdaddr_t_from_string(connection.address, &bda);
        sConnIdRegisterForNotification = conn_id;
        bt_status_t bt_status =
            sBtGattInterface->client->deregister_for_notification(sBtGattClientIf,
                                                                  &bda,
                                                                  characteristic.char_handle);
        if (bt_status != BT_STATUS_SUCCESS) {
          logw("deregister_for_notification(%d, %s, %d) failed with status = %s",
               sBtGattClientIf, bt_bdaddr_t_to_string(&bda).c_str(),
               characteristic.char_handle);
        }
      }
    }
  }
}

static bt_status_t DisconnectOutboundConnectionById(const int conn_id)
{
  EraseGattQueueItemsByConnId(conn_id);
  if (!sBtGattInterface || !sBtGattClientIf) {
    return BT_STATUS_NOT_READY;
  }
  bt_bdaddr_t bda = {0};
  auto search = FindOutboundConnectionById(conn_id);
  if (search != sOutboundConnections.end()) {
    search->second.status = BluetoothGattConnectionStatus::Disconnecting;
    DeregisterForNotifications(conn_id, search->second.connection);
    bt_bdaddr_t_from_string(search->second.connection.address, &bda);
  }
  bt_status_t status = sBtGattInterface->client->disconnect(sBtGattClientIf, &bda, conn_id);
  if (search != sOutboundConnections.end()) {
    if (sCallbacks.outbound_connection_cb) {
      sCallbacks.outbound_connection_cb(search->second.connection.address,
                                        0,
                                        search->second.connection);
    }
    sOutboundConnections.erase(search->second.connection.address);
  }
  return status;
}

static std::string GetCharacteristicUUIDFromConnIDAndHandle(const int conn_id, const int handle)
{
  auto search = FindOutboundConnectionById(conn_id);
  if (search != sOutboundConnections.end()) {
    const BluetoothGattConnection& connection = search->second.connection;
    for (auto const& service : connection.services) {
      for (auto const& characteristic : service.characteristics) {
        if (characteristic.char_handle == handle) {
          return characteristic.uuid;
        }
      }
    }
  }
  return "";
}

static int GetCharacteristicHandleFromConnIDAndUUID(const int conn_id, const std::string& uuid)
{
  auto search = FindOutboundConnectionById(conn_id);
  if (search != sOutboundConnections.end()) {
    const BluetoothGattConnection& connection = search->second.connection;
    for (auto const& service : connection.services) {
      for (auto const& characteristic : service.characteristics) {
        if (bt_uuid_string_equals(uuid, characteristic.uuid)) {
          return characteristic.char_handle;
        }
      }
    }
  }

  return -1;
}

static std::string GetDescriptorUUIDFromConnIDAndHandle(const int conn_id, const int handle)
{
  auto search = FindOutboundConnectionById(conn_id);
  if (search != sOutboundConnections.end()) {
    const BluetoothGattConnection& connection = search->second.connection;
    for (auto const& service : connection.services) {
      for (auto const& characteristic : service.characteristics) {
        for (auto const& descriptor : characteristic.descriptors) {
          if (descriptor.desc_handle == handle) {
            return descriptor.uuid;
          }
        }
      }
    }
  }
  return "";
}

static std::string GetCharacteristicUUIDFromConnIDAndDescriptorHandle(const int conn_id, const int handle)
{
  auto search = FindOutboundConnectionById(conn_id);
  if (search != sOutboundConnections.end()) {
    const BluetoothGattConnection& connection = search->second.connection;
    for (auto const& service : connection.services) {
      for (auto const& characteristic : service.characteristics) {
        for (auto const& descriptor : characteristic.descriptors) {
          if (descriptor.desc_handle == handle) {
            return characteristic.uuid;
          }
        }
      }
    }
  }
  return "";
}

static int GetDescriptorHandleFromConnIDAndUUIDs(const int conn_id,
                                                 const std::string& char_uuid,
                                                 const std::string& desc_uuid)
{
  auto search = FindOutboundConnectionById(conn_id);
  if (search != sOutboundConnections.end()) {
    const BluetoothGattConnection& connection = search->second.connection;
    for (auto const& service : connection.services) {
      for (auto const& characteristic : service.characteristics) {
        if (bt_uuid_string_equals(char_uuid, characteristic.uuid)) {
          for (auto const& descriptor : characteristic.descriptors) {
            if (bt_uuid_string_equals(desc_uuid, descriptor.uuid)) {
              return descriptor.desc_handle;
            }
          }
        }
      }
    }
  }

  return -1;
}

static bool IsOutboundConnectionFullyEstablished(const int conn_id)
{
  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    return false;
  }

  BluetoothGattConnection& connection = search->second.connection;
  for (auto & service : connection.services) {
    for (auto & characteristic : service.characteristics) {
      if (characteristic.properties & kGattCharacteristicPropNotify) {
        if (!characteristic.registered_for_notifications) {
          return false;
        }
        for (auto & descriptor : characteristic.descriptors) {
          if (bt_uuid_string_equals(Anki::kCCCDescriptorUUID, descriptor.uuid)) {
            if (!descriptor.descriptor_written) {
              return false;
            }
          }
        }
      }
    }
  }
  search->second.status = BluetoothGattConnectionStatus::Connected;
  return true;
}

static void LogConnectionEstablishmentDuration(const BluetoothGattConnectionInfo& info)
{
  logi("connection to %s established in %d milliseconds",
       info.connection.address.c_str(),
       Anki::Util::CodeTimer::MillisecondsElapsed(info.start));
}

static void TransmitNextWriteItem() {
  if (!sGattWriteClearToSend || sCongested) {
    return;
  }
  bool transmitted = false;
  while (!sCongested && sGattWriteClearToSend && !sGattWriteQueue.empty() && !transmitted) {
    BluetoothGattWriteItem writeItem = sGattWriteQueue.front();
    sGattWriteQueue.pop_front();
    auto search = FindOutboundConnectionById(writeItem.conn_id);
    if (search != sOutboundConnections.end()) {
      sGattWriteClearToSend = false;
      sTaskExecutor.Wake([writeItem]() {
          bt_status_t bt_status;
          if (writeItem.is_descriptor) {
            logv("Calling write_descriptor(conn_id = %d, handle = %d)",
                 writeItem.conn_id, writeItem.handle);
            bt_status =
                sBtGattInterface->client->write_descriptor(writeItem.conn_id,
                                                           writeItem.handle,
                                                           writeItem.write_type,
                                                           writeItem.value.size(),
                                                           0 /* auth_req */,
                                                           (char *) writeItem.value.data());
          } else {
            logv("Calling write_characteristic(conn_id = %d, handle = %d)",
                 writeItem.conn_id, writeItem.handle);
            bt_status =
                sBtGattInterface->client->write_characteristic(writeItem.conn_id,
                                                               writeItem.handle,
                                                               writeItem.write_type,
                                                               writeItem.value.size(),
                                                               0 /* auth_req */,
                                                               (char *) writeItem.value.data());
          }
          if (bt_status != BT_STATUS_SUCCESS) {
            std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
            (void) DisconnectOutboundConnectionById(writeItem.conn_id);
          }
        });
      transmitted = true;
    } else {
      logw("%s - Didn't find conn_id = %d", __FUNCTION__, writeItem.conn_id);
      EraseGattQueueItemsByConnId(writeItem.conn_id);
    }
  }
}

static void QueueWriteItem(const bool is_descriptor,
                           const int conn_id,
                           const int handle,
                           const int write_type,
                           const std::vector<uint8_t>& value) {
  BluetoothGattWriteItem writeItem(is_descriptor,
                                   conn_id,
                                   handle,
                                   write_type,
                                   value);
  sGattWriteQueue.push_back(writeItem);
  if (sGattWriteQueue.size() == 1) {
    TransmitNextWriteItem();
  }
}

static void TransmitNextReadItem() {
  if (!sGattReadClearToSend || sCongested) {
    return;
  }
  bool transmitted = false;
  while (!sCongested && sGattReadClearToSend && !sGattReadQueue.empty() && !transmitted) {
    BluetoothGattReadItem readItem = sGattReadQueue.front();
    sGattReadQueue.pop_front();
    auto search = FindOutboundConnectionById(readItem.conn_id);
    if (search != sOutboundConnections.end()) {
      sGattReadClearToSend = false;
      sTaskExecutor.Wake([readItem]() {
          bt_status_t bt_status;
          if (readItem.is_descriptor) {
            logv("Calling read_descriptor(conn_id = %d, handle = %d)",
                 readItem.conn_id, readItem.handle);
            bt_status =
                sBtGattInterface->client->read_descriptor(readItem.conn_id,
                                                          readItem.handle,
                                                          0 /* auth_req */);
          } else {
            logv("Calling read_characteristic(conn_id = %d, handle = %d)",
                 readItem.conn_id, readItem.handle);
            bt_status =
                sBtGattInterface->client->read_characteristic(readItem.conn_id,
                                                              readItem.handle,
                                                              0 /* auth_req */);
          }
          if (bt_status != BT_STATUS_SUCCESS) {
            std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
            (void) DisconnectOutboundConnectionById(readItem.conn_id);
          }
        });
      transmitted = true;
    } else {
      logw("%s - Didn't find conn_id = %d", __FUNCTION__, readItem.conn_id);
      EraseGattQueueItemsByConnId(readItem.conn_id);
    }
  }
}

static void QueueReadItem(const bool is_descriptor,
                          const int conn_id,
                          const int handle) {

  BluetoothGattReadItem readItem(is_descriptor,
                                 conn_id,
                                 handle);

  sGattReadQueue.push_back(readItem);
  if (sGattReadQueue.size() == 1) {
    TransmitNextReadItem();
  }
}

static bool set_wake_alarm(uint64_t delay_millis, bool should_wake, alarm_cb cb,
                         void *data) {
  logv("%s", __FUNCTION__);
  return BT_STATUS_SUCCESS;
}

static int acquire_wake_lock(const char *lock_name) {
  logv("%s", __FUNCTION__);
  return BT_STATUS_SUCCESS;
}

static int release_wake_lock(const char *lock_name) {
  logv("%s", __FUNCTION__);
  return BT_STATUS_SUCCESS;
}

static bt_os_callouts_t sBtOsCallouts = {
  sizeof(bt_os_callouts_t),
  set_wake_alarm,
  acquire_wake_lock,
  release_wake_lock,
};

static void adapter_state_changed_cb(bt_state_t state) {
  logv("%s(state = %s)", __FUNCTION__, bt_state_t_to_string(state));
  std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
  if (state == BT_STATE_ON) {
    sBtAdapterEnabled = true;
  } else if (state == BT_STATE_OFF) {
    sBtAdapterEnabled = false;
  }
  sBtStackCallbackReceived = true;
  sBtStackCallbackCV.notify_one();
}

static std::string bt_property_t_to_string(const bt_property_t *property) {
  std::ostringstream oss;
  oss << bt_property_type_t_to_string(property->type) << " = ";
  switch(property->type) {
    case BT_PROPERTY_BDNAME:
      {
        bt_bdname_t* bdname = (bt_bdname_t *) (property->val);
        std::string name(bdname->name, bdname->name + property->len);
        oss << name;
      }
      break;
    case BT_PROPERTY_BDADDR:
      {
        oss << bt_bdaddr_t_to_string((bt_bdaddr_t *) property->val);
      }
      break;
    case BT_PROPERTY_UUIDS:
      {
        size_t bt_uuid_t_sz = sizeof(bt_uuid_t);
        int num_uuids = property->len / bt_uuid_t_sz;
        bt_uuid_t* ptr = (bt_uuid_t *) property->val;
        oss << "[ ";
        for (int i = 0 ; i < num_uuids ; i++) {
          if (!i) {
            oss << ", ";
          }
          oss << bt_uuid_t_to_string(&(ptr[i]));
        }
        oss << " ]";
      }
      break;
    case BT_PROPERTY_CLASS_OF_DEVICE:
      oss << *((uint32_t *) property->val);
      break;
    case BT_PROPERTY_TYPE_OF_DEVICE:
      oss << bt_device_type_t_to_string(*((bt_device_type_t *) property->val));
      break;
    case BT_PROPERTY_SERVICE_RECORD:
      {
        bt_service_record_t* record = (bt_service_record_t *) property->val;
        oss << "{ " << bt_uuid_t_to_string(&(record->uuid)) << " " << record->channel
            << " " << record->name << " }";
      }
      break;
    case BT_PROPERTY_ADAPTER_SCAN_MODE:
      {
        bt_scan_mode_t mode = *((bt_scan_mode_t *) property->val);
        oss << bt_scan_mode_t_to_string(mode);
      }
      break;
    case BT_PROPERTY_ADAPTER_BONDED_DEVICES:
      {
        size_t bt_bdaddr_t_sz = sizeof(bt_bdaddr_t);
        int num_devices = property->len / bt_bdaddr_t_sz;
        bt_bdaddr_t* addr = (bt_bdaddr_t *) property->val;
        oss << "[ ";
        for (int i = 0 ; i < num_devices; i++) {
          if (!i) {
            oss << ", ";
          }
          oss << bt_bdaddr_t_to_string(&(addr[i]));
        }
        oss << " ]";
      }
      break;
    case BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT:
      oss << *((uint32_t *) property->val);
      break;
    case BT_PROPERTY_REMOTE_FRIENDLY_NAME:
      {
        bt_bdname_t* bdname = (bt_bdname_t *) (property->val);
        std::string name(bdname->name, bdname->name + property->len);
        oss << name;
      }
      break;
    case BT_PROPERTY_REMOTE_RSSI:
      {
        oss << *((int32_t *) property->val);
      }
      break;
    case BT_PROPERTY_REMOTE_VERSION_INFO:
      {
        bt_remote_version_t* v = (bt_remote_version_t *) property->val;
        oss << v->version << ", " << v->sub_ver << ", " << v->manufacturer;
      }
      break;
    case BT_PROPERTY_LOCAL_LE_FEATURES:
      {
        bt_local_le_features_t* f = (bt_local_le_features_t *) property->val;
        oss << f->version_supported << ", ";
        oss << f->local_privacy_enabled << ", ";
        oss << f->max_adv_instance << ", ";
        oss << f->rpa_offload_supported << ", ";
        oss << f->max_irk_list_size << ", ";
        oss << f->max_adv_filter_supported << ", ";
        oss << f->activity_energy_info_supported << ", ";
        oss << f->scan_result_storage_size << ", ";
        oss << f->total_trackable_advertisers << ", ";
        oss << f->extended_scan_support << ", ";
        oss << f->debug_logging_supported;
      }
      break;
    default:
      break;
  }
  return oss.str();
}

static void adapter_properties_cb(bt_status_t status, int num_properties,
                                  bt_property_t *properties) {
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  logv("%s(status = %s, num_properties = %d)",
       __FUNCTION__, bt_status_t_to_string(status).c_str(), num_properties);
  for (int i = 0 ; i < num_properties; i++) {
    logv("%s(%s)", __FUNCTION__, bt_property_t_to_string(&(properties[i])).c_str());
    if (properties[i].type == BT_PROPERTY_BDNAME) {
      bt_bdname_t* bdname = (bt_bdname_t *) (properties[i].val);
      std::string name(bdname->name, bdname->name + properties[i].len);
      sBtAdapterName = name;
    }
  }
}

static void remote_device_properties_cb(bt_status_t status, bt_bdaddr_t *bd_addr,
                                        int num_properties, bt_property_t *properties) {
  logv("%s", __FUNCTION__);
}

static void device_found_cb(int num_properties, bt_property_t *properties) {
  logv("%s", __FUNCTION__);
}

static void bond_state_changed_cb(bt_status_t status, bt_bdaddr_t *bd_addr,
                                  bt_bond_state_t state) {
  logv("%s(status = %s, bd_addr = %s, state = %s)",
       __FUNCTION__, bt_status_t_to_string(status).c_str(),
       bt_bdaddr_t_to_string(bd_addr).c_str(),
       bt_bond_state_t_to_string(state).c_str());
}

static void acl_state_changed_cb(bt_status_t status, bt_bdaddr_t *bd_addr,
                                 bt_acl_state_t state) {
  logv("%s(status = %s, bd_addr = %s, state = %s)",
       __FUNCTION__, bt_status_t_to_string(status).c_str(),
       bt_bdaddr_t_to_string(bd_addr).c_str(),
       bt_acl_state_t_to_string(state).c_str());
}

static void discovery_state_changed_cb(bt_discovery_state_t state) {
  logv("%s", __FUNCTION__);
}

static void pin_request_cb(bt_bdaddr_t *bd_addr, bt_bdname_t *bd_name,
                           uint32_t cod, bool min_16_digit) {
  logv("%s", __FUNCTION__);
}

static void ssp_request_cb(bt_bdaddr_t *bd_addr, bt_bdname_t *bd_name, uint32_t cod,
                           bt_ssp_variant_t pairing_variant, uint32_t pass_key) {
  logv("%s", __FUNCTION__);
}

static void thread_evt_cb(bt_cb_thread_evt event) {
  logv("%s(event = %s)", __FUNCTION__, bt_cb_thread_evt_to_string(event));
}

static void dut_mode_recv_cb (uint16_t event_code, uint8_t *buf, uint8_t len) {
  logv("%s", __FUNCTION__);
}

static void le_test_mode_cb (bt_status_t status, uint16_t num_packets) {
  logv("%s", __FUNCTION__);
}

static void energy_info_cb(bt_activity_energy_info *p_energy_info,
                           bt_uid_traffic_t *uid_data) {
  logv("%s", __FUNCTION__);
}

static bt_callbacks_t sBluetoothCallbacks = {
  sizeof(sBluetoothCallbacks),
  adapter_state_changed_cb,
  adapter_properties_cb,
  remote_device_properties_cb,
  device_found_cb,
  discovery_state_changed_cb,
  pin_request_cb,
  ssp_request_cb,
  bond_state_changed_cb,
  acl_state_changed_cb,
  thread_evt_cb,
  dut_mode_recv_cb,
  le_test_mode_cb,
  energy_info_cb
};

static void SsrCleanupCb() {
  logv("%s", __FUNCTION__);
}

static btvendor_callbacks_t sVendorCallbacks = {
  sizeof(sVendorCallbacks),
  nullptr,
  SsrCleanupCb,
};

void btgattc_register_client_cb(int status, int client_if, bt_uuid_t *app_uuid)
{
  logv("%s(status = %s, client_if = %d, app_uuid = %s)",
       __FUNCTION__, bt_status_t_to_string((bt_status_t) status).c_str(), client_if,
       bt_uuid_t_to_string(app_uuid).c_str());
  std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
  sBtGattClientIf = client_if;
  sBtStackCallbackReceived = true;
  sBtStackCallbackCV.notify_one();
}

void btgattc_scan_result_cb(bt_bdaddr_t* bda, int rssi, uint8_t* adv_data)
{
  if (!bda || !adv_data) {
    return;
  }
  std::string address = bt_bdaddr_t_to_string(bda);
  std::vector<uint8_t> advertising_data;

  uint8_t* p = adv_data;
  uint8_t* end = p + 62; // maximum of 62 bytes for advertisement + scan response
  while (p < end) {
    uint8_t length = *p;
    if (length == 0 || (p + length) > end) {
      break;
    }
    advertising_data.push_back(length);
    p++;
    std::vector<uint8_t> data(p, p + length);
    std::copy(data.begin(), data.end(), std::back_inserter(advertising_data));
    p += length;
  }

  logv("%s(bda = %s, rssi = %d, adv_data = %s)",
       __FUNCTION__, address.c_str(),
       rssi, bt_value_to_string(advertising_data.size(), advertising_data.data()).c_str());
  if (sCallbacks.scan_result_cb) {
    sCallbacks.scan_result_cb(address, rssi, advertising_data);
  }
}

void btgattc_open_cb(int conn_id, int status, int clientIf, bt_bdaddr_t* bda)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  bt_status_t bt_status = (bt_status_t) status;
  std::string address = bt_bdaddr_t_to_string(bda);
  logv("%s(conn_id = %d, status = %s, clientIf = %d, bda = %s)",
       __FUNCTION__, conn_id, bt_status_t_to_string(bt_status).c_str(),
       clientIf, address.c_str());
  auto search = sOutboundConnections.find(address);
  if (search != sOutboundConnections.end()) {
    BluetoothGattConnection& connection = search->second.connection;
    connection.conn_id = conn_id;
    if (bt_status == BT_STATUS_SUCCESS) {
      if (search->second.status == BluetoothGattConnectionStatus::Connecting) {
        search->second.status = BluetoothGattConnectionStatus::DiscoveringServices;
        bt_status = sBtGattInterface->client->search_service(conn_id, nullptr);
      } else if (search->second.status == BluetoothGattConnectionStatus::Disconnecting) {
        logv("%s(..., bda = %s) is Disconnecting.  Will drop this connection",
             __FUNCTION__, address.c_str());
        sTaskExecutor.Wake(std::bind(&DisconnectGattPeerByAddress, address));
      } else {
        logw("%s(conn_id = %d, clientIf = %d, bda = %s) - Unexpected connection status = %s",
             __FUNCTION__, conn_id, clientIf, address.c_str(),
             BluetoothGattConnectionStatusToString(search->second.status).c_str());
      }
    }
    if (bt_status != BT_STATUS_SUCCESS) {
      if (sCallbacks.outbound_connection_cb) {
        sCallbacks.outbound_connection_cb(address, 0, connection);
      }
      sOutboundConnections.erase(address);
      return;
    }
  }
}

void btgattc_close_cb(int conn_id, int status, int clientIf, bt_bdaddr_t* bda)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  bt_status_t bt_status = (bt_status_t) status;
  std::string address = bt_bdaddr_t_to_string(bda);
  logv("%s(conn_id = %d, status = %s, clientIf = %d, bda = %s)",
       __FUNCTION__, conn_id, bt_status_t_to_string(bt_status).c_str(),
       clientIf, address.c_str());
  EraseGattQueueItemsByConnId(conn_id);
  auto search = sOutboundConnections.find(address);
  if (search != sOutboundConnections.end()) {
    search->second.connection.conn_id = conn_id;
    DeregisterForNotifications(conn_id, search->second.connection);
    if (sCallbacks.outbound_connection_cb) {
      sCallbacks.outbound_connection_cb(address, 0, search->second.connection);
    }
    sOutboundConnections.erase(address);
  }
}

void btgattc_search_complete_cb(int conn_id, int status)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(conn_id = %d, status = %s)",
       __FUNCTION__, conn_id, bt_status_t_to_string(bt_status).c_str());

  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    // If this is not a connection we are tracking, then disconnect
    (void) DisconnectOutboundConnectionById(conn_id);
    return;
  }
  BluetoothGattConnection& connection = search->second.connection;

  if (bt_status == BT_STATUS_SUCCESS) {
    search->second.status = BluetoothGattConnectionStatus::GettingGattDb;
    bt_status = sBtGattInterface->client->get_gatt_db(conn_id);
  }

  if (bt_status != BT_STATUS_SUCCESS) {
    (void) DisconnectOutboundConnectionById(conn_id);
  }
}


void btgattc_register_for_notification_cb(int conn_id, int registered,
                                          int status, uint16_t handle)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(conn_id = %d, registered = %d, status = %s, handle = %d)",
       __FUNCTION__, conn_id, registered, bt_status_t_to_string(bt_status).c_str(), handle);

  if (!conn_id) {
    conn_id = sConnIdRegisterForNotification;
  }
  auto search = FindOutboundConnectionById(conn_id);
  if ((search == sOutboundConnections.end()) || (bt_status != BT_STATUS_SUCCESS)) {
    (void) DisconnectOutboundConnectionById(conn_id);
    return;
  }

  BluetoothGattConnection& connection = search->second.connection;
  for (auto& service : connection.services) {
    if (service.start_handle <= handle && handle <= service.end_handle) {
      for (auto& characteristic : service.characteristics) {
        if (characteristic.char_handle == handle) {
          characteristic.registered_for_notifications = (bool) registered;
        }
      }
    }
  }

  if (IsOutboundConnectionFullyEstablished(conn_id)) {
    LogConnectionEstablishmentDuration(search->second);
    if (sCallbacks.outbound_connection_cb) {
      sCallbacks.outbound_connection_cb(connection.address, 1, connection);
    }
  }
}

void btgattc_notify_cb(int conn_id, btgatt_notify_params_t *p_data)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  if (!p_data) {
    loge("%s(conn_id = %d). p_data is null", __FUNCTION__, conn_id);
    return;
  }
  std::string address = bt_bdaddr_t_to_string(&(p_data->bda));
  logv("%s(conn_id = %d, p_data = {value = %s, bda = %s, handle = %d, len = %d, is_notify = %d})",
       __FUNCTION__, conn_id, bt_value_to_string(p_data->len, p_data->value).c_str(), address.c_str(),
       p_data->handle, p_data->len, p_data->is_notify);
  if (!sCallbacks.notification_received_cb) {
    return;
  }

  auto search = sOutboundConnections.find(address);
  if (search == sOutboundConnections.end()) {
    loge("%s(conn_id = %d). %s not found in outbound connections",
         __FUNCTION__, conn_id, address.c_str());
    sTaskExecutor.Wake(std::bind(&DisconnectGattPeerByAddress, address));
    return;
  }
  std::string char_uuid = GetCharacteristicUUIDFromConnIDAndHandle(conn_id, p_data->handle);
  if (char_uuid.empty()) {
    loge("%s(conn_id = %d). could not find uuid for handle (%d)",
         __FUNCTION__, conn_id, p_data->handle);
    return;
  }
  std::vector<uint8_t> value(p_data->value, p_data->value + p_data->len);
  sCallbacks.notification_received_cb(address, conn_id, char_uuid, value);
}

void btgattc_read_characteristic_cb(int conn_id, int status,
                                    btgatt_read_params_t *p_data)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  if (!p_data) {
    loge("%s(conn_id = %d). p_data is null", __FUNCTION__, conn_id);
    sGattReadClearToSend = true;
    TransmitNextReadItem();
    return;
  }
  logv("%s(conn_id = %d, p_data = {value = %s, handle = %d, len = %d, value_type = %d, status = %d})",
       __FUNCTION__, conn_id, bt_value_to_string(p_data->value.len, p_data->value.value).c_str(),
       p_data->handle, p_data->value.len, p_data->value_type, p_data->status);
  if (!sCallbacks.characteristic_read_cb) {
    sGattReadClearToSend = true;
    TransmitNextReadItem();
    return;
  }

  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    loge("%s(conn_id = %d). not found in outbound connections",
         __FUNCTION__, conn_id);
    sGattReadClearToSend = true;
    TransmitNextReadItem();
    return;
  }
  std::string char_uuid = GetCharacteristicUUIDFromConnIDAndHandle(conn_id, p_data->handle);
  if (char_uuid.empty()) {
    loge("%s(conn_id = %d). could not find uuid for handle (%d)",
         __FUNCTION__, conn_id, p_data->handle);
    sGattReadClearToSend = true;
    TransmitNextReadItem();
    return;
  }
  std::vector<uint8_t> value(p_data->value.value, p_data->value.value + p_data->value.len);
  sCallbacks.characteristic_read_cb(search->second.connection.address,
                                    conn_id,
                                    p_data->status,
                                    char_uuid,
                                    value);
  sGattReadClearToSend = true;
  TransmitNextReadItem();
}

void btgattc_write_characteristic_cb(int conn_id, int status,
                                     uint16_t handle)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(conn_id = %d, status = %s, handle = %d)",
       __FUNCTION__, conn_id, bt_status_t_to_string(bt_status).c_str(), handle);
  sGattWriteClearToSend = true;
  TransmitNextWriteItem();
}

void btgattc_execute_write_cb(int conn_id, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_read_descriptor_cb(int conn_id, int status, btgatt_read_params_t *p_data)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  if (!p_data) {
    loge("%s(conn_id = %d). p_data is null", __FUNCTION__, conn_id);
    sGattReadClearToSend = true;
    TransmitNextReadItem();
    return;
  }
  logv("%s(conn_id = %d, p_data = {value = %s, handle = %d, len = %d, value_type = %d, status = %d})",
       __FUNCTION__, conn_id, bt_value_to_string(p_data->value.len, p_data->value.value).c_str(),
       p_data->handle, p_data->value.len, p_data->value_type, p_data->status);
  if (!sCallbacks.descriptor_read_cb) {
    sGattReadClearToSend = true;
    TransmitNextReadItem();
    return;
  }

  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    loge("%s(conn_id = %d). not found in outbound connections",
         __FUNCTION__, conn_id);
    sGattReadClearToSend = true;
    TransmitNextReadItem();
    return;
  }
  std::string desc_uuid = GetDescriptorUUIDFromConnIDAndHandle(conn_id, p_data->handle);
  if (desc_uuid.empty()) {
    loge("%s(conn_id = %D). could not find descriptor uuid for handle (%d)",
         __FUNCTION__, conn_id, p_data->handle);
    sGattReadClearToSend = true;
    TransmitNextReadItem();
    return;
  }
  std::string char_uuid = GetCharacteristicUUIDFromConnIDAndDescriptorHandle(conn_id, p_data->handle);
  std::vector<uint8_t> value(p_data->value.value, p_data->value.value + p_data->value.len);
  sCallbacks.descriptor_read_cb(search->second.connection.address,
                                conn_id,
                                p_data->status,
                                char_uuid,
                                desc_uuid,
                                value);
  sGattReadClearToSend = true;
  TransmitNextReadItem();
}

void btgattc_write_descriptor_cb(int conn_id, int status, uint16_t handle)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(conn_id = %d, status = %s, handle = %d)",
       __FUNCTION__, conn_id, bt_status_t_to_string(bt_status).c_str(), handle);

  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    loge("%s(conn_id = %d). connection not found", __FUNCTION__, conn_id);
    sGattWriteClearToSend = true;
    TransmitNextWriteItem();
    return;
  }

  BluetoothGattConnection& connection = search->second.connection;

  BluetoothGattDescriptor* descriptor =
      FindBluetoothGattDescriptorByConnectionAndHandle(connection, handle);

  if (!descriptor || (bt_status != BT_STATUS_SUCCESS)) {
    DisconnectOutboundConnectionById(conn_id);
    sGattWriteClearToSend = true;
    TransmitNextWriteItem();
    return;
  }

  descriptor->descriptor_written = true;

  if (IsOutboundConnectionFullyEstablished(conn_id)) {
    LogConnectionEstablishmentDuration(search->second);
    if (sCallbacks.outbound_connection_cb) {
      sCallbacks.outbound_connection_cb(connection.address, 1, connection);
    }
  }
  sGattWriteClearToSend = true;
  TransmitNextWriteItem();
}

void btgattc_remote_rssi_cb(int client_if,bt_bdaddr_t* bda, int rssi, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_advertise_cb(int status, int client_if)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(status = %s, client_if = %d)",
       __FUNCTION__, bt_status_t_to_string(bt_status).c_str(), client_if);
}

void btgattc_configure_mtu_cb(int conn_id, int status, int mtu)
{
  logv("%s", __FUNCTION__);
}

void btgattc_scan_filter_cfg_cb(int action, int client_if, int status, int filt_type, int avbl_space)
{
  logv("%s", __FUNCTION__);
}

void btgattc_scan_filter_param_cb(int action, int client_if, int status, int avbl_space)
{
  logv("%s", __FUNCTION__);
}

void btgattc_scan_filter_status_cb(int action, int client_if, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_multiadv_enable_cb(int client_if, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_multiadv_update_cb(int client_if, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_multiadv_setadv_data_cb(int client_if, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_multiadv_disable_cb(int client_if, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_congestion_cb(int conn_id, bool congested)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  logv("%s(conn_id = %d, congested = %s)",
       __FUNCTION__, conn_id, congested ? "true" : "false");
  bool previous_congested = sCongested;
  sCongested = congested;
  if (previous_congested && !sCongested) {
    TransmitNextWriteItem();
    TransmitNextReadItem();
  }
}

void btgattc_batchscan_cfg_storage_cb(int client_if, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_batchscan_startstop_cb(int startstop_action, int client_if, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_batchscan_reports_cb(int client_if, int status, int report_format,
                                  int num_records, int data_len, uint8_t *p_rep_data)
{
  logv("%s", __FUNCTION__);
}

void btgattc_batchscan_threshold_cb(int client_if)
{
  logv("%s", __FUNCTION__);
}

void btgattc_track_adv_event_cb(btgatt_track_adv_info_t *p_adv_track_info)
{
  logv("%s", __FUNCTION__);
}

void btgattc_scan_parameter_setup_completed_cb(int client_if, btgattc_error_t status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_get_gatt_db_cb(int conn_id, btgatt_db_element_t *db, int count)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  logv("%s(conn_id = %d, db = %p, count = %d)",
       __FUNCTION__, conn_id, db, count);
  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    DisconnectOutboundConnectionById(conn_id);
    return;
  }
  BluetoothGattConnection& connection = search->second.connection;
  std::vector<BluetoothGattService> services;
  for (int i = 0 ; i < count; i++) {
    std::string uuidStr = bt_uuid_t_to_string(&(db[i].uuid));
    if ((db[i].type == BTGATT_DB_PRIMARY_SERVICE)
        || (db[i].type == BTGATT_DB_SECONDARY_SERVICE)
        || (db[i].type == BTGATT_DB_INCLUDED_SERVICE)) {
      logv("%s(conn_id = %d). Handle = %02d. Service with uuid = '%s'",
           __FUNCTION__, conn_id, db[i].attribute_handle, uuidStr.c_str());
      BluetoothGattService service(uuidStr,
                                   db[i].attribute_handle,
                                   db[i].start_handle,
                                   db[i].end_handle);
      services.push_back(service);
    } else if (db[i].type == BTGATT_DB_CHARACTERISTIC) {
      BluetoothGattService& lastService = services.back();
      BluetoothGattCharacteristic characteristic(uuidStr,
                                                 db[i].properties,
                                                 0,
                                                 db[i].attribute_handle);
      lastService.characteristics.push_back(characteristic);
      logv("%s(conn_id = %d). Handle = %02d. Characteristic that is %s notifiable with uuid = '%s'",
           __FUNCTION__, conn_id, db[i].attribute_handle,
           (db[i].properties & kGattCharacteristicPropNotify) ? "" : "not",
           uuidStr.c_str());

      if (db[i].properties & kGattCharacteristicPropNotify) {
        bt_bdaddr_t bda = {0};
        bt_bdaddr_t_from_string(connection.address, &bda);
        sConnIdRegisterForNotification = conn_id;
        bt_status_t bt_status =
            sBtGattInterface->client->register_for_notification(sBtGattClientIf,
                                                                &bda,
                                                                db[i].attribute_handle);

        if (bt_status != BT_STATUS_SUCCESS) {
          DisconnectOutboundConnectionById(conn_id);
          return;
        }
        search->second.status = BluetoothGattConnectionStatus::RegisteringForNotifications;
      }
    } else if (db[i].type == BTGATT_DB_DESCRIPTOR) {
      BluetoothGattCharacteristic& lastCharacteristic = services.back().characteristics.back();
      BluetoothGattDescriptor descriptor(uuidStr, 0, db[i].attribute_handle);
      lastCharacteristic.descriptors.push_back(descriptor);
      logv("%s(conn_id = %d). Handle = %02d. Descriptor characteristic (%d) with uuid = '%s'",
           __FUNCTION__, conn_id, db[i].attribute_handle,
           lastCharacteristic.char_handle, uuidStr.c_str());
      if (lastCharacteristic.properties & kGattCharacteristicPropNotify) {
        if (bt_uuid_string_equals(Anki::kCCCDescriptorUUID, descriptor.uuid)) {
          std::vector<uint8_t> value({0x01, 0x00});
          QueueWriteItem(true,
                         conn_id,
                         descriptor.desc_handle,
                         kGattWriteTypeWithResponse,
                         value);
        }
      }
    }
  }
  connection.services = services;
  return;
}

static const btgatt_client_callbacks_t sGattClientCallbacks = {
  btgattc_register_client_cb,
  btgattc_scan_result_cb,
  btgattc_open_cb,
  btgattc_close_cb,
  btgattc_search_complete_cb,
  btgattc_register_for_notification_cb,
  btgattc_notify_cb,
  btgattc_read_characteristic_cb,
  btgattc_write_characteristic_cb,
  btgattc_read_descriptor_cb,
  btgattc_write_descriptor_cb,
  btgattc_execute_write_cb,
  btgattc_remote_rssi_cb,
  btgattc_advertise_cb,
  btgattc_configure_mtu_cb,
  btgattc_scan_filter_cfg_cb,
  btgattc_scan_filter_param_cb,
  btgattc_scan_filter_status_cb,
  btgattc_multiadv_enable_cb,
  btgattc_multiadv_update_cb,
  btgattc_multiadv_setadv_data_cb,
  btgattc_multiadv_disable_cb,
  btgattc_congestion_cb,
  btgattc_batchscan_cfg_storage_cb,
  btgattc_batchscan_startstop_cb,
  btgattc_batchscan_reports_cb,
  btgattc_batchscan_threshold_cb,
  btgattc_track_adv_event_cb,
  btgattc_scan_parameter_setup_completed_cb,
  btgattc_get_gatt_db_cb,
  nullptr,
  nullptr
};

void btgatts_register_server_cb(int status, int server_if, bt_uuid_t *app_uuid)
{
  logv("%s(status = %s, server_if = %d, app_uuid = %s)",
       __FUNCTION__, bt_status_t_to_string((bt_status_t) status).c_str(), server_if,
       bt_uuid_t_to_string(app_uuid).c_str());
  std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
  sBtGattServerIf = server_if;
  sBtStackCallbackReceived = true;
  sBtStackCallbackCV.notify_one();
}

void btgatts_connection_cb(int conn_id, int server_if, int connected, bt_bdaddr_t *bda)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  std::string address = bt_bdaddr_t_to_string(bda);
  logv("%s(conn_id = %d, server_if = %d, connected = %d, bda = %s)",
       __FUNCTION__, conn_id, server_if, connected, address.c_str());

  if (sCallbacks.inbound_connection_cb) {
    // Make sure this isn't an outbound connection
    auto search = sOutboundConnections.find(address);
    if (search == sOutboundConnections.end()) {
      sCallbacks.inbound_connection_cb(conn_id, connected);
    }
  }
}

bt_status_t AddNextCharacteristicOrDescriptor() {
  bt_status_t result = BT_STATUS_SUCCESS;
  bool found = false;
  for (auto const& it : sBluetoothGattService->characteristics) {
    if (it.char_handle < 0) {
      found = true;
      bt_uuid_t uuid;
      bt_uuid_t_from_string(it.uuid, &uuid);
      result = sBtGattInterface->server->add_characteristic(sBtGattServerIf,
                                                            sBluetoothGattService->service_handle,
                                                            &uuid,
                                                            it.properties,
                                                            it.permissions);
      if (result != BT_STATUS_SUCCESS) {
        loge("add_characteristic returned %s", bt_status_t_to_string(result).c_str());
      }
      break;
    }
    for (auto const& dit : it.descriptors) {
      if (dit.desc_handle < 0) {
        found = true;
        bt_uuid_t uuid;
        bt_uuid_t_from_string(dit.uuid, &uuid);
        result = sBtGattInterface->server->add_descriptor(sBtGattServerIf,
                                                          sBluetoothGattService->service_handle,
                                                          &uuid,
                                                          dit.permissions);
        if (result != BT_STATUS_SUCCESS) {
          loge("add_descriptor returned %s", bt_status_t_to_string(result).c_str());
        }
        break;
      }
    }
    if (found) {
      break;
    }
  }
  if (!found) {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    sBtGattServiceAdded = true;
    sBtStackCallbackReceived = true;
    sBtStackCallbackCV.notify_one();
  }
  return result;
}

void btgatts_service_added_cb(int status, int server_if,
                              btgatt_srvc_id_t *srvc_id, int srvc_handle)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(status = %s, server_if = %d, srvc_id->id.uuid = %s, srvc_handle = %d)",
       __FUNCTION__, bt_status_t_to_string(bt_status).c_str(), server_if,
       bt_uuid_t_to_string(&(srvc_id->id.uuid)).c_str(), srvc_handle);
  sBluetoothGattService->service_handle = srvc_handle;

  if (bt_status == BT_STATUS_SUCCESS) {
    bt_status = AddNextCharacteristicOrDescriptor();
  }

  if (bt_status != BT_STATUS_SUCCESS) {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    sBtGattOpError = true;
    sBtStackCallbackReceived = true;
    sBtStackCallbackCV.notify_one();
  }
}

void btgatts_included_service_added_cb(int status, int server_if, int srvc_handle,
                                       int incl_srvc_handle)
{
  logv("%s", __FUNCTION__);
}

void btgatts_characteristic_added_cb(int status, int server_if, bt_uuid_t *char_id,
                                     int srvc_handle, int char_handle)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(status = %s, server_if = %d, char_id = %s, srvc_handle = %d, char_handle = %d)",
       __FUNCTION__, bt_status_t_to_string(bt_status).c_str(), server_if,
       bt_uuid_t_to_string(char_id).c_str(), srvc_handle, char_handle);
  if (bt_status == BT_STATUS_SUCCESS) {
    for (auto & it : sBluetoothGattService->characteristics) {
      if (it.char_handle < 0) {
        bt_uuid_t uuid;
        bt_uuid_t_from_string(it.uuid, &uuid);
        if (bt_uuid_t_equals(&uuid, char_id)) {
          it.char_handle = char_handle;
          break;
        }
      }
    }
    bt_status = AddNextCharacteristicOrDescriptor();
  }

  if (bt_status != BT_STATUS_SUCCESS) {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    sBtGattOpError = true;
    sBtStackCallbackReceived = true;
    sBtStackCallbackCV.notify_one();
  }

}

void btgatts_descriptor_added_cb(int status, int server_if, bt_uuid_t *descr_id,
                                 int srvc_handle, int descr_handle)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(status = %s, server_if = %d, descr_id = %s, srvc_handle = %d, descr_handle = %d)",
       __FUNCTION__, bt_status_t_to_string(bt_status).c_str(), server_if,
       bt_uuid_t_to_string(descr_id).c_str(), srvc_handle, descr_handle);
  if (bt_status == BT_STATUS_SUCCESS) {
    bool found = false;
    for (auto & it : sBluetoothGattService->characteristics) {
      for (auto & dit : it.descriptors) {
        if (dit.desc_handle < 0) {
          bt_uuid_t uuid;
          bt_uuid_t_from_string(dit.uuid, &uuid);
          if (bt_uuid_t_equals(&uuid, descr_id)) {
            dit.desc_handle = descr_handle;
            found = true;
            break;
          }
        }
      }
      if (found) {
        break;
      }
    }
    bt_status = AddNextCharacteristicOrDescriptor();
  }
  if (bt_status != BT_STATUS_SUCCESS) {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    sBtGattOpError = true;
    sBtStackCallbackReceived = true;
    sBtStackCallbackCV.notify_one();
  }

}

void btgatts_service_started_cb(int status, int server_if, int srvc_handle)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(status = %s, server_if = %d, srvc_handle = %d)",
       __FUNCTION__, bt_status_t_to_string(bt_status).c_str(), server_if, srvc_handle);
  std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
  sBtGattOpError = !(bt_status == BT_STATUS_SUCCESS);
  sBtGattServiceStarted =
      ((bt_status == BT_STATUS_SUCCESS) && (srvc_handle == sBluetoothGattService->service_handle));
  sBtStackCallbackReceived = true;
  sBtStackCallbackCV.notify_one();
}

void btgatts_service_stopped_cb(int status, int server_if, int srvc_handle)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(status = %s, server_if = %d, srvc_handle = %d)",
       __FUNCTION__, bt_status_t_to_string(bt_status).c_str(), server_if, srvc_handle);
  std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
  sBtGattOpError = !(bt_status == BT_STATUS_SUCCESS);
  sBtGattServiceStarted =
      !((bt_status == BT_STATUS_SUCCESS) && (srvc_handle == sBluetoothGattService->service_handle));
  sBtStackCallbackReceived = true;
  sBtStackCallbackCV.notify_one();
}

void btgatts_service_deleted_cb(int status, int server_if, int srvc_handle)
{
  logv("%s", __FUNCTION__);
}

void btgatts_request_read_cb(int conn_id, int trans_id, bt_bdaddr_t *bda, int attr_handle,
                             int offset, bool is_long)
{
  logv("%s(conn_id = %d, trans_id = %d, bda = %s, attr_handle = %d, offset = %d, is_long = %s)",
       __FUNCTION__, conn_id, trans_id, bt_bdaddr_t_to_string(bda).c_str(),
       attr_handle, offset, is_long ? "true" : "false");
  if (sCallbacks.request_read_cb) {
    sCallbacks.request_read_cb(conn_id, trans_id, attr_handle, offset);
  }
}

void btgatts_request_write_cb(int conn_id, int trans_id, bt_bdaddr_t *bda, int attr_handle,
                              int offset, int length, bool need_rsp, bool is_prep,
                              uint8_t* value)
{
  logv("%s(conn_id = %d, trans_id = %d, bda = %s, attr_handle = %d, offset = %d, length = %d, need_rsp = %s, is_prep = %s, value = %s)",
       __FUNCTION__, conn_id, trans_id, bt_bdaddr_t_to_string(bda).c_str(), attr_handle,
       offset, length, need_rsp ? "true" : "false", is_prep ? "true" : "false",
       bt_value_to_string(length, value).c_str());
  if (is_prep) {
    /* We do not support prepared writes */
    std::vector<uint8_t> dummy;
    (void) SendResponse(conn_id, trans_id, attr_handle,
                        kGattErrorRequestNotSupported, offset, dummy);
    return;
  }
  if (sCallbacks.request_write_cb) {
    std::vector<uint8_t> val(value, value + length);
    sCallbacks.request_write_cb(conn_id,
                                trans_id,
                                attr_handle,
                                offset,
                                need_rsp,
                                val);
  }
}

void btgatts_request_exec_write_cb(int conn_id, int trans_id,
                                   bt_bdaddr_t *bda, int exec_write)
{
  logv("%s", __FUNCTION__);
}

void btgatts_response_confirmation_cb(int status, int handle)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(status = %s, handle = %d)",
       __FUNCTION__, bt_status_t_to_string(bt_status).c_str(), handle);
}

void btgatts_indication_sent_cb(int conn_id, int status)
{
  logv("%s(conn_id = %d, status = %s)",
       __FUNCTION__, conn_id, bt_gatt_error_to_string(status).c_str());
  if (sCallbacks.indication_sent_cb) {
    sCallbacks.indication_sent_cb(conn_id, status);
  }
}

void btgatts_congestion_cb(int conn_id, bool congested)
{
  logv("%s(conn_id = %d, congested = %s)",
       __FUNCTION__, conn_id, congested ? "true" : "false");
  if (sCallbacks.congestion_cb) {
    sCallbacks.congestion_cb(conn_id, congested);
  }
}

void btgatts_mtu_changed_cb(int conn_id, int mtu)
{
  logv("%s(conn_id = %d, mtu = %d)", __FUNCTION__, conn_id, mtu);
}

static const btgatt_server_callbacks_t sGattServerCallbacks = {
  btgatts_register_server_cb,
  btgatts_connection_cb,
  btgatts_service_added_cb,
  btgatts_included_service_added_cb,
  btgatts_characteristic_added_cb,
  btgatts_descriptor_added_cb,
  btgatts_service_started_cb,
  btgatts_service_stopped_cb,
  btgatts_service_deleted_cb,
  btgatts_request_read_cb,
  btgatts_request_write_cb,
  btgatts_request_exec_write_cb,
  btgatts_response_confirmation_cb,
  btgatts_indication_sent_cb,
  btgatts_congestion_cb,
  btgatts_mtu_changed_cb
};

static btgatt_callbacks_t sGattCallbacks = {
  sizeof(btgatt_callbacks_t),
  &sGattClientCallbacks,
  &sGattServerCallbacks
};

#ifdef __cplusplus
}
#endif

void SetCallbacks(Callbacks* cbs) {
  if (cbs) {
    memcpy(&sCallbacks, cbs, sizeof(sCallbacks));
  } else {
    memset(&sCallbacks, 0, sizeof(sCallbacks));
  }
}

bool LoadBtStack() {
  hw_module_t *module;

  logv("Loading Bluetooth HAL Module");

  if (hw_get_module(BT_STACK_MODULE_ID, (hw_module_t const **) &module)) {
    loge("hw_get_module failed");
    return false;
  }

  if (module->methods->open(module, BT_STACK_MODULE_ID, &sDevice)) {
    loge("failed to open device");
    return false;
  }

  sBtDevice = (bluetooth_device_t *) sDevice;
  sBtInterface = sBtDevice->get_bluetooth_interface();
  if (!sBtInterface) {
    sBtDevice->common.close((hw_device_t *) & sBtDevice->common);
    sBtDevice = nullptr;
    return false;
  }

  if ((sBtInterface->init(&sBluetoothCallbacks) == BT_STATUS_SUCCESS)) {
    sBtInterface->set_os_callouts(&sBtOsCallouts);
  }

  sBtVendorInterface =
      (btvendor_interface_t *) sBtInterface->get_profile_interface(BT_PROFILE_VENDOR_ID);

  if (sBtVendorInterface) {
    sBtVendorInterface->init(&sVendorCallbacks);
  }

  sBtGattInterface = (btgatt_interface_t*) sBtInterface->get_profile_interface(BT_PROFILE_GATT_ID);
  if (!sBtGattInterface) {
    loge("failed to get GATT profile from bluetooth iface");
    return false;
  }

  bt_status_t status = sBtGattInterface->init(&sGattCallbacks);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to init GATT profile with callbacks");
    return false;
  }

  return true;
}

void UnLoadBtStack() {

  logv("Unloading Bluetooth HAL module");

  if (sBtGattInterface) {
    if (sBtGattServerIf > 0) {
      (void) sBtGattInterface->server->unregister_server(sBtGattServerIf);
      sBtGattServerIf = 0;
    }
    if (sBtGattClientIf > 0) {
      (void) sBtGattInterface->client->unregister_client(sBtGattClientIf);
      sBtGattClientIf = 0;
    }
    sBtGattInterface->cleanup();
    sBtGattInterface = nullptr;
  }

  if (sBtVendorInterface) {
    sBtVendorInterface->cleanup();
    sBtVendorInterface = nullptr;
  }

  if (sBtInterface) {
    sBtInterface->cleanup();
    sBtInterface = nullptr;
  }

  if (sBtDevice) {
    sBtDevice->common.close((hw_device_t *) &sBtDevice->common);
    sBtDevice = nullptr;
  }
}

bool EnableAdapter() {
  if (!sBtInterface) {
    return false;
  }
  if (sBtAdapterEnabled) {
    return true;
  }
  {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    sBtStackCallbackReceived = false;
  }
  int status = sBtInterface->enable(false /* guest_mode */);
  if (status != BT_STATUS_SUCCESS) {
    logw("sBtInterface->enable returned %s", bt_status_t_to_string((bt_status_t) status).c_str());
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lk(sBtStackCallbackMutex);
  if (sBtStackCallbackCV.wait_until(lk, now + 12000ms,
                                    [](){return (sBtStackCallbackReceived || sBtAdapterEnabled);})) {
    return true;
  } else {
    loge("adapter not enabled after 12000 milliseconds");
    return false;
  }
}

std::string GetAdapterName() {
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  return sBtAdapterName;
}

bool SetAdapterName(const std::string& name) {
  logv("Anki::BluetoothStack::SetAdapterName('%s')", name.c_str());
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  if (!sBtInterface || !sBtAdapterEnabled) {
    return false;
  }
  if (name == sBtAdapterName) {
    return true;
  }
  bt_bdname_t bd_name = {0};
  if (strlcpy((char *) &bd_name.name[0], name.c_str(), sizeof(bd_name)) >= sizeof(bd_name.name)) {
    loge("Failed to set adapter name to %s.  Too long (%zu bytes).  Max is %zu bytes.",
         name.c_str(), name.size(), sizeof(bd_name.name) - 1);
    return false;
  }
  bt_property_t property = {BT_PROPERTY_BDNAME, (int) strlen((char *) bd_name.name), &bd_name};
  int status = sBtInterface->set_adapter_property(&property);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to set adapter name to %s. status = %s",
         name.c_str(), bt_status_t_to_string((bt_status_t) status).c_str());
    return false;
  }
  return true;
}

bool RegisterGattClient() {
  if (!sBtGattInterface) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    sBtStackCallbackReceived = false;
  }
  bt_status_t status = sBtGattInterface->client->register_client(&sGattClientUUID);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to register GATT client. status = %s",
         bt_status_t_to_string(status).c_str());
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lk(sBtStackCallbackMutex);
  if (sBtStackCallbackCV.wait_until(lk, now + 1000ms,
                                    [](){return (sBtStackCallbackReceived && sBtGattClientIf > 0);})) {
    return true;
  } else {
    loge("gatt client not registered after 1000 milliseconds");
    return false;
  }
}

bool RegisterGattServer() {
  if (!sBtGattInterface) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    sBtStackCallbackReceived = false;
  }
  bt_status_t status = sBtGattInterface->server->register_server(&sGattServerUUID);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to register GATT server. status = %s",
         bt_status_t_to_string(status).c_str());
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lk(sBtStackCallbackMutex);
  if (sBtStackCallbackCV.wait_until(lk, now + 1000ms,
                                    [](){return (sBtStackCallbackReceived && sBtGattServerIf > 0);})) {
    return true;
  } else {
    loge("gatt server not registered after 1000 milliseconds");
    return false;
  }
}

bool AddGattService(BluetoothGattService* service) {
  sBluetoothGattService = service;

  btgatt_srvc_id_t srvc_id = {0};
  bt_uuid_t_from_string(service->uuid, &(srvc_id.id.uuid));
  srvc_id.id.inst_id = 0;
  srvc_id.is_primary = 1;

  // 1 handle for the service
  // 2 handles for each characteristic (don't know why it isn't just 1)
  // 1 handle for each descriptor
  int num_handles = 1 + (2 * service->characteristics.size());
  for (auto const& it : service->characteristics) {
    num_handles += it.descriptors.size();
  }

  {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    sBtStackCallbackReceived = false;
    sBtGattServiceAdded = false;
    sBtGattOpError = false;
  }
  bt_status_t status = sBtGattInterface->server->add_service(sBtGattServerIf,
                                                             &srvc_id,
                                                             num_handles);

  if (status != BT_STATUS_SUCCESS) {
    loge("failed to add GATT service. status = %s",
         bt_status_t_to_string(status).c_str());
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lk(sBtStackCallbackMutex);
  if (sBtStackCallbackCV.wait_until(lk, now + 1000ms,
                                    [](){return (sBtStackCallbackReceived &&
                                                 (sBtGattServiceAdded || sBtGattOpError));})) {
    if (sBtGattServiceAdded) {
      return true;
    } else if (sBtGattOpError) {
      loge("Failed to add GATT service");
      return false;
    }
  } else {
    loge("gatt service not added after 1000 milliseconds");
    return false;
  }

  return true;
}

bool StartGattService(BluetoothGattService* service) {
  if (service != sBluetoothGattService) {
    loge("service doesn't match");
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    if (sBtGattServiceStarted) {
      logv("service is already started");
      return true;
    }
    sBtStackCallbackReceived = false;
    sBtGattOpError = false;
  }
  bt_status_t status = sBtGattInterface->server->start_service(sBtGattServerIf,
                                                               sBluetoothGattService->service_handle,
                                                               GATT_TRANSPORT_LE);
  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to start service");
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lk(sBtStackCallbackMutex);
  if (sBtStackCallbackCV.wait_until(lk, now + 1000ms,
                                    [](){return (sBtStackCallbackReceived &&
                                                 (sBtGattServiceStarted || sBtGattOpError));})) {
    if (sBtGattServiceStarted) {
      return true;
    } else if (sBtGattOpError) {
      loge("Failed to start GATT service");
      return false;
    }
  } else {
    loge("gatt service not started after 1000 milliseconds");
    return false;
  }

}

bool StopGattService(BluetoothGattService* service) {
  if (service != sBluetoothGattService) {
    loge("service doesn't match");
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(sBtStackCallbackMutex);
    if (!sBtGattServiceStarted) {
      logv("service is not started. exiting");
      return true;
    }
    sBtStackCallbackReceived = false;
    sBtGattOpError = false;
  }
  bt_status_t status = sBtGattInterface->server->stop_service(sBtGattServerIf,
                                                              sBluetoothGattService->service_handle);
  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to stop service");
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lk(sBtStackCallbackMutex);
  if (sBtStackCallbackCV.wait_until(lk, now + 1000ms,
                                    [](){return (sBtStackCallbackReceived &&
                                                 (!sBtGattServiceStarted || sBtGattOpError));})) {
    if (!sBtGattServiceStarted) {
      return true;
    } else if (sBtGattOpError) {
      loge("Failed to stop GATT service");
      return false;
    }
  } else {
    loge("gatt service not stopped after 1000 milliseconds");
    return false;
  }

}

void ScheduleAdvertisementRestart() {
  // Restart Advertisement after 1 minute
  auto f = std::bind(&RestartAdvertisement);
  auto when = std::chrono::steady_clock::now() + std::chrono::minutes(1);
  sTaskExecutor.WakeAfter(f, when);
}

bool StartAdvertisement(const Anki::BLEAdvertiseSettings& settings)
{
  const Anki::BLEAdvertiseData& advertisement = settings.GetAdvertisement();
  std::vector<uint8_t> service_uuid = byte_vector_from_uuid_string(advertisement.GetServiceUUID());

  bt_status_t status =
      sBtGattInterface->client->set_adv_data(sBtGattClientIf,
                                             false /* set_scan_rsp */,
                                             advertisement.GetIncludeDeviceName(),
                                             advertisement.GetIncludeTxPowerLevel(),
                                             advertisement.GetMinInterval(),
                                             advertisement.GetMaxInterval(),
                                             settings.GetAppearance(),
                                             advertisement.GetManufacturerData().size(),
                                             (char *)(advertisement.GetManufacturerData().data()),
                                             advertisement.GetServiceData().size(),
                                             (char *)(advertisement.GetServiceData().data()),
                                             service_uuid.size(),
                                             (char *)(service_uuid.data()));

  if (status != BT_STATUS_SUCCESS) {
    loge("failed to set advertisement data");
    return false;
  }
  const Anki::BLEAdvertiseData& scanResponse = settings.GetScanResponse();
  if (!scanResponse.empty()) {
    service_uuid = byte_vector_from_uuid_string(scanResponse.GetServiceUUID());

    status =
        sBtGattInterface->client->set_adv_data(sBtGattClientIf,
                                               true /* set_scan_rsp */,
                                               scanResponse.GetIncludeDeviceName(),
                                               scanResponse.GetIncludeTxPowerLevel(),
                                               scanResponse.GetMinInterval(),
                                               scanResponse.GetMaxInterval(),
                                               settings.GetAppearance(),
                                               scanResponse.GetManufacturerData().size(),
                                               (char *)(scanResponse.GetManufacturerData().data()),
                                               scanResponse.GetServiceData().size(),
                                               (char *)(scanResponse.GetServiceData().data()),
                                               service_uuid.size(),
                                               (char *)(service_uuid.data()));

    if (status != BT_STATUS_SUCCESS) {
      loge("failed to set scan response data");
      return false;
    }
  }

  status = sBtGattInterface->client->listen(sBtGattClientIf, true /* start */);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to start advertising");
    return false;
  }

  sBtAdvertisingEnabled = true;

  ScheduleAdvertisementRestart();

  return true;
}

bool StopAdvertisement() {
  bt_status_t status = sBtGattInterface->client->listen(sBtGattClientIf, false /* start */);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to stop advertising");
    return false;
  }

  sBtAdvertisingEnabled = false;

  return true;
}

void RestartAdvertisement() {
  if (!sBtAdvertisingEnabled) {
    return;
  }

  if (!sBtGattInterface) {
    return;
  }

  bt_status_t status = sBtGattInterface->client->listen(sBtGattClientIf, false /* start */);
  if (status != BT_STATUS_SUCCESS) {
    logw("failed to stop advertising during a restart. status = %s",
         bt_status_t_to_string(status).c_str());
    return;
  }

  status = sBtGattInterface->client->listen(sBtGattClientIf, true /* start */);
  if (status != BT_STATUS_SUCCESS) {
    logw("failed to start advertising during a restart. status = %s",
         bt_status_t_to_string(status).c_str());
  }

  ScheduleAdvertisementRestart();
}

bool SendResponse(int conn_id, int trans_id, int handle, int error, int offset,
                  const std::vector<uint8_t>& value)
{
  btgatt_response_t response = {0};
  response.handle = (uint16_t) handle;
  response.attr_value.handle = handle;
  memcpy(response.attr_value.value, value.data(), value.size());
  response.attr_value.offset = (uint16_t) offset;
  response.attr_value.len = value.size();

  bt_status_t status = sBtGattInterface->server->send_response(conn_id, trans_id, error, &response);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to send response");
    return false;
  }

  return true;
}

bool SendGattIndication(int attribute_handle, int conn_id, int confirm,
                        const std::vector<uint8_t>& value)
{
  if (sBtGattServerIf <= 0) {
    return false;
  }

  bt_status_t status =
      sBtGattInterface->server->send_indication(sBtGattServerIf,
                                                attribute_handle,
                                                conn_id,
                                                value.size(),
                                                confirm,
                                                (char *) value.data());
  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to send indication");
    return false;
  }

  return true;
}

bool WriteGattCharacteristic(const int conn_id,
                             const std::string& uuid,
                             const bool reliable,
                             const std::vector<uint8_t>& value)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  logv("WriteGattCharacteristic(conn_id = %d, uuid = %s, reliable = %s, value = %s)",
       conn_id, uuid.c_str(), reliable ? "true" : "false",
       bt_value_to_string(value.size(), value.data()).c_str());
  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    loge("WriteGattCharacteristic. conn_id (%d) not found", conn_id);
    return false;
  }
  int handle = GetCharacteristicHandleFromConnIDAndUUID(conn_id, uuid);
  if (handle == -1) {
    loge("WriteGattCharacteristic. char uuid(%s) not found", uuid.c_str());
    return false;
  }
  QueueWriteItem(false /* is_descriptor */,
                 conn_id,
                 handle,
                 reliable ? kGattWriteTypeWithResponse : kGattWriteTypeNoResponse,
                 value);
  return true;
}

bool ReadGattCharacteristic(const int conn_id,
                            const std::string& uuid)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  logv("ReadGattCharacteristic(conn_id = %d, uuid = %s)",
       conn_id, uuid.c_str());
  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    loge("ReadGattCharacteristic. conn_id (%d) not found", conn_id);
    return false;
  }
  int handle = GetCharacteristicHandleFromConnIDAndUUID(conn_id, uuid);
  if (handle == -1) {
    loge("ReadGattCharacteristic. char uuid(%s) not found", uuid.c_str());
    return false;
  }
  QueueReadItem(false /* is_descriptor */,
                conn_id,
                handle);
  return true;
}

bool ReadGattDescriptor(const int conn_id,
                        const std::string& char_uuid,
                        const std::string& desc_uuid)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  logv("ReadGattDescriptor(conn_id = %d, char_uuid = %s, desc_uuid = %s)",
       conn_id, char_uuid.c_str(), desc_uuid.c_str());
  auto search = FindOutboundConnectionById(conn_id);
  if (search == sOutboundConnections.end()) {
    loge("ReadGattDescriptor. conn_id (%d) not found", conn_id);
    return false;
  }
  int handle = GetDescriptorHandleFromConnIDAndUUIDs(conn_id, char_uuid, desc_uuid);
  if (handle == -1) {
    loge("ReadGattDescriptor. desc uuid (%s) for char uuid(%s) not found",
         desc_uuid.c_str(), char_uuid.c_str());
    return false;
  }
  QueueReadItem(true /* is_descriptor */,
                conn_id,
                handle);
  return true;
}

bool DisconnectGattPeer(int conn_id)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  EraseGattQueueItemsByConnId(conn_id);
  if (sBtGattServerIf <= 0) {
    return false;
  }

  bt_status_t status;

  auto search = FindOutboundConnectionById(conn_id);
  if (search != sOutboundConnections.end()) {
    search->second.status = BluetoothGattConnectionStatus::Disconnecting;
    DeregisterForNotifications(conn_id, search->second.connection);
    bt_bdaddr_t bda = {0};
    bt_bdaddr_t_from_string(search->second.connection.address, &bda);
    status = sBtGattInterface->client->disconnect(sBtGattClientIf,
                                                  &bda,
                                                  conn_id);
  } else {
    status = sBtGattInterface->server->disconnect(sBtGattServerIf,
                                                  nullptr,
                                                  conn_id);
  }
  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to disconnect");
    return false;
  }

  return true;
}

void DisconnectGattPeerByAddress(const std::string& address)
{
  logv("btstack - DisconnectGattPeerByAddress(%s)", address.c_str());
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  int conn_id = 0;
  auto search = sOutboundConnections.find(address);
  if (search != sOutboundConnections.end()) {
    BluetoothGattConnection& connection = search->second.connection;
    conn_id = connection.conn_id;
    search->second.status = BluetoothGattConnectionStatus::Disconnecting;
    DeregisterForNotifications(conn_id, connection);
    EraseGattQueueItemsByConnId(conn_id);
  }

  if (sBtGattClientIf <= 0) {
    return;
  }
  bt_bdaddr_t bda = {0};
  bt_bdaddr_t_from_string(address, &bda);
  bt_status_t status = sBtGattInterface->client->disconnect(sBtGattClientIf,
                                                            &bda,
                                                            conn_id);
  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to disconnect from %s", address.c_str());
  }

  sBtInterface->remove_bond(&bda);

  return;
}

bool SetScanning(const bool enable)
{
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  if (!sBtGattInterface || !sBtGattInterface->client) {
    return false;
  }

  bt_status_t status = sBtGattInterface->client->scan(enable);

  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to %s scanning. status = %s",
         enable ? "start" : "stop",
         bt_status_t_to_string(status).c_str());
    return false;
  }

  return true;
}

// From Fluoride's stack/include/bt_types.h
#define BT_TRANSPORT_LE 2

bool ConnectToBLEPeripheral(const std::string& address, const bool is_direct)
{
  logv("ConnectToBLEPeripheral(address = %s, is_direct = %s)",
       address.c_str(), is_direct ? "true" : "false");
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  if (!sBtGattInterface || !sBtGattInterface->client) {
    return false;
  }
  auto search = sOutboundConnections.find(address);
  if (search != sOutboundConnections.end()) {
    if (search->second.status == BluetoothGattConnectionStatus::Disconnecting) {
      /* If we are in the process of Disconnecting, treat this the same as if we didn't
       * find anything in sOutboundConnections.
       */
      logv("%s(address = %s) - status is Disconnecting, so will restart the connection flow",
           __FUNCTION__, address.c_str());
    } else {
      /* If we are in any other state, besides Disconnecting, then a connection is in
       * progress and we can just return true.
       */
      return true;
    }
  }
  BluetoothGattConnectionInfo info(address);
  sOutboundConnections[address] = info;
  bt_bdaddr_t bda;
  bt_bdaddr_t_from_string(address, &bda);
  int connection_state = sBtInterface->get_connection_state(&bda);
  if (connection_state) {
    logw("Unexpected connection state (%d) with %s", connection_state, address.c_str());
  }
  bt_status_t status = sBtGattInterface->client->connect(sBtGattClientIf,
                                                         &bda,
                                                         is_direct,
                                                         BT_TRANSPORT_LE);
  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to connect to %s. status = %s",
         address.c_str(), bt_status_t_to_string(status).c_str());
    sOutboundConnections.erase(address);
    return false;
  }

  return true;
}

int RequestConnectionParameterUpdate(const std::string& address,
                                     int min_interval,
                                     int max_interval,
                                     int latency,
                                     int timeout)
{
  logv("RequestConnectionParameterUpdate(address = %s, min_interval = %d, max_interval = %d, latency = %d, timeout = %d)",
       address.c_str(), min_interval, max_interval, latency, timeout);
  std::lock_guard<std::mutex> lock(sBtStackCallbackMutex);
  if (!sBtGattInterface || !sBtGattInterface->client) {
    return kGattStatusFail;
  }

  bt_bdaddr_t bda;
  bt_bdaddr_t_from_string(address, &bda);
  bt_status_t status = sBtGattInterface->client->conn_parameter_update(&bda,
                                                                       min_interval,
                                                                       max_interval,
                                                                       latency,
                                                                       timeout);
  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to change connection parameters for %s. status = %s",
         address.c_str(), bt_status_t_to_string(status).c_str());
  }

  return (int) status;
}


} // namespace BluetoothStack
} // namespace Anki
