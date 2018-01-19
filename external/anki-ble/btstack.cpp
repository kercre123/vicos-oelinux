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

#include "btstack.h"

#include "log.h"
#include "btutils.h"
#include "gatt_constants.h"
#include <hardware/bluetooth.h>
#include <hardware/bt_gatt.h>
#include <hardware/bt_gatt_client.h>
#include <hardware/bt_gatt_server.h>
#include <hardware/hardware.h>
#include <hardware/vendor.h>
#include <string.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

#ifdef __cplusplus
extern "C" {
#endif

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
  logv("%s", __FUNCTION__);
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
  logv("%s", __FUNCTION__);
}

void btgattc_open_cb(int conn_id, int status, int clientIf, bt_bdaddr_t* bda)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(conn_id = %d, status = %s, clientIf = %d, bda = %s)",
       __FUNCTION__, conn_id, bt_status_t_to_string(bt_status).c_str(),
       clientIf, bt_bdaddr_t_to_string(bda).c_str());
}

void btgattc_close_cb(int conn_id, int status, int clientIf, bt_bdaddr_t* bda)
{
  bt_status_t bt_status = (bt_status_t) status;
  logv("%s(conn_id = %d, status = %s, clientIf = %d, bda = %s)",
       __FUNCTION__, conn_id, bt_status_t_to_string(bt_status).c_str(),
       clientIf, bt_bdaddr_t_to_string(bda).c_str());
}

void btgattc_search_complete_cb(int conn_id, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_register_for_notification_cb(int conn_id, int registered,
                                          int status, uint16_t handle)
{
  logv("%s", __FUNCTION__);
}

void btgattc_notify_cb(int conn_id, btgatt_notify_params_t *p_data)
{
  logv("%s", __FUNCTION__);
}

void btgattc_read_characteristic_cb(int conn_id, int status,
                                    btgatt_read_params_t *p_data)
{
  logv("%s", __FUNCTION__);
}

void btgattc_write_characteristic_cb(int conn_id, int status,
                                     uint16_t handle)
{
  logv("%s", __FUNCTION__);
}

void btgattc_execute_write_cb(int conn_id, int status)
{
  logv("%s", __FUNCTION__);
}

void btgattc_read_descriptor_cb(int conn_id, int status, btgatt_read_params_t *p_data)
{
  logv("%s", __FUNCTION__);
}

void btgattc_write_descriptor_cb(int conn_id, int status, uint16_t handle)
{
  logv("%s", __FUNCTION__);
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
  logv("%s(conn_id = %d, congested = %s)",
       __FUNCTION__, conn_id, congested ? "true" : "false");
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
  logv("%s", __FUNCTION__);
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
  logv("%s(conn_id = %d, server_if = %d, connected = %d, bda = %s)",
       __FUNCTION__, conn_id, server_if, connected, bt_bdaddr_t_to_string(bda).c_str());
  if (sBluetoothGattService && sBluetoothGattService->connection_cb) {
    sBluetoothGattService->connection_cb(conn_id, connected);
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
        break;
      }
    }
    if (found) {
      break;
    }
  }
  if (!found) {
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
      bt_uuid_t uuid;
      bt_uuid_t_from_string(it.uuid, &uuid);
      if (bt_uuid_t_equals(&uuid, char_id)) {
        it.char_handle = char_handle;
        break;
      }
    }
    bt_status = AddNextCharacteristicOrDescriptor();
  }

  if (bt_status != BT_STATUS_SUCCESS) {
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
        bt_uuid_t uuid;
        bt_uuid_t_from_string(dit.uuid, &uuid);
        if (bt_uuid_t_equals(&uuid, descr_id)) {
          dit.desc_handle = descr_handle;
          found = true;
          break;
        }
      }
      if (found) {
        break;
      }
    }
    bt_status = AddNextCharacteristicOrDescriptor();
  }
  if (bt_status != BT_STATUS_SUCCESS) {
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

  sBtGattOpError = !(bt_status == BT_STATUS_SUCCESS);
  sBtGattServiceStarted =
      ((bt_status == BT_STATUS_SUCCESS) && (srvc_handle == sBluetoothGattService->service_handle));
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
  if (sBluetoothGattService && sBluetoothGattService->request_read_cb) {
    sBluetoothGattService->request_read_cb(conn_id, trans_id, attr_handle, offset);
  }
}

void btgatts_request_write_cb(int conn_id, int trans_id, bt_bdaddr_t *bda, int attr_handle,
                              int offset, int length, bool need_rsp, bool is_prep,
                              uint8_t* value)
{
  logv("%s(conn_id = %d, trans_id = %d, bda = %s, attr_handle = %d, offset = %d, length = %d, need_rsp = %s, is_prep = %s, value = 0x%lx)",
       __FUNCTION__, conn_id, trans_id, bt_bdaddr_t_to_string(bda).c_str(), attr_handle,
       offset, length, need_rsp ? "true" : "false", is_prep ? "true" : "false",
       (uint32_t) value);
  if (is_prep) {
    /* We do not support prepared writes */
    std::vector<uint8_t> dummy;
    (void) SendResponse(conn_id, trans_id, attr_handle,
                        kGattErrorRequestNotSupported, offset, dummy);
    return;
  }
  if (sBluetoothGattService && sBluetoothGattService->request_write_cb) {
    std::vector<uint8_t> val(value, value + length);
    sBluetoothGattService->request_write_cb(conn_id,
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
  if (sBluetoothGattService && sBluetoothGattService->indication_sent_cb) {
    sBluetoothGattService->indication_sent_cb(conn_id, status);
  }
}

void btgatts_congestion_cb(int conn_id, bool congested)
{
  logv("%s(conn_id = %d, congested = %s)",
       __FUNCTION__, conn_id, congested ? "true" : "false");
  if (sBluetoothGattService && sBluetoothGattService->congestion_cb) {
    sBluetoothGattService->congestion_cb(conn_id, congested);
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
  sBtStackCallbackReceived = false;
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
  return sBtAdapterName;
}

bool SetAdapterName(const std::string& name) {
  if (!sBtInterface || !sBtAdapterEnabled) {
    return false;
  }
  if (name == sBtAdapterName) {
    return true;
  }
  bt_property_t property;
  property.type = BT_PROPERTY_BDNAME;
  bt_bdname_t bd_name;
  (void) memset(&bd_name, 0, sizeof(bd_name));
  strncpy((char *) &bd_name.name[0], name.c_str(), sizeof(bd_name) - 1);
  property.val = &bd_name;
  property.len = strlen((char *)bd_name.name);
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
  sBtStackCallbackReceived = false;
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
  sBtStackCallbackReceived = false;
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

  btgatt_srvc_id_t srvc_id;
  memset(&srvc_id, 0, sizeof(srvc_id));
  bt_uuid_t_from_string(service->uuid, &(srvc_id.id.uuid));
  srvc_id.id.inst_id = 0;
  srvc_id.is_primary = 1;

  int num_handles = 3 + service->characteristics.size();
  for (auto const& it : service->characteristics) {
    num_handles += it.descriptors.size();
  }

  sBtStackCallbackReceived = false;
  sBtGattServiceAdded = false;
  sBtGattOpError = false;
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
  sBtStackCallbackReceived = false;
  sBtGattServiceStarted = false;
  sBtGattOpError = false;
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
      loge("Failed to add GATT service");
      return false;
    }
  } else {
    loge("gatt service not started after 1000 milliseconds");
    return false;
  }

}

bool StartAdvertisement(std::string service_uuid)
{
  bt_uuid_t uuid;
  bt_uuid_t_from_string(service_uuid, &uuid);

  bt_status_t status = sBtGattInterface->client->set_adv_data(sBtGattClientIf,
                                                              false /* set_scan_rsp */,
                                                              true /* include_name */,
                                                              true /* include_txpower */,
                                                              100 /* min_interval */,
                                                              1000 /* max_interval */,
                                                              0 /* appearance */,
                                                              0 /* manufacturer_len */,
                                                              nullptr /* manufacturer_data */,
                                                              0 /* service_data_len */,
                                                              nullptr /* service_data */,
                                                              0 /* service_uuid_len */,
                                                              nullptr /* service_uuid */);

  if (status != BT_STATUS_SUCCESS) {
    loge("failed to set advertisement data");
    return false;
  }

  status = sBtGattInterface->client->set_adv_data(sBtGattClientIf,
                                                  true /* set_scan_rsp */,
                                                  false /* include_name */,
                                                  false /* include_txpower */,
                                                  100 /* min_interval */,
                                                  1000 /* max_interval */,
                                                  0 /* appearance */,
                                                  0 /* manufacturer_len */,
                                                  nullptr /* manufacturer_data */,
                                                  0 /* service_data_len */,
                                                  nullptr /* service_data */,
                                                  sizeof(uuid.uu) /* service_uuid_len */,
                                                  (char *) uuid.uu /* service_uuid */);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to set scan response data");
    return false;
  }

  status = sBtGattInterface->client->listen(sBtGattClientIf, true /* start */);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to start advertising");
    return false;
  }

  return true;
}

bool StopAdvertisement() {
  bt_status_t status = sBtGattInterface->client->listen(sBtGattClientIf, false /* start */);
  if (status != BT_STATUS_SUCCESS) {
    loge("failed to stop advertising");
    return false;
  }

  return true;
}


bool SendResponse(int conn_id, int trans_id, int handle, int error, int offset,
                  const std::vector<uint8_t>& value)
{
  btgatt_response_t response;
  memset(&response, 0, sizeof(response));
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

bool DisconnectGattPeer(int conn_id)
{
  if (sBtGattServerIf <= 0) {
    return false;
  }

  bt_status_t status = sBtGattInterface->server->disconnect(sBtGattServerIf,
                                                            nullptr,
                                                            conn_id);
  if (status != BT_STATUS_SUCCESS) {
    loge("Failed to disconnect");
    return false;
  }

  return true;
}
