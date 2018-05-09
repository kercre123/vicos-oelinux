/**
 * File: ipc-server.cpp
 *
 * Author: seichert
 * Created: 2/5/2018
 *
 * Description: IPC Server for ankibluetoothd
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "ipc-server.h"
#include "include_ev.h"
#include "log.h"
#include "memutils.h"
#include "strlcpy.h"

#include <algorithm>

#include <errno.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace Anki {
namespace BluetoothDaemon {

IPCServer::IPCServer(struct ev_loop* loop)
    : IPCEndpoint(loop)
    , accept_watcher_(nullptr)
{
  (void) unlink(Anki::BluetoothDaemon::kSocketName.c_str());
}

IPCServer::~IPCServer()
{
  delete accept_watcher_; accept_watcher_ = nullptr;
  (void) unlink(Anki::BluetoothDaemon::kSocketName.c_str());
}

bool IPCServer::ListenForIPCConnections()
{
  mode_t previous_mask = umask(S_IROTH | S_IWOTH | S_IXOTH);
  int rc = bind(sockfd_, (struct sockaddr *)&addr_, sizeof(addr_));
  (void) umask(previous_mask);
  if (rc == -1) {
    loge("ipc-server: bind. errno = %d (%s)", errno, strerror(errno));
    CloseSocket();
    return false;
  }
  rc = listen(sockfd_, 5);
  if (rc == -1) {
    loge("ipc-server: listen. errno = %d (%s)", errno, strerror(errno));
    CloseSocket();
    return false;
  }
  accept_watcher_ = new ev::io(loop_);
  accept_watcher_->set <IPCServer, &IPCServer::AcceptWatcherCallback> (this);
  accept_watcher_->start(sockfd_, ev::READ);
}

void IPCServer::AcceptWatcherCallback(ev::io& w, int revents)
{
  if (revents & ev::ERROR) {
    loge("ipc-server: AcceptWatcherCallback - ev::ERROR");
    return;
  }
  if (revents & ev::READ) {
    logv("ipc-server: AcceptWatcherCallback. ev::READ");
    int fd = accept4(w.fd, nullptr, nullptr, SOCK_NONBLOCK);
    if (fd == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      loge("ipc-server. accept errno = %d (%s)", errno, strerror(errno));
      return;
    }
    AddPeerByFD(fd);
    OnNewIPCClient(fd);
  }
}

void IPCServer::OnReceiveIPCMessage(const int sockfd,
                                    const IPCMessageType type,
                                    const std::vector<uint8_t>& data)
{
  switch(type) {
    case IPCMessageType::Ping:
      logv("ipc-server: Ping received");
      SendMessageToPeer(sockfd, IPCMessageType::OnPingResponse, 0, nullptr);
      break;
    case IPCMessageType::OnPingResponse:
      logv("ipc-server: Ping response received");
      break;
    case IPCMessageType::SendMessage:
      logv("ipc-server: SendMessage received");
      {
        SendMessageArgs* args = (SendMessageArgs *) data.data();
        SendMessage(args->connection_id,
                    std::string(args->characteristic_uuid),
                    args->reliable,
                    std::vector<uint8_t>(args->value, args->value + args->length));
      }
      break;
    case IPCMessageType::CharacteristicReadRequest:
      logv("ipc-server: CharacteristicReadRequest received");
      {
        CharacteristicReadRequestArgs* args = (CharacteristicReadRequestArgs *) data.data();
        ReadCharacteristic(args->connection_id,
                           std::string(args->characteristic_uuid));
      }
      break;
    case IPCMessageType::DescriptorReadRequest:
      logv("ipc-server: DescriptorReadRequest received");
      {
        DescriptorReadRequestArgs* args = (DescriptorReadRequestArgs *) data.data();
        ReadDescriptor(args->connection_id,
                       std::string(args->characteristic_uuid),
                       std::string(args->descriptor_uuid));
      }
      break;
    case IPCMessageType::Disconnect:
      logv("ipc-server: Disconnect received");
      {
        DisconnectArgs* args = (DisconnectArgs *) data.data();
        Disconnect(args->connection_id);
      }
      break;
    case IPCMessageType::StartAdvertising:
      logv("ipc-server: StartAdvertising received");
      {
        StartAdvertisingArgs* args = (StartAdvertisingArgs *) data.data();
        BLEAdvertiseSettings settings;
        settings.SetAppearance(args->appearance);
        const std::vector<const AdvertisingData *>
            ad_data = {&(args->advertisement), &(args->scan_response)};
        std::vector<BLEAdvertiseData*>
            ble_ad_data = {&(settings.GetAdvertisement()), &(settings.GetScanResponse())};
        for (int i = 0 ; i < ble_ad_data.size(); i++) {
          const AdvertisingData* src = ad_data[i];
          BLEAdvertiseData* dst = ble_ad_data[i];
          dst->SetIncludeDeviceName(src->include_device_name);
          dst->SetIncludeTxPowerLevel(src->include_tx_power_level);
          if (src->manufacturer_data_len > 0) {
            std::vector<uint8_t> mdata(src->manufacturer_data,
                                       src->manufacturer_data + src->manufacturer_data_len);
            dst->SetManufacturerData(mdata);
          }
          if (src->service_data_len > 0) {
            std::vector<uint8_t> sdata(src->service_data,
                                       src->service_data + src->service_data_len);
            dst->SetServiceData(sdata);
          }
          if (src->have_service_uuid) {
            dst->SetServiceUUID(std::string(src->service_uuid));
          }
          dst->SetMinInterval(src->min_interval);
          dst->SetMaxInterval(src->max_interval);
        }

        StartAdvertising(sockfd, settings);
      }
      break;
    case IPCMessageType::StopAdvertising:
      logv("ipc-server: StopAdvertising received");
      {
        StopAdvertising();
      }
      break;
    case IPCMessageType::StartScan:
      logv("ipc-server: StartScan received");
      {
        StartScanArgs* args = (StartScanArgs *) data.data();
        StartScan(sockfd, std::string(args->service_uuid));
      }
      break;
    case IPCMessageType::StopScan:
      logv("ipc-server: StopScan received");
      {
        StopScan(sockfd);
      }
      break;
    case IPCMessageType::ConnectToPeripheral:
      logv("ipc-server: ConnectToPeripheral received");
      {
        ConnectToPeripheralArgs* args = (ConnectToPeripheralArgs *) data.data();
        ConnectToPeripheral(sockfd, std::string(args->address));
      }
      break;
    case IPCMessageType::RequestConnectionParameterUpdate:
      logv("ipc-server: RequestConnectionParameterUpdate received");
      {
        RequestConnectionParameterUpdateArgs* args =
            (RequestConnectionParameterUpdateArgs *) data.data();
        RequestConnectionParameterUpdate(sockfd,
                                         std::string(args->address),
                                         args->min_interval,
                                         args->max_interval,
                                         args->latency,
                                         args->timeout);
      }
      break;
    case IPCMessageType::SetAdapterName:
      logv("ipc-server: SetAdapterName received");
      {
        SetAdapterNameArgs* args = (SetAdapterNameArgs *) data.data();
        SetAdapterName(std::string(args->name));
      }
      break;
    case IPCMessageType::DisconnectByAddress:
      logv("ipc-server: DisconnectByAddress");
      {
        DisconnectByAddressArgs* args = (DisconnectByAddressArgs *) data.data();
        DisconnectByAddress(std::string(args->address));
      }
      break;
    default:
      loge("ipc-server: Unknown IPC message (%d)", (int) type);
      break;
  }
}

void IPCServer::OnPeerClose(const int sockfd)
{
  IPCEndpoint::OnPeerClose(sockfd);
}

void IPCServer::OnPeripheralStateUpdate(const bool advertising,
                                        const int connection_id,
                                        const int connected,
                                        const bool congested)
{
  OnPeripheralStateUpdateArgs args = {
    .advertising = advertising,
    .connection_id = connection_id,
    .connected = connected,
    .congested = congested
  };
  SendMessageToAllPeers(IPCMessageType::OnPeripheralStateUpdate,
                        sizeof(args),
                        (uint8_t *) &args);
}

void IPCServer::OnInboundConnectionChange(const int connection_id, const int connected)
{
  OnInboundConnectionChangeArgs args = {
    .connection_id = connection_id,
    .connected = connected
  };
  SendMessageToAllPeers(IPCMessageType::OnInboundConnectionChange,
                        sizeof(args),
                        (uint8_t *) &args);
}

void IPCServer::OnReceiveMessage(const int connection_id,
                                 const std::string& characteristic_uuid,
                                 const std::vector<uint8_t>& data)
{
  OnReceiveMessageArgs* args;
  uint32_t args_length = sizeof(*args) + data.size();
  args = (OnReceiveMessageArgs *) malloc_zero(args_length);
  args->connection_id = connection_id;
  (void) strlcpy(args->characteristic_uuid,
                 characteristic_uuid.c_str(),
                 sizeof(args->characteristic_uuid));
  args->length = (uint32_t) data.size();
  memcpy(args->value, data.data(), data.size());
  SendMessageToAllPeers(IPCMessageType::OnReceiveMessage,
                        args_length,
                        (uint8_t *) args);
  free(args);
}

void IPCServer::OnCharacteristicReadResult(const int connection_id,
                                           const int error,
                                           const std::string& characteristic_uuid,
                                           const std::vector<uint8_t>& data)
{
  OnCharacteristicReadResultArgs* args;
  uint32_t args_length = sizeof(*args) + data.size();
  args = (OnCharacteristicReadResultArgs *) malloc_zero(args_length);
  args->connection_id = connection_id;
  args->error = error;
  (void) strlcpy(args->characteristic_uuid,
                 characteristic_uuid.c_str(),
                 sizeof(args->characteristic_uuid));
  args->length = (uint32_t) data.size();
  memcpy(args->value, data.data(), data.size());
  SendMessageToAllPeers(IPCMessageType::OnCharacteristicReadResult,
                        args_length,
                        (uint8_t *) args);
  free(args);
}

void IPCServer::OnDescriptorReadResult(const int connection_id,
                                       const int error,
                                       const std::string& characteristic_uuid,
                                       const std::string& descriptor_uuid,
                                       const std::vector<uint8_t>& data)
{
  OnDescriptorReadResultArgs* args;
  uint32_t args_length = sizeof(*args) + data.size();
  args = (OnDescriptorReadResultArgs *) malloc_zero(args_length);
  args->connection_id = connection_id;
  args->error = error;
  (void) strlcpy(args->characteristic_uuid,
                 characteristic_uuid.c_str(),
                 sizeof(args->characteristic_uuid));
  (void) strlcpy(args->descriptor_uuid,
                 descriptor_uuid.c_str(),
                 sizeof(args->descriptor_uuid));
  args->length = (uint32_t) data.size();
  memcpy(args->value, data.data(), data.size());
  SendMessageToAllPeers(IPCMessageType::OnCharacteristicReadResult,
                        args_length,
                        (uint8_t *) args);
  free(args);
}

void IPCServer::OnScanResults(int error, const std::vector<ScanResultRecord>& records)
{
  OnScanResultsArgs* args;
  uint32_t args_length = sizeof(*args) + (sizeof(ScanResultRecord) * records.size());
  args = (OnScanResultsArgs *) malloc_zero(args_length);
  args->error = error;
  args->record_count = (int) records.size();
  for (int i = 0 ; i < args->record_count ; i++) {
    memcpy(&(args->records[i]), &(records[i]), sizeof(ScanResultRecord));
  }
  SendMessageToAllPeers(IPCMessageType::OnScanResults,
                        args_length,
                        (uint8_t *) args);
  free(args);
}

void IPCServer::OnRequestConnectionParameterUpdateResult(const int sockfd,
                                                         const std::string& address,
                                                         const int status)
{
  OnRequestConnectionParameterUpdateResultArgs args = {0};
  (void) strlcpy(args.address, address.c_str(), sizeof(args.address));
  args.status = status;
  SendMessageToPeer(sockfd,
                    IPCMessageType::OnRequestConnectionParameterUpdateResult,
                    sizeof(args),
                    (uint8_t *) &args);
}

void IPCServer::OnOutboundConnectionChange(const std::string& address,
                                           const int connected,
                                           const int connection_id,
                                           const std::vector<GattDbRecord>& records)
{
  logv("ipc-server: OnOutboundConnectionChange(address = %s, connected = %d, connection_id = %d, records.size = %d)",
       address.c_str(), connected, connection_id, records.size());
  OnOutboundConnectionChangeArgs* args;
  uint32_t args_length = sizeof(*args) + (sizeof(GattDbRecord) * records.size());
  args = (OnOutboundConnectionChangeArgs *) malloc_zero(args_length);
  (void) strlcpy(args->address, address.c_str(), sizeof(args->address));
  args->connected = connected;
  args->connection_id = connection_id;
  args->num_gatt_db_records = records.size();
  for (int i = 0 ; i < args->num_gatt_db_records ; i++) {
    memcpy(&(args->records[i]), &(records[i]), sizeof(GattDbRecord));
  }
  SendMessageToAllPeers(IPCMessageType::OnOutboundConnectionChange,
                        args_length,
                        (uint8_t *) args);
  free(args);
}

} // namespace BluetoothDaemon
} // namespace Anki

