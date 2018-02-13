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
        OnSendMessage(args->connection_id,
                      std::string(args->characteristic_uuid),
                      args->reliable,
                      std::vector<uint8_t>(args->value, args->value + args->length));
      }
      break;
    case IPCMessageType::Disconnect:
      logv("ipc-server: Disconnect received");
      {
        DisconnectArgs* args = (DisconnectArgs *) data.data();
        OnDisconnect(args->connection_id);
      }
      break;
    default:
      loge("ipc-server: Unknown IPC message (%d)", (int) type);
      break;
  }
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
  strncpy(args->characteristic_uuid,
          characteristic_uuid.c_str(),
          sizeof(args->characteristic_uuid) - 1);
  args->length = (uint32_t) data.size();
  memcpy(args->value, data.data(), data.size());
  SendMessageToAllPeers(IPCMessageType::OnReceiveMessage,
                        args_length,
                        (uint8_t *) args);
  free(args);
}

} // namespace BluetoothDaemon
} // namespace Anki

