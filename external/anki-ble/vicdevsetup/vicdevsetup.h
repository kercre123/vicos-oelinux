/**
 * File: vicdevsetup.h
 *
 * Author: seichert
 * Created: 2/7/2018
 *
 * Description: Help developers setup Victor
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "ipc-client.h"
#include "anki_ble_uuids.h"
#include "exec_command.h"
#include "robot_name.h"

class VicDevSetup : public Anki::BluetoothDaemon::IPCClient {
 public:
  VicDevSetup(struct ev_loop* loop)
      : IPCClient(loop)
      , connection_id_(-1)
      , heartbeat_count_(1)
      , robot_name_(Anki::GetRobotName())
      , adapter_name_set_(false)
      , send_output_callback_(std::bind(&VicDevSetup::SendOutputToConnectedCentral,
                                        this, std::placeholders::_1, std::placeholders::_2))
                              {}

 protected:
  virtual void OnInboundConnectionChange(int connection_id, int connected);
  virtual void OnReceiveMessage(const int connection_id,
                                const std::string& characteristic_uuid,
                                const std::vector<uint8_t>& value);
  virtual void OnPeripheralStateUpdate(const bool advertising,
                                       const int connection_id,
                                       const int connected,
                                       const bool congested);

 private:
  void HandleIncomingMessageFromCentral(const std::vector<uint8_t>& message);
  void SendMessageToConnectedCentral(const std::vector<uint8_t>& value,
                                     const std::string& char_uuid = Anki::kAppReadCharacteristicUUID);
  void SendMessageToConnectedCentral(uint8_t msgID, const std::vector<uint8_t>& value,
                                     const std::string& char_uuid = Anki::kAppReadCharacteristicUUID);
  void SendMessageToConnectedCentral(uint8_t msgID, const std::string& str,
                                     const std::string& char_uuid = Anki::kAppReadCharacteristicUUID);
  void SendOutputToConnectedCentral(int rc, const std::string& output);
  void ExecCommandInBackgroundAndSendOutputToCentral(const std::vector<std::string>& args);
  void SendHeartBeat();
  void HeartBeatCallback(ev::timer& w, int revents);

  int connection_id_;
  std::vector<uint8_t> multipart_message_;
  uint8_t heartbeat_count_;
  std::string robot_name_;
  bool adapter_name_set_;
  ev::timer* heartbeat_timer_;
  Anki::ExecCommandCallback send_output_callback_;
};

