/**
 * File: victor-developer-setup-over-ble.cpp
 *
 * Author: seichert
 * Created: 2/5/2018
 *
 * Description: Help developers setup Victor
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "vicdevsetup.h"
#include "anki_ble_uuids.h"
#include "ble_advertise_settings.h"
#include "constants.h"
#include "exec_command.h"
#include "log.h"
#include "ssh.h"
#include "stringutils.h"
#include "wifi.h"

#include <algorithm>
#include <sstream>

#include <cutils/properties.h>
#include <unistd.h>

void VicDevSetup::OnInboundConnectionChange(int connection_id, int connected)
{
  if (!connected && (connection_id != connection_id_)) {
    return;
  }
  logv("OnInboundConnectionChange(id = %d, connected = %d)",
       connection_id,
       connected);

  Anki::CancelBackgroundCommands();
  if (connected) {
    connection_id_ = connection_id;
    heartbeat_count_ = 1;
    if (!heartbeat_timer_) {
      heartbeat_timer_ = new ev::timer(loop_);
    }
    heartbeat_timer_->set <VicDevSetup, &VicDevSetup::HeartBeatCallback> (this);
    heartbeat_timer_->start(10., 10.);
  } else {
    connection_id_ = -1;
    delete heartbeat_timer_; heartbeat_timer_ = nullptr;
  }
}

void VicDevSetup::OnReceiveMessage(const int connection_id,
                                   const std::string& characteristic_uuid,
                                   const std::vector<uint8_t>& value)
{
  logv("OnReceiveMessage(connection_id = %d, char_uuid = '%s', value.size = %d)",
       connection_id, characteristic_uuid.c_str(), value.size());
  if (connection_id == connection_id_) {
    if (AreCaseInsensitiveStringsEqual(characteristic_uuid, Anki::kAppWriteCharacteristicUUID)) {
      HandleIncomingMessageFromCentral(value);
    }
  }
}

void VicDevSetup::OnPeripheralStateUpdate(const bool advertising,
                                          const int connection_id,
                                          const int connected,
                                          const bool congested)
{
  (void) congested; // currently unused
  if (!adapter_name_set_) {
    SetAdapterName(robot_name_);
    adapter_name_set_ = true;
  }
  OnInboundConnectionChange(connection_id, connected);
  if (!connected && !advertising) {
    Anki::BLEAdvertiseSettings settings;
    settings.GetAdvertisement().SetServiceUUID(Anki::kAnkiSingleMessageService_128_BIT_UUID);
    settings.GetAdvertisement().SetIncludeDeviceName(true);
    std::vector<uint8_t> mdata = Anki::kAnkiBluetoothSIGCompanyIdentifier;
    mdata.push_back(Anki::kVictorProductIdentifier); // distinguish from future Anki products
    settings.GetAdvertisement().SetManufacturerData(mdata);
    StartAdvertising(settings);
  } else if (connected && advertising) {
    StopAdvertising();
  }
}

void VicDevSetup::HandleIncomingMessageFromCentral(const std::vector<uint8_t>& message)
{
  if (message.size() < 2) {
    return;
  }

  uint8_t size = message[0];
  uint8_t msgID = message[1];

  switch (msgID) {
  case Anki::VictorMsg_Command::MSG_B2V_BTLE_DISCONNECT:
    logv("Received request to disconnect");
    Disconnect(connection_id_);
    break;
  case Anki::VictorMsg_Command::MSG_B2V_CORE_PING_REQUEST:
    logv("Received ping request");
    SendMessageToConnectedCentral({0x01, Anki::VictorMsg_Command::MSG_V2B_CORE_PING_RESPONSE});
    break;
  case Anki::VictorMsg_Command::MSG_B2V_HEARTBEAT:
    logv("Received heartbeat");
    // Nothing to do here, we already send a periodic heartbeat of our own back to the central
    break;
  case Anki::VictorMsg_Command::MSG_B2V_WIFI_START:
    logv("Received WiFi start");
    Anki::EnableWiFiInterface(true, send_output_callback_);
    break;
  case Anki::VictorMsg_Command::MSG_B2V_WIFI_STOP:
    logv("Received WiFi stop");
    Anki::EnableWiFiInterface(false, send_output_callback_);
    break;
  case Anki::VictorMsg_Command::MSG_B2V_WIFI_SCAN:
    {
      logv("Received WiFi scan");
      std::vector<Anki::WiFiScanResult> results = Anki::ScanForWiFiAccessPoints();
      std::vector<uint8_t> packed_results = Anki::PackWiFiScanResults(results);
      SendMessageToConnectedCentral(Anki::VictorMsg_Command::MSG_V2B_WIFI_SCAN_RESULTS,
                                    packed_results);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_WIFI_SET_CONFIG_EXT:
    {
      logv("Receive WiFi Set Config Ext");
      std::vector<uint8_t> packedWiFiConfig(message.begin() + 2, message.end());
      std::vector<Anki::WiFiConfig> networks = Anki::UnPackWiFiConfig(packedWiFiConfig);
      Anki::SetWiFiConfig(networks, send_output_callback_);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_SSH_SET_AUTHORIZED_KEYS:
    {
      logv("Receive SSH authorized keys");
      // Payload is text for the .ssh/authorized_keys file
      std::string keys(message.begin() + 2, message.end());
      int rc = Anki::SetSSHAuthorizedKeys(keys);
      std::string output;
      if (rc) {
        output = "Failed to set SSH authorized keys";
      } else {
        output = "SSH authorized keys set";
      }
      SendOutputToConnectedCentral(rc, output);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_DEV_PING_WITH_DATA_REQUEST:
    {
      logv("Received ping with data request");
      std::vector<uint8_t> data = message;
      data[1] = Anki::VictorMsg_Command::MSG_V2B_DEV_PING_WITH_DATA_RESPONSE;
      SendMessageToConnectedCentral(data);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_DEV_RESTART_ADBD:
    logv("Received restart adbd request");
    ExecCommandInBackgroundAndSendOutputToCentral({"/etc/initscripts/adbd", "stop"});
    ExecCommandInBackgroundAndSendOutputToCentral({"/etc/initscripts/adbd", "start"});
    break;
  case Anki::VictorMsg_Command::MSG_B2V_DEV_EXEC_CMD_LINE:
    {
      logv("Received command line to execute");
      std::vector<std::string> args;
      for (auto it = message.begin() + 2; it != message.end(); ) {
        auto terminator = std::find(it, message.end(), 0);
        if (terminator == message.end()) {
          break;
        }
        std::string arg(it, terminator);
        args.push_back(arg);
        it = terminator + 1;
      }
      ExecCommandInBackgroundAndSendOutputToCentral(args);
    }
    break;
  case Anki::VictorMsg_Command::MSG_B2V_MULTIPART_START:
    multipart_message_.clear();
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(multipart_message_));
    break;
  case Anki::VictorMsg_Command::MSG_B2V_MULTIPART_CONTINUE:
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(multipart_message_));
    break;
  case Anki::VictorMsg_Command::MSG_B2V_MULTIPART_FINAL:
    std::copy(message.begin() + 2, message.end(),
              std::back_inserter(multipart_message_));
    HandleIncomingMessageFromCentral(multipart_message_);
    break;
  default:
    break;
  }

}

void VicDevSetup::SendMessageToConnectedCentral(const std::vector<uint8_t>& value,
                                                const std::string& char_uuid)
{
  if (connection_id_ == -1) {
    return;
  }
  if (value.size() <= Anki::kAnkiVictorMsgMaxSize) {
    SendMessage(connection_id_,
                char_uuid,
                true,
                value);
  } else {
    // Fragment large message into multipart messages to queue
    size_t offset = 0;
    while (offset < value.size()) {
      std::vector<uint8_t> message;
      uint8_t msgID;
      uint8_t msgSize = Anki::kAnkiVictorMsgBaseSize;
      if (offset == 0) {
        msgID = Anki::VictorMsg_Command::MSG_V2B_MULTIPART_START;
        msgSize += Anki::kAnkiVictorMsgPayloadMaxSize;
      } else if (value.size() - offset > Anki::kAnkiVictorMsgPayloadMaxSize) {
        msgID = Anki::VictorMsg_Command::MSG_V2B_MULTIPART_CONTINUE;
        msgSize += Anki::kAnkiVictorMsgPayloadMaxSize;
      } else {
        msgID = Anki::VictorMsg_Command::MSG_V2B_MULTIPART_FINAL;
        msgSize += (value.size() - offset);
      }
      message.push_back(msgSize);
      message.push_back(msgID);
      std::copy(value.begin() + offset,
                value.begin() + offset + msgSize - 1,
                std::back_inserter(message));
      SendMessageToConnectedCentral(message, char_uuid);
      offset += (msgSize - 1);
    }
  }
}

void VicDevSetup::SendMessageToConnectedCentral(uint8_t msgID, const std::vector<uint8_t>& value,
                                                const std::string& char_uuid)
{
  if (connection_id_ == -1) {
    return;
  }
  uint8_t msgSize = std::min(static_cast<unsigned int>(std::numeric_limits<uint8_t>::max()),
                             value.size() + 1);
  std::vector<uint8_t> msg {msgSize, msgID};
  msg.insert(std::end(msg), std::begin(value), std::end(value));
  SendMessageToConnectedCentral(msg, char_uuid);
}

void VicDevSetup::SendMessageToConnectedCentral(uint8_t msgID, const std::string& str,
                                                const std::string& char_uuid)
{
  if (connection_id_ == -1) {
    return;
  }
  std::vector<uint8_t> value(str.begin(), str.end());
  value.push_back(0);
  SendMessageToConnectedCentral(msgID, value, char_uuid);
}

void VicDevSetup::SendOutputToConnectedCentral(int rc, const std::string& output)
{
  if (connection_id_ == -1) {
    return;
  }
  std::ostringstream oss;
  if (rc) {
    oss << "Error: " << rc << std::endl;
  }
  oss << output;
  SendMessageToConnectedCentral(Anki::VictorMsg_Command::MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE, oss.str());
}

void VicDevSetup::ExecCommandInBackgroundAndSendOutputToCentral(const std::vector<std::string>& args)
{
  if (connection_id_ == -1) {
    return;
  }
  if (args.empty()) {
    SendMessageToConnectedCentral(Anki::VictorMsg_Command::MSG_V2B_DEV_EXEC_CMD_LINE_RESPONSE,
                                  "Invalid argument: args in empty");
    return;
  }
  Anki::ExecCommandInBackground(args, send_output_callback_);
}

void VicDevSetup::SendHeartBeat() {
  if (connection_id_ == -1) {
    return;
  }

  std::vector<uint8_t> value;
  value.push_back(heartbeat_count_++);
  SendMessageToConnectedCentral(Anki::VictorMsg_Command::MSG_V2B_HEARTBEAT, value);
  return;
}

void VicDevSetup::HeartBeatCallback(ev::timer& w, int revents)
{
  if (revents && ev::TIMER) {
    SendHeartBeat();
  }
}

static struct ev_loop* sDefaultLoop = ev_default_loop(EVBACKEND_SELECT);
static VicDevSetup* sVicDevSetup = nullptr;

static void ExitHandler(int status = 0) {
  delete sVicDevSetup; sVicDevSetup = nullptr;
  _exit(status);
}

static void SignalCallback(struct ev_loop* loop, struct ev_signal* w, int revents)
{
  logv("Exiting for signal %d", w->signum);
  ev_unloop(loop, EVUNLOOP_ALL);
  ExitHandler();
}

static struct ev_signal sIntSig;
static struct ev_signal sTermSig;

static struct ev_timer sTimer;

static void TimerWatcherCallback(struct ev_loop* loop, struct ev_timer* w, int revents)
{
  if (sVicDevSetup) {
    if (!sVicDevSetup->IsConnected()) {
      (void) sVicDevSetup->Connect();
    }
    if (sVicDevSetup->IsConnected()) {
      bool result =
          sVicDevSetup->SendIPCMessageToServer(Anki::BluetoothDaemon::IPCMessageType::Ping);
      if (!result) {
        loge("Failed to send IPC message to server");
        ExitHandler();
      }
      ev_timer_set(&sTimer, 20., 0.);
    } else {
      ev_timer_set(&sTimer, 2., 0.);
    }
    ev_timer_start(sDefaultLoop, &sTimer);
  }
}


int main(int argc, char *argv[]) {
  ev_signal_init(&sIntSig, SignalCallback, SIGINT);
  ev_signal_start(sDefaultLoop, &sIntSig);
  ev_signal_init(&sTermSig, SignalCallback, SIGTERM);
  ev_signal_start(sDefaultLoop, &sTermSig);

  setAndroidLoggingTag("vdsob");
  setMinLogLevel(property_get_int32("persist.anki.vdsob.log_level", kLogLevelInfo));
  logv("%s", "Victor Developer Setup over BLE launched");
  sVicDevSetup = new VicDevSetup(sDefaultLoop);
  ev_timer_init(&sTimer, TimerWatcherCallback, 2., 0.);
  ev_timer_start(sDefaultLoop, &sTimer);
  ev_loop(sDefaultLoop, 0);
  ExitHandler();
  return 0;
}
