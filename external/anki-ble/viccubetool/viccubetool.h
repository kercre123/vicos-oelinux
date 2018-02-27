/**
 * File: viccubetool.h
 *
 * Author: seichert
 * Created: 2/15/2018
 *
 * Description: IPC client for VicCubeTool
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "ipc-client.h"

class VicCubeTool : public Anki::BluetoothDaemon::IPCClient {
 public:
  VicCubeTool(struct ev_loop* loop,
              const std::string& address,
              const bool interactive,
              const std::vector<std::string>& args)
      : IPCClient(loop)
      , address_(address)
      , interactive_(interactive)
      , args_(args)
  {}
  ~VicCubeTool();
  void Execute();

 protected:
  virtual void OnScanResults(int error,
                             const std::vector<Anki::BluetoothDaemon::ScanResultRecord>& records);
  virtual void OnOutboundConnectionChange(const std::string& address,
                                          const int connected,
                                          const int connection_id,
                                          const std::vector<Anki::BluetoothDaemon::GattDbRecord>& records);

 private:
  void OnConnectedToDaemon();
  void ScanForCubes();
  void ConnectToCube(const std::string& address);
  void FlashCube(const std::string& address, const std::string& pathToFirmware);
  void DisconnectFromCube();
  void ConnectRetryTimerCallback(ev::timer& w, int revents);
  void ScanTimerCallback(ev::timer& w, int revents);

  std::string address_;
  bool interactive_;
  std::vector<std::string> args_;
  ev::timer* connect_retry_timer_ = nullptr;
  ev::timer* stop_scan_timer_ = nullptr;
  std::map<std::string, Anki::BluetoothDaemon::ScanResultRecord> scan_records_;
  bool connect_to_first_cube_found_;
};

