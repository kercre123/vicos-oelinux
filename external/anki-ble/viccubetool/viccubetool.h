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

#include "codeTimer.h"
#include "ipc-client.h"

class VicCubeTool : public Anki::BluetoothDaemon::IPCClient {
 public:
  VicCubeTool(struct ev_loop* loop,
              const std::string& address,
              const std::vector<std::string>& args)
      : IPCClient(loop)
      , address_(address)
      , args_(args)
      , connection_id_(-1)
      , retries_(3)
      , task_executor_(new Anki::TaskExecutor(loop))
      , rssi_min_(-100)
  {}
  ~VicCubeTool();
  void Execute();

  void SetMinRssi(int min) {
    rssi_min_ = min;
  }

 protected:
  virtual void OnScanResults(int error,
                             const std::vector<Anki::BluetoothDaemon::ScanResultRecord>& records);
  virtual void OnOutboundConnectionChange(const std::string& address,
                                          const int connected,
                                          const int connection_id,
                                          const std::vector<Anki::BluetoothDaemon::GattDbRecord>& records);
  virtual void OnReceiveMessage(const int connection_id,
                                const std::string& characteristic_uuid,
                                const std::vector<uint8_t>& value);
  virtual void OnCharacteristicReadResult(const int connection_id,
                                          const int error,
                                          const std::string& characteristic_uuid,
                                          const std::vector<uint8_t>& data);
  virtual void OnDescriptorReadResult(const int connection_id,
                                      const int error,
                                      const std::string& characteristic_uuid,
                                      const std::string& descriptor_uuid,
                                      const std::vector<uint8_t>& data);

 private:
  void OnConnectedToDaemon();
  void ScanForCubes();
  void ConnectToCube();
  void FlashCube(const std::string& pathToFirmware);
  void FlashCubeDVT1(const std::string& pathToFirmware);
  void ConnectRetryTimerCallback(ev::timer& w, int revents);
  void ScanTimerCallback(ev::timer& w, int revents);
  void CubeConnectRetryTimerCallback(ev::timer& w, int revents);
  void IdleCubeConnectionTimerCallback(ev::timer& w, int revents);

  std::string address_;
  std::vector<std::string> args_;
  ev::timer* connect_retry_timer_ = nullptr;
  ev::timer* stop_scan_timer_ = nullptr;
  ev::timer* cube_connect_retry_timer_ = nullptr;
  ev::timer* idle_cube_connection_timer_ = nullptr;
  std::map<std::string, Anki::BluetoothDaemon::ScanResultRecord> scan_records_;
  bool scanning_;
  bool connect_to_first_cube_found_;
  bool flash_cube_after_connect_;
  bool use_dvt1_flasher_;
  bool cube_test_mode_;
  int connection_id_;
  int rssi_min_;
  int retries_;
  Anki::Util::CodeTimer::TimePoint connection_start_time_;
  std::string path_to_firmware_;
  std::string cube_model_number_;
  std::string new_firmware_version_;
  Anki::TaskExecutor* task_executor_;
};
