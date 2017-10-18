//
//  Copyright (C) 2015 Google, Inc.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at:
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifdef BT_LIBCHROME_NDEBUG
#define NDEBUG 1
#endif

#include <base/at_exit.h>
#include <base/bind.h>
#include <base/command_line.h>
#include <base/location.h>
#include <base/logging.h>
#include <base/message_loop/message_loop.h>
#include <base/run_loop.h>

#include <binder/IPCThreadState.h>
#include <binder/ProcessState.h>

#include <bluetooth/binder/IBluetooth.h>

#include <cutils/properties.h>

#include <algorithm>

#include "anki_ble_server.h"

using android::sp;
using ipc::binder::IBluetooth;

// Set up a message loop so that we can schedule timed Heart Beat
// notifications.
static base::MessageLoop main_loop;
static sp<IBluetooth> bt_iface;
static std::unique_ptr<Anki::BLEServer> able;
static bool anki_ble_server_started = false;
static int exit_code = EXIT_SUCCESS;

namespace {

void QuitMessageLoop() {
  // I don't know why both of these calls are necessary but the message loop
  // doesn't stop unless I call both. Bug in base::MessageLoop?
  base::RunLoop().Quit();
  base::MessageLoop::current()->QuitNow();
}

void SetAdapterName() {
  char serialNo[PROPERTY_VALUE_MAX] = {'\0'};
  (void) property_get("ro.serialno", serialNo, "XXXXXX");
  std::string name = "VICTOR_" + std::string(serialNo);
  name.erase(std::remove(name.begin(), name.end(), ':'), name.end());
  std::string current_local_name = bt_iface->GetName();
  if (current_local_name != name) {
    LOG(INFO) << "Current local name is '" << current_local_name << "'."
              << "Setting to '" << name << "'";
    bt_iface->SetName(name);
  }
}

void BLEServerCallback(bool success) {
  if (success) {
    LOG(INFO) << "Anki BLE service started successfully";
    return;
  }

  LOG(ERROR) << "Starting Anki BLE server failed asynchronously";
  main_loop.QuitWhenIdle();
  return;
}

void StartAnkiBLEServer() {
  if (anki_ble_server_started) {
    LOG(INFO) << "Anki BLE Server is already started";
    return;
  }
  DLOG(INFO) << "Attempting to start Anki BLE Server";
  anki_ble_server_started = true;

  SetAdapterName();

  // Create the Anki BLE server.
  DLOG(INFO) << "Creating Anki BLE Server";
  able.reset(new Anki::BLEServer(bt_iface, main_loop.task_runner(), true));
  if (!able->Run(&BLEServerCallback)) {
    LOG(ERROR) << "Failed to start Anki BLE server";
    exit_code = EXIT_FAILURE;
    QuitMessageLoop();
  }
  DLOG(INFO) << "Done creating Anki BLE Server";
}

class BluetoothStateCallback : public ipc::binder::BnBluetoothCallback {
public:
  BluetoothStateCallback(
      scoped_refptr<base::SingleThreadTaskRunner> main_task_runner)
      : main_task_runner_(main_task_runner) {
  }

  ~BluetoothStateCallback() override = default;

  // IBluetoothCallback overrides:
  void OnBluetoothStateChange(
                              bluetooth::AdapterState prev_state,
                              bluetooth::AdapterState new_state) override {
    LOG(INFO) << "Adapter state changed : "
              << bluetooth::AdapterStateToString(prev_state)
              << " -> "
              << bluetooth::AdapterStateToString(new_state);
    if (new_state == bluetooth::AdapterState::ADAPTER_STATE_ON) {
      main_task_runner_->PostTask(FROM_HERE, base::Bind(&StartAnkiBLEServer));
    }
  }

private:
  scoped_refptr<base::SingleThreadTaskRunner> main_task_runner_;
  DISALLOW_COPY_AND_ASSIGN(BluetoothStateCallback);
};


// Handles the case where the Bluetooth process dies.
class BluetoothDeathRecipient : public android::IBinder::DeathRecipient {
 public:
  BluetoothDeathRecipient(
      scoped_refptr<base::SingleThreadTaskRunner> main_task_runner)
      : main_task_runner_(main_task_runner) {
  }

  ~BluetoothDeathRecipient() override = default;

  // android::IBinder::DeathRecipient override:
  void binderDied(const android::wp<android::IBinder>& /* who */) override {
    LOG(ERROR) << "The Bluetooth daemon has died. Aborting.";

    // binderDied executes on a dedicated thread. We need to stop the main loop
    // on the main thread so we post a message to it here. The main loop only
    // runs on the main thread.
    main_task_runner_->PostTask(FROM_HERE, base::Bind(&QuitMessageLoop));

    android::IPCThreadState::self()->stopProcess();
  }

 private:
  scoped_refptr<base::SingleThreadTaskRunner> main_task_runner_;
};

}  // namespace

int main(int argc, char* argv[]) {
  base::AtExitManager exit_manager;
  base::CommandLine::Init(argc, argv);
  logging::LoggingSettings log_settings;

  // Initialize global logging based on command-line parameters (this is a
  // libchrome pattern).
  if (!logging::InitLogging(log_settings)) {
    LOG(ERROR) << "Failed to set up logging";
    return EXIT_FAILURE;
  }


  LOG(INFO) << "Starting GATT Anki BLE Service";

  // Obtain the IBluetooth binder from the service manager service.
  bt_iface = IBluetooth::getClientInterface();
  if (!bt_iface.get()) {
    LOG(ERROR) << "Failed to obtain a handle on IBluetooth";
    return EXIT_FAILURE;
  }

  sp<BluetoothStateCallback> bt_callback = new BluetoothStateCallback(main_loop.task_runner());
  bt_iface->RegisterCallback(bt_callback);

  // Register for death notifications on the IBluetooth binder. This let's us
  // handle the case where the Bluetooth daemon process (bluetoothtbd) dies
  // outside of our control.
  sp<BluetoothDeathRecipient> dr(
      new BluetoothDeathRecipient(main_loop.task_runner()));
  if (android::IInterface::asBinder(bt_iface.get())->linkToDeath(dr) !=
      android::NO_ERROR) {
    LOG(ERROR) << "Failed to register DeathRecipient for IBluetooth";
    return EXIT_FAILURE;
  }

  // Initialize the Binder process thread pool. We have to set this up,
  // otherwise, incoming callbacks from the Bluetooth daemon would block the
  // main thread (in other words, we have to do this as we are a "Binder
  // server").
  android::ProcessState::self()->startThreadPool();

  if (bt_iface->IsEnabled()) {
    StartAnkiBLEServer();
  } else {
    bt_iface->Enable(false);
  }

  // Run the main loop on the main process thread. Binder callbacks will be
  // received in dedicated threads set up by the ProcessState::startThreadPool
  // call above but we use this main loop for sending out heart beat
  // notifications.

  DLOG(INFO) << "Running the main loop";
  main_loop.Run();
  DLOG(INFO) << "Done running the main loop";

  LOG(INFO) << "Exiting";
  return exit_code;
}
