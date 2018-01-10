/**
 * File: main.cpp
 *
 * Author: seichert
 * Created: 1/10/2018
 *
 * Description: main entry point for anki bluetooth daemon
 *
 * Copyright: Anki, Inc. 2018
 *
 **/


#include "btstack.h"
#include "device_info.h"
#include "log.h"
#include "peripheral.h"

#include <signal.h>
#include <unistd.h>

#include <thread>
#include <mutex>
#include <condition_variable>

static void ExitHandler(int status = 0) {
  UnLoadBtStack();
  _exit(status);
}

static void SignalHandler(int sig) {
  ExitHandler();
}

static void SetupSignalHandler() {
  struct sigaction new_action, old_action;

  new_action.sa_handler = SignalHandler;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;

  (void) sigaction(SIGINT, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN) {
    (void) sigaction(SIGINT, &new_action, NULL);
  }
  (void) sigaction(SIGHUP, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN) {
    (void) sigaction(SIGHUP, &new_action, NULL);
  }
  (void) sigaction(SIGTERM, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN) {
    (void) sigaction(SIGTERM, &new_action, NULL);
  }
}

static std::mutex m;
static std::condition_variable cv;

void WaitForever() {
  std::unique_lock<std::mutex> lk(m);
  cv.wait(lk, []{return false;});
}

int main(int argc, char *argv[]) {
  SetupSignalHandler();
  setAndroidLoggingTag("ankibtd");
  setMinLogLevel(kLogLevelVerbose);
  if (!LoadBtStack()) {
    ExitHandler(1);
  }
  if (!EnableAdapter()) {
    ExitHandler(1);
  }
  std::string adapterName = std::string("VICTOR_") + GetDeviceSerialNumber();
  if (!SetAdapterName(adapterName)) {
    ExitHandler(1);
  }
  if (!RegisterGattClient()) {
    ExitHandler(1);
  }
  if (!RegisterGattServer()) {
    ExitHandler(1);
  }
  if (!StartBLEPeripheral()) {
    ExitHandler(1);
  }
  WaitForever();
  ExitHandler();
  return 0;
}
