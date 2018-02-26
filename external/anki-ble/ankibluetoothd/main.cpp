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

#include "include_ev.h"
#include "btstack.h"
#include "device_info.h"
#include "log.h"
#include "agent.h"

#include <unistd.h>

static struct ev_loop* sDefaultLoop = ev_default_loop(EVBACKEND_SELECT);

static void ExitHandler(int status = 0) {
  Anki::BluetoothStack::UnLoadBtStack();
  _exit(status);
}

static void SignalCallback(struct ev_loop* loop, struct ev_signal* w, int revents)
{
  logi("Exiting for signal %d", w->signum);
  ev_unloop(loop, EVUNLOOP_ALL);
  ExitHandler();
}

static struct ev_signal sIntSig;
static struct ev_signal sTermSig;

int main(int argc, char *argv[]) {
  ev_signal_init(&sIntSig, SignalCallback, SIGINT);
  ev_signal_start(sDefaultLoop, &sIntSig);
  ev_signal_init(&sTermSig, SignalCallback, SIGTERM);
  ev_signal_start(sDefaultLoop, &sTermSig);

  setAndroidLoggingTag("ankibtd");
  setMinLogLevel(kLogLevelVerbose);
  if (!Anki::BluetoothStack::LoadBtStack()) {
    ExitHandler(1);
  }
  if (!Anki::BluetoothStack::EnableAdapter()) {
    ExitHandler(1);
  }
  std::string adapterName = std::string("VICTOR_") + GetDeviceSerialNumber();
  if (!Anki::BluetoothStack::SetAdapterName(adapterName)) {
    ExitHandler(1);
  }
  if (!Anki::BluetoothStack::RegisterGattClient()) {
    ExitHandler(1);
  }
  if (!Anki::BluetoothStack::RegisterGattServer()) {
    ExitHandler(1);
  }
  if (!StartBLEAgent()) {
    ExitHandler(1);
  }
  ev_loop(sDefaultLoop, 0);
  ExitHandler();
  return 0;
}
