/**
 * File: viccubetool's main.cpp
 *
 * Author: seichert
 * Created: 2/15/2018
 *
 * Description: Talk to Victor's cubes over BLE
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "viccubetool.h"

#include "include_ev.h"
#include "log.h"

#include <unistd.h>

#include <cctype>
#include <iostream>

static struct ev_loop* sDefaultLoop = ev_default_loop(EVBACKEND_SELECT);
static VicCubeTool* sVicCubeTool = nullptr;

static void ExitHandler(int status = 0) {
  delete sVicCubeTool; sVicCubeTool = nullptr;
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

void usage() {
  std::cout << "viccubetool [ -a address ] command args" << std::endl
            << "-a address Bluetooth MAC address of cube. default is first found" << std::endl
            << "-r<rssi>   Minimum RSSI for connection. default is -40" << std::endl
            << "-h         Help (this message)" << std::endl
            << std::endl
            << "commands" << std::endl
            << "--------" << std::endl
            << "scan                       - Search for advertising cubes" << std::endl
            << "connect [address]          - Connect to a cube over BLE" << std::endl
            << "flash path/to/firmware     - Flash the cube with new firmware" << std::endl
            << "flashdvt1 path/to/firmware - Flash the DVT1/2 cube with new firmware" << std::endl
            << "cube-test path/to/firmware - Flash evey cube within minimum RSSI with new firmware" << std::endl;
}

long str_to_int(const char*str, long default_val) {
  char* endp;
  long l = strtol(str, &endp, 10);
  if (endp==str) {return default_val;}
  return l;
}


int main(int argc, char *argv[]) {
  std::string address;
  std::vector<std::string> args;
  int rssi_min = 0;

  int c;
  opterr = 0;

  while ((c = getopt(argc, argv, "a:hr:")) != -1) {
    switch(c) {
      case 'a':
        address = std::string(optarg);
        break;
      case 'h':
        usage();
        return 0;
        break;
      case 'r':
        rssi_min = str_to_int(optarg,-40);
          std::cerr << "Minimum rssi is "<< rssi_min << std::endl;
        break;
      case '?':
        if (optopt == 'a') {
          std::cerr << "Option -a requires an argument" << std::endl;
        } else if (std::isprint(optopt)) {
          std::cerr << "Unknown option: " << (char) optopt << std::endl;
        } else {
          std::cerr << "Unknown option character\n" << optopt << std::endl;
        }
        /* fall through */
      default:
        usage();
        return 1;
    }
  }

  int index = optind;
  while (index < argc) {
    args.push_back(std::string(argv[index++]));
  }

  ev_signal_init(&sIntSig, SignalCallback, SIGINT);
  ev_signal_start(sDefaultLoop, &sIntSig);
  ev_signal_init(&sTermSig, SignalCallback, SIGTERM);
  ev_signal_start(sDefaultLoop, &sTermSig);

  setAndroidLoggingTag("vctool");
  setMinLogLevel(kLogLevelVerbose);
  sVicCubeTool = new VicCubeTool(sDefaultLoop, address, args);
  if (rssi_min) { sVicCubeTool->SetMinRssi(rssi_min);}
  sVicCubeTool->Execute();

  ev_loop(sDefaultLoop, 0);
  ExitHandler();
  return 0;
}
