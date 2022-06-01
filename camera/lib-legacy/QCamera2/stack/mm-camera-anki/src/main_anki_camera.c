/**
 * File: main_anki_camera.c
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: Camera Server Daemon main executable. Manages the lifecycle of a camera_server
 *              instance.
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <assert.h>
#include <stdint.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>

#include "camera_server.h"
#include "log.h"

static struct server_ctx s_camera_server;

static void handle_signal(int signum)
{
  // stop server to trigger graceful shutdown sequence
  stop_server(&s_camera_server);
}

void usage(char* name)
{
  printf("usage: %s\n", name);
  printf("   --output,-o <DIR>                   output directory for image dumps\n");
  printf("   --capture,-C                        auto-start camera capture on startup\n");
  printf("   --slave_mode,-S                     after entering a connected state, exit on disconnect\n");
  printf("   --fps-reduction,-r <SCALE>          reduce frame rate to 30/<SCALE> fps (default: 2)\n");
  printf("   --verbose,-v <LOG LEVEL>            enable verbose output: 0-5 [silent,errors(default),warnings,info,debug,verbose]\n");
  printf("   --dump,-d                           dump processed image received from camera system\n");
  printf("   --legacy,-l                         use legacy 1mp camera\n");
  printf("   --format,-f <FORMAT>                set initial capture format (RGB or YUV, default: RGB)\n");
  printf("   --help,-h                           this helpful message\n");
}

static void setup_signal_handler()
{
  struct sigaction new_action, old_action;

  new_action.sa_handler = handle_signal;
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

static uint32_t vector_hw_version()
{
  int fd = -1;
  uint32_t emr_data[8]; // The emr header

  fd = open("/dev/block/bootdevice/by-name/emr",O_RDONLY);

  if (fd == -1) { return 0;} // ABORT!
  
  read(fd, &emr_data, sizeof(emr_data));

  // See emr-cat.c to determine how we got the offset
  uint32_t hw_ver = emr_data[1];

  close(fd);
  
  return hw_ver;
}

int main(int argc, char* argv[])
{
  setup_signal_handler();

  char* outdir = NULL;
  uint8_t autostart_camera = 0;
  uint8_t slave_mode = 0;
  uint8_t debug_dump_images = 0;
  uint8_t one_megapixel = 0;
  uint8_t fps_reduction = 2;
  uint8_t verbose_level = 1;
  char* format = NULL;
  int c;
  int optidx = 0;

  struct option longopt[] = {
    {"output",              1, NULL, 'o'},
    {"capture",             0, NULL, 'C'},
    {"slave_mode",          0, NULL, 'S'},
    {"fpsreduction",        1, NULL, 'r'},
    {"verbose",             1, NULL, 'v'},
    {"dump",                0, NULL, 'd'},
    {"format",              1, NULL, 'f'},
    {"help",                0, NULL, 'h'},
    {0, 0, 0, 0}
  };

  while ((c = getopt_long(argc, argv, "Cdho:r:Sv:", longopt, &optidx)) != -1) {
    switch (c) {
    case 'C':
      autostart_camera = 1;
      break;
    case 'd':
      debug_dump_images = 1;
      break;
    case 'h':
      usage(argv[0]);
      return 0;
      break;
    case 'o':
      outdir = strdup(optarg);
      break;
    case 'r':
      fps_reduction = atoi(optarg);
      break;
    case 'S':
      slave_mode = 1;
      break;
    case 'v':
      verbose_level = atoi(optarg);
      break;
    case 'f':
      format = strdup(optarg);
      break;
    default:
      printf("bad arg\n");
      usage(argv[0]);
      return 1;
    }
  }

  printf("VECTOR HW VER %d\n", vector_hw_version());
  if(vector_hw_version() < 0x20) {
    one_megapixel = 1;
    printf("%s\n", "LEGACY one megapixel mode!");
  } else {
    one_megapixel = 0;
    printf("%s\n", "DEFAULT two megapixel mode!");
  }
  
  // configure logging
  int log_level = AnkiCameraLogLevelMax - verbose_level;
  setMinLogLevel(log_level);

  int rc = 0;

  printf("START: Anki Camera Server\n");

  // start server
  // Pass in pre-connected fd if we have one
  rc = create_server(&s_camera_server, -1);

  // pass in server params
  s_camera_server.params.autostart_camera = autostart_camera;
  s_camera_server.params.exit_on_disconnect = slave_mode;
  s_camera_server.params.debug_dump_images = debug_dump_images;
  s_camera_server.capture_params.fps_reduction = (fps_reduction) >= 1 ? fps_reduction : 1;
  if (one_megapixel) {
    s_camera_server.capture_params.pixel_format = ANKI_CAM_FORMAT_RAW;
  } else {
    s_camera_server.capture_params.pixel_format = ANKI_CAM_FORMAT_RAW_2MP;
  }

  if(format != NULL)
  {
    if(strcmp(format, "YUV") == 0)
    {
      s_camera_server.capture_params.pixel_format = ANKI_CAM_FORMAT_YUV;
    }
    else if(strcmp(format, "RGB") != 0)
    {
      printf("Unrecognized capture format %s, defaulting to RGB\n", format);   
    }
  }

  if (rc == 0) {
    rc = start_server(&s_camera_server);
  }

  // Server runs until stop, error or timeout
  stop_server(&s_camera_server);
  destroy_server(&s_camera_server);

  printf("EXIT (%d): Anki Camera Server\n", rc);

  return rc;
}
