#include <stdio.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <assert.h>

#include "messages.h"
#include "spine_hal.h"
#include "rampost.h"
#include "lcd.h"
#include "dfu.h"
#include "das.h"

#define REACTION_TIME  ((uint64_t)(30 * NSEC_PER_SEC))
#define LOW_BATTERY_TIME ((uint64_t)(15 * NSEC_PER_SEC)) // Per VIC-4663
#define DFU_TIMEOUT ((uint64_t)(30 * NSEC_PER_SEC)) // Per VIC-4663
#define FRAME_WAIT_MS 500
#define SHUTDOWN_FRAME_INTERVAL ((uint64_t)(0.5 * NSEC_PER_SEC))

enum {
  app_DEVICE_OPEN_ERROR = 1,
  app_IO_ERROR,
  app_VALIDATION_ERROR,
  app_MEMORY_ERROR,

};

/******** TIMING UTILITIES ********/

#define NSEC_PER_SEC  ((uint64_t)1000000000)
#define NSEC_PER_MSEC ((uint64_t)1000000)
#define NSEC_PER_USEC (1000)


uint64_t steady_clock_now(void) {
   struct timespec time;
   clock_gettime(CLOCK_MONOTONIC,&time);
   return time.tv_nsec + time.tv_sec * NSEC_PER_SEC;
}

void microwait(long microsec)
{
  struct timespec time;
  uint64_t nsec = microsec * NSEC_PER_USEC;
  time.tv_sec =  nsec / NSEC_PER_SEC;
  time.tv_nsec = nsec % NSEC_PER_SEC;
  nanosleep(&time, NULL);
}


#define SPINE_TTY "/dev/ttyHS0"
#define SPINE_BAUD B3000000

/*********** RAMPOST IMPLEMENTATION **************/

enum {LED_BACKPACK_FRONT, LED_BACKPACK_MIDDLE, LED_BACKPACK_BACK};

void set_body_leds(int success, int inRecovery) {
  struct LightState ledPayload = {0};

  if (!success) {
    ledPayload.ledColors[LED_BACKPACK_FRONT * LED_CHANEL_CT + LED0_RED] = 0xFF;
  }
  else {   //2 Blues for recovery
    ledPayload.ledColors[LED_BACKPACK_FRONT * LED_CHANEL_CT + LED0_BLUE] = 0xFF;
    ledPayload.ledColors[LED_BACKPACK_MIDDLE * LED_CHANEL_CT + LED0_BLUE] = 0xFF;
    if (!inRecovery) { //make them white for normal operation
      ledPayload.ledColors[LED_BACKPACK_FRONT * LED_CHANEL_CT + LED0_RED] = 0xFF;
      ledPayload.ledColors[LED_BACKPACK_FRONT * LED_CHANEL_CT + LED0_GREEN] = 0xFF;
      ledPayload.ledColors[LED_BACKPACK_MIDDLE * LED_CHANEL_CT + LED0_RED] = 0xFF;
      ledPayload.ledColors[LED_BACKPACK_MIDDLE * LED_CHANEL_CT + LED0_GREEN] = 0xFF;
    }
  }

  hal_send_frame(PAYLOAD_LIGHT_STATE, &ledPayload, sizeof(ledPayload));
  hal_get_next_frame(FRAME_WAIT_MS); //need response, don't worry about what it says


}

#define CMDLINE_FILE "/proc/cmdline"
#define MAX_COMMANDLINE_CHARS 512
#define RECOVERY_MODE_INDICATOR "anki.unbrick"

int recovery_mode_check(void) {
  char buffer[MAX_COMMANDLINE_CHARS];
  int fd = open(CMDLINE_FILE, O_RDONLY);
  int result = read(fd, buffer, sizeof(buffer)-1);
  if  (result > 0) {
    buffer[result] = '\0'; //null terminate
    DAS_LOG(DAS_DEBUG, "recovery_mode_check.scan", "scanning [%s] for [%s]", buffer, RECOVERY_MODE_INDICATOR);
    char* pos = strstr(buffer, RECOVERY_MODE_INDICATOR);
    DAS_LOG(DAS_DEBUG, "recovery_mode_check.result", "%s", pos?"found":"nope");
    return (pos!=NULL);
  }
  return 0; //fault mode
}

void cleanup(bool blank_display)
{
  if( blank_display) {
    lcd_set_brightness(0);
    lcd_device_sleep();
  }
}

int error_exit(RampostErr err) {
  cleanup(true);
  exit(err);
}

#include "anki_dev_unit.h"
#include "lowbattery.h"
#include "error_801.h"


void show_dev_unit(void) {
  lcd_draw_frame2((uint16_t*)anki_dev_unit, anki_dev_unit_len);
}

void show_low_battery(void) {
  lcd_draw_frame2((uint16_t*)low_battery, low_battery_len);
}

void show_error_801(void) {
  lcd_draw_frame2((uint16_t*)error_801, error_801_len);
}


typedef enum {
  battery_LEVEL_GOOD,
  battery_LEVEL_TOOLOW,
  battery_BOOTLOADER,
  battery_TIMEOUT
} BatteryState;


BatteryState confirm_battery_level(void) {
  int i;
  for (i=0; i<5; i++) {
    const struct SpineMessageHeader* hdr = hal_get_next_frame(FRAME_WAIT_MS);
    if (hdr == NULL) continue;
    else if (hdr->payload_type == PAYLOAD_BOOT_FRAME) return battery_BOOTLOADER;
    else if (hdr->payload_type == PAYLOAD_DATA_FRAME) {
      static const float kBatteryScale = 2.8f / 2048.f;
      const int16_t counts = ((struct BodyToHead*)(hdr+1))->battery.main_voltage;
      const float volts = counts*kBatteryScale;
      DAS_LOG(DAS_EVENT, "battery_level", "%0.3f", volts);
      if (volts > 3.55f) return battery_LEVEL_GOOD;
      else return battery_LEVEL_TOOLOW;
    }
  }
  return battery_TIMEOUT;
}


void send_shutdown_message(void) {
  static uint64_t lastMsgTime = 0;
  uint64_t now = steady_clock_now();

  if (now - lastMsgTime > SHUTDOWN_FRAME_INTERVAL) {
    DAS_LOG(DAS_INFO, "send_shutdown_message", "PAYLOAD_SHUT_DOWN");
    hal_send_frame(PAYLOAD_SHUT_DOWN, NULL, 0);
    lastMsgTime = now;
    hal_get_next_frame(FRAME_WAIT_MS);
  }
}


void show_lowbat_and_shutdown(void) {
  uint64_t start = steady_clock_now();
  uint64_t now;
  show_low_battery();
  for (now = steady_clock_now(); now-start < LOW_BATTERY_TIME; now=steady_clock_now()) continue;
  while (true) send_shutdown_message();
}


void force_syscon_resync(void) {
  uint8_t ALL_EFFS[256];
  memset(ALL_EFFS, 0xFF, sizeof(ALL_EFFS));
  int bytes_to_send = 2048;
  while (bytes_to_send) {
    hal_serial_send(ALL_EFFS, sizeof(ALL_EFFS));
    bytes_to_send-= sizeof(ALL_EFFS);
  }
}



/************ MAIN *******************/
int main(int argc, const char* argv[]) {
  bool success = false;
  bool in_recovery_mode = false;
  bool blank_on_exit = true;
  bool show_801 = false;
  bool hal_failure = false;
  bool is_dev_unit = false;
  bool is_battery_low = false;
  bool force_update = false;
  const char* dfu_file = NULL;

  // Handle arguments:
  int argn = 1;
  for (; argn < argc; argn++) {
    if (argv[argn][0] == '-') {
      switch (argv[argn][1]) {
        case 'd':
        {
          is_dev_unit = true;
          break;
        }
        case 'f':
        {
          force_update = true;
          break;
        }
      }
    }
    else {   //does not start with dash, must be dfu file
      dfu_file = argv[argn];
    }
  }

  lcd_init();

  lcd_device_reset();
  success = lcd_device_read_status();
  DAS_LOG(DAS_INFO, "lcd_check", "%d",success);

  in_recovery_mode = recovery_mode_check();

  int errCode = hal_init(SPINE_TTY, SPINE_BAUD);
  if (errCode) {
    hal_failure = true;
  }
  else {
    force_syscon_resync();
    //clear buffer
    const struct SpineMessageHeader* hdr;
    do {
      hdr= hal_get_next_frame(1); //clear backlog
    } while (hdr);

    if (dfu_file != NULL) { // A DFU file has been specified
      RampostErr result = dfu_sequence(dfu_file, DFU_TIMEOUT, force_update);
      if (result == err_SYSCON_VERSION_GOOD) {
        //hooray!
      }
      else if (result != err_OK ) {
        DAS_LOG(DAS_ERROR, "dfu_error", "DFU Error %d", result);
        show_801 = true;
      }
    }
  }
  if (hal_failure) {
    show_801 = true;
  }
  else {
    BatteryState bat_state = confirm_battery_level();
    switch (bat_state) {
      case battery_LEVEL_GOOD:
        break;
      case battery_LEVEL_TOOLOW:
        is_battery_low = true;
        break;
      case battery_BOOTLOADER:
        DAS_LOG(DAS_EVENT, "battery_check_fail", "Battery check saw bootloader!");
        show_801 = true; //should be impossible.
        break;
      case battery_TIMEOUT:
        DAS_LOG(DAS_EVENT, "battery_check_fail", "Battery check timed out!");
        break; // But do continue boot in case this is because something needs recovering etc.
    }

    //set LEDS only after we expect syscon is good
    set_body_leds(success, in_recovery_mode);

  }

  lcd_device_init(); // We'll be displaying something
  lcd_set_brightness(5);

  //Skip everything else on syscon error!
  if (show_801) {
    show_error_801();
    blank_on_exit = false;
  }
  else if (is_battery_low) {
    show_lowbat_and_shutdown();
  }
  else if (is_dev_unit) {
    show_dev_unit();
    blank_on_exit = false; // Exit without blanking the display
  }

  cleanup(blank_on_exit);
  bool has_error = show_801 || is_battery_low;
  return has_error;
}
