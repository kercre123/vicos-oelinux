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
#include <stdio.h>
#include <sys/ioctl.h>
#include <assert.h>

#include "messages.h"
#include "spine_hal.h"
#include "rampost.h"
#include "lcd.h"
#include "dfu.h"
#include "das.h"

#define FRAME_WAIT_MS 500

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


//returns monotonic time in ns.
uint64_t steady_clock_now(void)
{
  struct timespec time;
  clock_gettime(CLOCK_MONOTONIC, &time);
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

void set_body_leds(int success, int inRecovery)
{
  struct LightState ledPayload = {{0}};

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
#define SEMAPHORE_FILE "/dev/rampost_error"
#define MAX_COMMANDLINE_CHARS 512
#define RECOVERY_MODE_INDICATOR "anki.unbrick"

int recovery_mode_check(void)
{
  char buffer[MAX_COMMANDLINE_CHARS];
  int fd = open(CMDLINE_FILE, O_RDONLY);
  int result = read(fd, buffer, sizeof(buffer) - 1);
  if (result > 0) {
    buffer[result] = '\0'; //null terminate
    DAS_LOG(DAS_DEBUG, "recovery_mode_check.scan", "scanning [%s] for [%s]", buffer,
            RECOVERY_MODE_INDICATOR);
    char* pos = strstr(buffer, RECOVERY_MODE_INDICATOR);
    DAS_LOG(DAS_DEBUG, "recovery_mode_check.result", "%s", pos ? "found" : "nope");
    return (pos != NULL);
  }
  return 0; //fault mode
}

void cleanup(bool blank_display)
{
  if (blank_display) {
    lcd_set_brightness(0);
    lcd_device_sleep();
  }
  lcd_gpio_teardown();
}

#include "anki_dev_unit.h"
#include "low_battery.h"
#include "too_hot.h"
#include "error_801.h"
#include "error_802.h"
#define MAX_ERROR_STR_LEN (16)
#define ERROR_HAL_ERROR (802)
#define ERROR_DFU_ERROR (801)

static inline void show_dev_unit(void)
{
  lcd_draw_frame2((uint16_t*)anki_dev_unit, anki_dev_unit_len);
}

static inline void show_low_battery(void)
{
  lcd_draw_frame2((uint16_t*)low_battery, low_battery_len);
}

static inline void show_too_hot(void)
{
  lcd_draw_frame2((uint16_t*)too_hot, too_hot_len);
}

static void write_semaphore(const char* const text, const int length)
{
  int fd = open(SEMAPHORE_FILE, O_CREAT | O_WRONLY, 00640);
  if (fd >= 0) {
    write(fd, text, length);
    close(fd);
  }
  else {
    DAS_LOG(DAS_ERROR, "semaphore",
            "Couldn't write semaphore file \"%s\": %d for error \"%s\"", SEMAPHORE_FILE, fd, text);
  }
}

static void show_error(const int error_code)
{
  char error_str[MAX_ERROR_STR_LEN];
  const int wlen = snprintf(error_str, MAX_ERROR_STR_LEN, "ERROR %d\n", error_code);
  write_semaphore(error_str, wlen);

  switch (error_code) {
  case 801:
    lcd_draw_frame2((uint16_t*)error_801, error_801_len);
    break;
  case 802:
    lcd_draw_frame2((uint16_t*)error_802, error_802_len);
    break;
  default:
    DAS_LOG(DAS_ERROR, "show_error", "No image for error %d", error_code);
  }
}

int error_exit(RampostErr err)
{
  show_error(ERROR_HAL_ERROR);
  cleanup(false);
  exit(err);
}

typedef enum {
  battery_LEVEL_GOOD,
  battery_LEVEL_TOOLOW,
  battery_TOO_HOT,
  battery_BOOTLOADER,
  battery_TIMEOUT
} BatteryState;


BatteryState confirm_battery_level(void)
{
  int i;
  for (i = 0; i < 5; i++) {
    const struct SpineMessageHeader* hdr = hal_get_next_frame(FRAME_WAIT_MS);
    if (hdr == NULL) { continue; }
    else if (hdr->payload_type == PAYLOAD_BOOT_FRAME) { return battery_BOOTLOADER; }
    else if (hdr->payload_type == PAYLOAD_DATA_FRAME) {
      static const float kBatteryScale = 2.8f / 2048.f;
      struct BodyToHead* const b2h = (struct BodyToHead*)(hdr + 1);
      const int16_t counts = b2h->battery.main_voltage;
      const float volts = counts * kBatteryScale;

      DAS_LOG(DAS_EVENT, "battery_level", "%0.3f", volts);
      DAS_LOG(DAS_EVENT, "battery_temperature", "%d", b2h->battery.temperature);
      DAS_LOG(DAS_EVENT, "battery_flags", "%08x", b2h->battery.flags);

      if (b2h->battery.flags & (POWER_IS_OVERHEATED)) { return battery_TOO_HOT; }
      if (b2h->battery.flags & POWER_ON_CHARGER) { return battery_LEVEL_GOOD; }
      if (b2h->battery.flags & (POWER_IS_TOO_LOW | POWER_BATTERY_SHUTDOWN)) { return battery_LEVEL_TOOLOW; }
      return battery_LEVEL_GOOD;
    }
  }
  return battery_TIMEOUT;
}


void force_syscon_resync(void)
{
  uint8_t ALL_EFFS[256];
  memset(ALL_EFFS, 0xFF, sizeof(ALL_EFFS));
  int bytes_to_send = 2048;
  while (bytes_to_send) {
    hal_serial_send(ALL_EFFS, sizeof(ALL_EFFS));
    bytes_to_send -= sizeof(ALL_EFFS);
  }
}


/************ MAIN *******************/
int main(int argc, const char* argv[])
{
  static const char* const LOW_BAT = "low bat";
  static const char* const TOO_HOT = "too hot";
  int error_code = 0;
  SpineErr spine_error = 0;
  bool success = false;
  bool in_recovery_mode = false;
  bool is_low_battery = false;
  bool is_too_hot = false;
  bool is_dev_unit = false;
  bool force_update = false;
  const char* dfu_file = NULL;

  // Handle arguments:
  int argn = 1;
  for (; argn < argc; argn++) {
    if (argv[argn][0] == '-') {
      switch (argv[argn][1]) {
      case 'd': {
        is_dev_unit = true;
        break;
      }
      case 'f': {
        force_update = true;
        break;
      }
      }
    }
    else {   //does not start with dash, must be dfu file
      dfu_file = argv[argn];
    }
  }

  lcd_gpio_setup();
  lcd_spi_init();

  lcd_device_reset();
  success = lcd_device_read_status();
  DAS_LOG(DAS_INFO, "lcd_check", "%d", success);

  in_recovery_mode = recovery_mode_check();

  spine_error = hal_init(SPINE_TTY, SPINE_BAUD);
  if (spine_error) {
    error_code = ERROR_HAL_ERROR;
  }
  else {
    force_syscon_resync();
    //clear buffer
    uint64_t timeout = steady_clock_now() + 1 * NSEC_PER_SEC;
    const struct SpineMessageHeader* hdr;
    do {
      hdr = hal_get_next_frame(1); //clear backlog already in UART RX FIFO
    }
    while (hdr && steady_clock_now() < timeout);
    // If we haven't seen anything yet, try to make sure we're receiving something
    if (!hdr) { hdr = hal_get_next_frame(10); }
    if (hdr) { DAS_LOG(DAS_EVENT, "before_dfu.got_frame", "%c%c", hdr->payload_type & 0xff, hdr->payload_type >> 8); }

    if (dfu_file != NULL) { // A DFU file has been specified
      RampostErr result = dfu_sequence(dfu_file, force_update);
      if (result == err_SYSCON_VERSION_GOOD) {
        //hooray!
      }
      else if (result > err_OK) {
        DAS_LOG(DAS_ERROR, "dfu_error", "DFU Error %d", result);
        if (result < err_DFU_ERASE_ERROR) { error_code = ERROR_HAL_ERROR; }
        else { error_code = ERROR_DFU_ERROR; }
      }
    }
  }

  if (!error_code) {
    BatteryState bat_state = confirm_battery_level();
    switch (bat_state) {
    case battery_LEVEL_GOOD:
      break;
    case battery_LEVEL_TOOLOW:
      DAS_LOG(DAS_EVENT, "battery_level_low", "%d", bat_state);
      is_low_battery = true;
      break;
    case battery_TOO_HOT:
      DAS_LOG(DAS_EVENT, "battery_too_hot", "%d", bat_state);
      is_too_hot = true;
      break;
    case battery_BOOTLOADER:
      DAS_LOG(DAS_EVENT, "battery_check_fail", "Battery check saw bootloader!");
      error_code = ERROR_DFU_ERROR; //should be impossible.
      break;
    case battery_TIMEOUT:
      DAS_LOG(DAS_EVENT, "battery_check_fail", "Battery check timed out!");
      break; // But do continue boot in case this is because something needs recovering etc.
    }

    //set LEDS only after we expect syscon is good
    set_body_leds(success, in_recovery_mode);
  }

  // Always init, even if we don't show image, to clear the screen.
  // If not we see some random garbage in the boot process on PVT/MP
  // machines.
  lcd_device_init(); 
  
  if (error_code || is_dev_unit || is_low_battery || is_too_hot) {
    lcd_set_brightness(5);
    //Skip everything else on syscon error!
    if (error_code) {
      show_error(error_code);
    }
    else if (is_low_battery) {
      show_low_battery();
      write_semaphore(LOW_BAT, strlen(LOW_BAT));
    }
    else if (is_too_hot) {
      show_too_hot();
      write_semaphore(TOO_HOT, strlen(TOO_HOT));
    }
    else if (is_dev_unit) {
      show_dev_unit();
    }
    cleanup(false);
  }
  else {
    cleanup(true);
  }

  hal_exit();

  DAS_LOG(DAS_EVENT, "rampost.exit", "%d", error_code);
  return error_code;
}
