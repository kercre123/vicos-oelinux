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
#include "imu.h"
#include "dfu.h"


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
    printf("scanning [%s] for [%s]\n", buffer, RECOVERY_MODE_INDICATOR);
    char* pos = strstr(buffer, RECOVERY_MODE_INDICATOR);
    printf("%s\n", pos?"found":"nope");
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
  lcd_gpio_teardown();
}

int error_exit(RampostErr err) {
  set_body_leds(0, 0);
  cleanup(true);
  exit(err);
}



#include "warning_orange.h"
#include "locked_qsn.h"
#include "anki_dev_unit.h"
#include "lowbattery.h"
#include "error_801.h"


void show_orange_icon(void) {
  lcd_draw_frame2((uint16_t*)warning_orange, warning_orange_len);
}

void show_locked_qsn_icon(void) {
  lcd_draw_frame2((uint16_t*)locked_qsn, locked_qsn_len);
}

void show_dev_unit(void) {
  lcd_draw_frame2((uint16_t*)anki_dev_unit, anki_dev_unit_len);
}

void show_low_battery(void) {
  lcd_draw_frame2((uint16_t*)low_battery, low_battery_len);
}

void show_error_801(void) {
  lcd_draw_frame2((uint16_t*)error_801, error_801_len);
}

bool imu_is_inverted(void) {

  static int imu_print_freq =  20;
  IMURawData rawData[IMU_MAX_SAMPLES_PER_READ];
  int imuerr = imu_manage(rawData);
  printf ("imu %d\n",imuerr);
  if (!--imu_print_freq) {
    imu_print_freq = 20;
    printf("acc = <%d, %d, %d> %d\n", rawData->acc[0], rawData->acc[1], rawData->acc[2],
                                      (rawData->acc[2] < -MIN_INVERTED_G));
  }
  if (rawData[0].acc[2] < -MIN_INVERTED_G) {
    return true;
  }
  return false;
}

bool button_was_released(uint16_t buttonState) {
  static bool buttonPressed = false;
  if (buttonState > 0) {
    buttonPressed = true;
  }
  else if (buttonPressed) {
    buttonPressed = false;
    return true;
  }
  return false;
}

typedef enum {
  unlock_SUCCESS,
  unlock_TIMEOUT,
} UnlockState;


UnlockState wait_for_unlock(void) {
  int buttonCount = 0;
  uint64_t start = steady_clock_now();
  uint64_t now;

  printf("Waiting %lld ns for UNLOCK!\n", REACTION_TIME);

  for (now = steady_clock_now(); now-start < REACTION_TIME; now=steady_clock_now()) {
    uint16_t buttonState = 0;
    const struct SpineMessageHeader* hdr = hal_get_next_frame(FRAME_WAIT_MS);
    int inverted = imu_is_inverted();
    if (hdr == NULL) continue;
    if (hdr->payload_type == PAYLOAD_DATA_FRAME) {
      buttonState = ((struct BodyToHead*)(hdr+1))->touchLevel[1];
    }
    else if (hdr->payload_type == PAYLOAD_BOOT_FRAME) {
      //extract button data from stub packet and put in fake full packet
      buttonState = ((struct MicroBodyToHead*)(hdr+1))->buttonPressed;
    }
    printf(":%d ^%d\n", buttonState, inverted);
    if ( inverted ) {
      if (button_was_released(buttonState)) {
        buttonCount++;
printf("Press %d!\n", buttonCount);
        if (buttonCount >= 3 ) {
          return unlock_SUCCESS;
        }
      }
    }
  }
  return unlock_TIMEOUT;
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
      printf("Battery: %f V\n", volts);
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
    printf("Shutting Down\n");
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


/************ MAIN *******************/
int main(int argc, const char* argv[]) {
  bool success = false;
  bool in_recovery_mode = false;
  bool blank_on_exit = true;
  bool skip_dfu = false;

  lcd_gpio_setup();
  lcd_spi_init();

  lcd_device_reset();
  success = lcd_device_read_status();
  printf("lcd check = %d\n",success);

  in_recovery_mode = recovery_mode_check();

  int errCode = hal_init(SPINE_TTY, SPINE_BAUD);
  if (errCode) {
    error_exit(errCode);
  }
  //clear buffer
  const struct SpineMessageHeader* hdr;
  do {
    hdr= hal_get_next_frame(0);
  } while (hdr);


/*  switch (confirm_battery_level()) {
    case battery_LEVEL_GOOD:
      break;
    case battery_LEVEL_TOOLOW:
      lcd_device_init(); // We'll be displaying something
      lcd_set_brightness(5);
      show_lowbat_and_shutdown();
      return 1;
    case battery_BOOTLOADER:
      printf("Battery check saw bootloader!\n");
      break;
    case battery_TIMEOUT:
      printf("Battery timeout\n");
      skip_dfu = true; // Don't try DFU if we can't communicate well enough to get a battery level
      break; // But do continue boot in case this is because something needs recovering etc.
  }
*/

  // Handle arguments:
  int argn = 1;

  if (skip_dfu == false && argc > argn && argv[argn][0] != '-') { // A DFU file has been specified
    if (!dfu_if_needed(argv[argn], DFU_TIMEOUT)) {
      error_801 = true;
    }
    argn++;
  }

  set_body_leds(success, in_recovery_mode);



  for (; argn < argc; argn++) {
    if (argv[argn][0] == '-') {
      lcd_device_init(); // We'll be displaying something
      lcd_set_brightness(5);
      switch (argv[argn][1]) {
        case 'x':
        {
          show_locked_qsn_icon();
          while (1);
          return 1;
        }
        case 'd':
        {
          show_dev_unit();
          blank_on_exit = false; // Exit without blanking the display
          break;
        }
        case 'o':
        {
          show_orange_icon();
          imu_open();
          imu_init();
          if (wait_for_unlock() != unlock_SUCCESS) {
            while (1) {
              send_shutdown_message();
            }
          }
          break;
        }
      }
    }
  }
  if (error_801) {
    show_error_801();
    blank_on_exit = false;
  }

  cleanup(blank_on_exit);

  return 0;
}
