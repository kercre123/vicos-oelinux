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
  uint64_t nsec = microsec * NSEC_PER_MSEC;
  time.tv_sec =  nsec / NSEC_PER_SEC;
  time.tv_nsec = nsec % NSEC_PER_SEC;
  nanosleep(&time, NULL);
}


#define SPINE_TTY "/dev/ttyHS0"
#define SPINE_BAUD B3000000

/*********** RAMPOST IMPLEMENTATION **************/

struct HeadToBody gHeadData = {0};

enum {LED_BACKPACK_FRONT, LED_BACKPACK_MIDDLE, LED_BACKPACK_BACK};

void set_body_leds(int success, int inRecovery) {
  gHeadData.framecounter++;
  if (!success) {
    gHeadData.ledColors[LED_BACKPACK_FRONT * LED_CHANEL_CT + LED0_RED] = 0xFF;
  }
  else {   //2 Blues for recovery
    gHeadData.ledColors[LED_BACKPACK_FRONT * LED_CHANEL_CT + LED0_BLUE] = 0xFF;
    gHeadData.ledColors[LED_BACKPACK_MIDDLE * LED_CHANEL_CT + LED0_BLUE] = 0xFF;
    if (!inRecovery) { //make them white for normal operation
      gHeadData.ledColors[LED_BACKPACK_FRONT * LED_CHANEL_CT + LED0_RED] = 0xFF;
      gHeadData.ledColors[LED_BACKPACK_FRONT * LED_CHANEL_CT + LED0_GREEN] = 0xFF;
      gHeadData.ledColors[LED_BACKPACK_MIDDLE * LED_CHANEL_CT + LED0_RED] = 0xFF;
      gHeadData.ledColors[LED_BACKPACK_MIDDLE * LED_CHANEL_CT + LED0_GREEN] = 0xFF;
    }
  }

  int errCode = hal_init(SPINE_TTY, SPINE_BAUD);
  if (errCode) { error_exit(errCode); }

  hal_set_mode(RobotMode_RUN);

  //kick off the body frames
  hal_send_frame(PAYLOAD_DATA_FRAME, &gHeadData, sizeof(gHeadData));

  usleep(5000);
  hal_send_frame(PAYLOAD_DATA_FRAME, &gHeadData, sizeof(gHeadData));
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

void exit_cleanup(void)
{
  lcd_device_sleep();
  lcd_gpio_teardown();
}

int error_exit(RampostErr err) {
  set_body_leds(0, 0);
  exit_cleanup();
  exit(err);

}


/*********** ORANGE MODE UTILS **************/

#if (ORANGE == 1 )
#include "warning_orange.h"
#define REACTION_TIME  (0.5 * NSEC_PER_SEC)
#define FRAME_WAIT_MS 200
#define SHUTDOWN_FRAME_INTERVAL (0.5 * NSEC_PER_SEC)

#define IMU_ACC_RANGE 2.0    // g       [2.11.12: acc_range 2 => +- 2g ]
#define MAX_16BIT_POSITIVE 0x7FFF
#define IMU_ACCEL_SCALE_G ((double)(IMU_ACC_RANGE)/MAX_16BIT_POSITIVE)
#define MIN_INVERTED_G  ((long)(0.5 / IMU_ACCEL_SCALE_G))

void show_orange_icon(void) {
  lcd_draw_frame2(warning_orange, warning_orange_len);
}

bool imu_is_inverted(void) {
  IMURawData rawData[IMU_MAX_SAMPLES_PER_READ];
  const int imu_read_samples = imu_manage(rawData);
  if (rawData[0].acc[2] > MIN_INVERTED_G) {
    return true;
  }
  return false;
}

bool button_was_released(struct BodyToHead* bodyData) {
  static bool buttonPressed = false;
  if (bodyData->touchLevel[1] > 0) {
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
  for (now = steady_clock_now(); now-start > REACTION_TIME; now=steady_clock_now()) {
    struct SpineMessageHeader* hdr = hal_get_frame(PAYLOAD_DATA_FRAME, FRAME_WAIT_MS);
    struct BodyToHead* bodyData = (struct BodyToHead*)(hdr+1);
    if ( imu_is_inverted() ) {
      if (button_was_released(bodyData)) {
        buttonCount++;
        if (buttonCount >= 3 ) {
          return unlock_SUCCESS;
        }
      }
    }
  }
  return unlock_TIMEOUT;
}

void send_shutdown_message(void) {
  static uint64_t lastMsgTime = steady_clock_now();
  uint64_t now = steady_clock_now();

  if (now - lastMsgTime > SHUTDOWN_FRAME_INTERVAL) {
    hal_send_frame(PAYLOAD_DATA_FRAME, NULL, 0);
    lastMsgTime = now;
  }
}

#endif


/************ MAIN *******************/
int main(int argc, const char* argv[]) {
  bool success = false;

  lcd_gpio_setup();
  lcd_spi_init();

  lcd_device_reset();
  success = lcd_device_read_status();
  printf("lcd check = %d\n",success);
  set_body_leds(success, recovery_mode_check());

#if (ORANGE == 1)


  show_orange_icon();
  imu_init();
  if (wait_for_unlock() != unlock_SUCCESS) {
    while (1) {
      send_shutdown_message();
    }
  }

#endif

  exit_cleanup();

  return 0;
}
