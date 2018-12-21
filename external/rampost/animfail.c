#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "rampost.h"
#include "lcd.h"
#include "error_565.h"

/******** TIMING UTILITIES ********/

#define NSEC_PER_SEC  ((uint64_t)1000000000)
#define NSEC_PER_MSEC ((uint64_t)1000000)
#define NSEC_PER_USEC (1000)


void microwait(long microsec)
{
  struct timespec time;
  uint64_t nsec = microsec * NSEC_PER_MSEC;
  time.tv_sec =  nsec / NSEC_PER_SEC;
  time.tv_nsec = nsec % NSEC_PER_SEC;
  nanosleep(&time, NULL);
}


/*********** ANIMFAIL IMPLEMENTATION **************/

void exit_cleanup(void)
{
  lcd_gpio_teardown();
}

int error_exit(RampostErr err)
{
  exit_cleanup();
  exit(err);
}


int main(int argc, const char* argv[])
{
  lcd_set_brightness(5);

  lcd_gpio_setup();
  lcd_spi_init();

  lcd_device_reset();
  lcd_device_init();

  lcd_draw_frame2((uint16_t*)error_565, error_565_len);

  exit_cleanup();

  return 0;
}
