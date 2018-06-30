#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <linux/fb.h>

#include "gpio.h"
#include "rampost.h"
#include "lcd.h"

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))

/************** LCD INTERFACE *****************/

static const char* BACKLIGHT_DEVICES[] = {
  "/sys/class/leds/face-backlight/brightness",
  "/sys/class/leds/face-backlight-left/brightness",
  "/sys/class/leds/face-backlight-right/brightness"
};

static const char* fb0_blank = "/sys/class/graphics/fb0/blank";
static int fb_fd;

int lcd_init()
{
  fb_fd = open("/dev/fb0", O_RDWR);
  if (!fb_fd)  {
    error_exit(err_SPI_INIT);
  }
  return fb_fd;
}

static void _led_set_brightness(const int brightness, const char* led)
{
  int fd = open(led,O_WRONLY);
  if (fd) {
    char buf[3];
    snprintf(buf,3,"%02d\n",brightness);
    write(fd, buf, 3);
    close(fd);
  }
}

void lcd_set_brightness(int brightness)
{
  int l;
  brightness = MIN(brightness, 10);
  brightness = MAX(brightness, 0);
  for (l=0; l<3; ++l) {
    _led_set_brightness(brightness, BACKLIGHT_DEVICES[l]);
  }
}


void lcd_draw_frame2(const uint16_t* frame, size_t size) {
  lseek(fb_fd, 0, SEEK_SET);
  write(fb_fd, frame, size);
}

void lcd_device_sleep(void)
{
  int fd = open(fb0_blank, O_WRONLY);
  if (fd) {
    char buf[] = "4\n";
    write(fd, buf, sizeof(buf));
    close(fd);
  }
}

int lcd_device_reset(void)
{
  char buf[] = "0\n";
  int fd = open(fb0_blank, O_WRONLY);

  if (!fd)
    return -1;

  write(fd, buf, sizeof(buf));
  close(fd);
  return 0;
}

int lcd_device_read_status(void)
{
  struct fb_fix_screeninfo fixed_info;

  if (!fb_fd)
    return -1;

  if (ioctl(fb_fd,FBIOGET_FSCREENINFO, &fixed_info) < 0)
    return -1;

  return 0;
}
