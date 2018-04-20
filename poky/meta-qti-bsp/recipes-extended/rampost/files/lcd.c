#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


#include "gpio.h"
#include "rampost.h"

/************** LCD INTERFACE *****************/

#define GPIO_LCD_WRX   110
#define GPIO_LCD_RESET1 96
#define GPIO_LCD_RESET2 55

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))


static GPIO RESET_PIN1;
static GPIO RESET_PIN2;
static GPIO DnC_PIN;

static const int DAT_CLOCK = 17500000;
static const int MAX_TRANSFER = 0x1000;


static int spi_fd;




static const char* BACKLIGHT_DEVICES[] = {
  "/sys/class/leds/face-backlight/brightness",
  "/sys/class/leds/face-backlight-left/brightness",
  "/sys/class/leds/face-backlight-right/brightness"
};



int lcd_spi_init()
{
  // SPI setup
  int err = -1;
  static const uint8_t  MODE = 0;
  spi_fd = open("/dev/spidev1.0", O_RDWR);
  if (!spi_fd)  {
    error_exit(err_SPI_INIT);
  }
  if (spi_fd>0) {
    err = ioctl(spi_fd, SPI_IOC_RD_MODE, &MODE);
  }
  return spi_fd;
}




static void lcd_spi_transfer(int cmd, int bytes, const void* data) {
  const uint8_t* tx_buf = data;

  gpio_set_value(DnC_PIN, cmd ? gpio_LOW : gpio_HIGH);

  while (bytes > 0) {
    const size_t count = bytes > MAX_TRANSFER ? MAX_TRANSFER : bytes;

    write(spi_fd, tx_buf, count);

    bytes -= count;
    tx_buf += count;
  }
}

int lcd_spi_readback(int bytes,  const uint8_t* outbuf, uint8_t result[]) {

  struct spi_ioc_transfer mesg = {
    .tx_buf = (unsigned long)outbuf,
    .rx_buf = (unsigned long)result,
    .len = 1,
    .delay_usecs = 0,
    .speed_hz = DAT_CLOCK,
    .bits_per_word = 8,
  };

  gpio_set_value(DnC_PIN, gpio_LOW);
  int err = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &mesg);
  if (err < 1) {
    printf("cmd Error = %d %s \n", errno, strerror(errno));
    return -1;
  }
  mesg.tx_buf = (unsigned long)(&outbuf[1]);
  mesg.len = bytes-1;
  gpio_set_value(DnC_PIN, gpio_HIGH);
   err = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &mesg);
  if (err < 1) {
    printf("dat Error = %d %s \n", errno, strerror(errno));
    return -1;
  }
  return 0;
}


int lcd_device_read_status(void) {
  uint8_t message[] = {0x0F, 0x00, 0x00};
  uint8_t result[sizeof(message)] = {1,2,3};
  memset(result, 0, sizeof(result));
  if (lcd_spi_readback(sizeof(message), message, result))
  {
    printf("Readback error\n");
  }
  printf("Result = %x %x %x\n", result[0], result[1], result[2]);
  return result[2]==0;

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


void lcd_gpio_teardown(void) {
  if (DnC_PIN) {
    gpio_close(DnC_PIN);
  }
  if (RESET_PIN1) {
    gpio_close(RESET_PIN1);
  }
  if (RESET_PIN2) {
    gpio_close(RESET_PIN2);
  }
}


void lcd_gpio_setup(void) {
  // IO Setup
  DnC_PIN = gpio_create(GPIO_LCD_WRX, gpio_DIR_OUTPUT, gpio_HIGH);

  RESET_PIN1 = gpio_create_open_drain_output(GPIO_LCD_RESET1, gpio_HIGH);
  RESET_PIN2 = gpio_create_open_drain_output(GPIO_LCD_RESET2, gpio_HIGH);
}


int lcd_device_reset(void) {

  // Send reset signal
  microwait(50);
  gpio_set_value(RESET_PIN1, 0);
  gpio_set_value(RESET_PIN2, 0);
  microwait(50);
  gpio_set_value(RESET_PIN1, 1);
  gpio_set_value(RESET_PIN2, 1);
  microwait(50);

  static const uint8_t SLEEPOUT = 0x11;
  lcd_spi_transfer(true, 1,&SLEEPOUT);   //sleep off
  microwait(50);

  return 0;
}

void lcd_device_sleep(void)
{
    if (spi_fd) {
    static const uint8_t SLEEP = 0x10;
    lcd_spi_transfer(true, 1, &SLEEP);
    close(spi_fd);
  }

}
