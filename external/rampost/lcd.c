#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "gpio.h"
#include "rampost.h"
#include "das.h"
#include "lcd.h"

/************** LCD INTERFACE *****************/

#define GPIO_LCD_WRX   110
#define GPIO_LCD_RESET1 55

#define LCD_FRAME_WIDTH     184
#define LCD_FRAME_HEIGHT    96

#define RSHIFT 0x1C

typedef struct LcdFrame_t {
  uint16_t data[LCD_FRAME_WIDTH * LCD_FRAME_HEIGHT];
} LcdFrame;

typedef struct {
  uint8_t cmd;
  uint8_t data_bytes;
  uint8_t data[14];
  uint32_t delay_ms;
} INIT_SCRIPT;

static const INIT_SCRIPT init_scr[] = {
  { 0x10, 1, { 0x00 }, 120}, // Sleep in
  { 0x2A, 4, { 0x00, RSHIFT, (LCD_FRAME_WIDTH + RSHIFT - 1) >> 8, (LCD_FRAME_WIDTH + RSHIFT - 1) & 0xFF } }, // Column address set
  { 0x2B, 4, { 0x00, 0x00, (LCD_FRAME_HEIGHT - 1) >> 8, (LCD_FRAME_HEIGHT - 1) & 0xFF } }, // Row address set
  { 0x36, 1, { 0x00 }, 0 }, // Memory data access control
  { 0x3A, 1, { 0x55 }, 0 }, // Interface pixel format (16 bit/pixel 65k RGB data)
  { 0xB0, 2, { 0x00, 0x08 } }, // RAM control (LSB first)
  { 0xB2, 5, { 0x0C, 0x0C, 0x00, 0x33, 0x33 }, 0 }, // Porch setting
  { 0xB7, 1, { 0x72 }, 0 }, // Gate control (VGH 14.97v, VGL -8.23v)
  { 0xBB, 1, { 0x3B }, 0 }, // VCOMS setting (1.575v)
  { 0xC0, 1, { 0x2C }, 0 }, // LCM control
  { 0xC2, 1, { 0x01 }, 0 }, // VDV and VRH command enable
  { 0xC3, 1, { 0x14 }, 0 }, // VRH set
  { 0xC4, 1, { 0x20 }, 0 }, // VDV set
  { 0xC6, 1, { 0x0F }, 0 }, // Frame rate control in normal mode (60hz)
  { 0xD0, 2, { 0xA4, 0xA1 }, 0 }, // Power control 1
  { 0xE0, 14, { 0xD0, 0x10, 0x16, 0x0A, 0x0A, 0x26, 0x3C, 0x53, 0x53, 0x18, 0x15, 0x12, 0x36, 0x3C }, 0 }, // Positive voltage gamma control
  { 0xE1, 14, { 0xD0, 0x11, 0x19, 0x0A, 0x09, 0x25, 0x3D, 0x35, 0x54, 0x17, 0x15, 0x12, 0x36, 0x3C }, 0 }, // Negative voltage gamma control
  { 0xE9, 3, { 0x05, 0x05, 0x01 }, 0 }, // Equalize time control
  { 0 }
};


static const INIT_SCRIPT display_on_scr[] = {
  { 0x21, 1, { 0x00 }, 0 }, // Display inversion on
  { 0x11, 1, { 0x00 }, 120 }, // Sleep out
  { 0x29, 1, { 0x00 }, 120 }, // Display on
  { 0 }
};


static GPIO RESET_PIN1;
static GPIO DnC_PIN;

static const int MAX_TRANSFER = 0x1000;

static int spi_fd;

static const char* BACKLIGHT_DEVICES[] = {
  "/sys/class/leds/face-backlight-left/brightness",
  "/sys/class/leds/face-backlight-right/brightness"
};


int lcd_spi_init()
{
  // SPI setup
  static const uint8_t  MODE = 0;
  spi_fd = open("/dev/spidev1.0", O_RDWR);
  if (!spi_fd)  {
    error_exit(err_SPI_INIT);
  }
  if (spi_fd > 0) {
    ioctl(spi_fd, SPI_IOC_RD_MODE, &MODE);
  }
  return spi_fd;
}

static void lcd_spi_transfer(int cmd, int bytes, const void* data)
{
  const uint8_t* tx_buf = data;

  gpio_set_value(DnC_PIN, cmd ? gpio_LOW : gpio_HIGH);

  while (bytes > 0) {
    const size_t count = bytes > MAX_TRANSFER ? MAX_TRANSFER : bytes;

    write(spi_fd, tx_buf, count);

    bytes -= count;
    tx_buf += count;
  }
}

static void _led_set_brightness(const int brightness, const char* led)
{
  int fd = open(led, O_WRONLY);
  if (fd) {
    char buf[3];
    snprintf(buf, 3, "%02d\n", brightness);
    write(fd, buf, 3);
    close(fd);
  }
}

void lcd_set_brightness(int brightness)
{
  int l;
  if (brightness > 10) { brightness = 10; }
  if (brightness <  0) { brightness =  0; }

  const int numLights = sizeof(BACKLIGHT_DEVICES)/sizeof(BACKLIGHT_DEVICES[0]);
  for (l = 0; l < numLights; ++l) {
    _led_set_brightness(brightness, BACKLIGHT_DEVICES[l]);
  }
}


void lcd_draw_frame2(const uint16_t* frame, size_t size)
{
  static const uint8_t WRITE_RAM = 0x2C;
  lcd_spi_transfer(true, 1, &WRITE_RAM);
  lcd_spi_transfer(false, size, frame);
}

void lcd_clear_screen(void)
{
  const LcdFrame frame = {{0}};
  lcd_draw_frame2(frame.data, sizeof(frame.data));
}

void lcd_gpio_teardown(void)
{
  if (DnC_PIN) {
    gpio_close(DnC_PIN);
  }
  if (RESET_PIN1) {
    gpio_close(RESET_PIN1);
  }
}


void lcd_gpio_setup(void)
{
  // IO Setup
  DnC_PIN = gpio_create(GPIO_LCD_WRX, gpio_DIR_OUTPUT, gpio_HIGH);

  RESET_PIN1 = gpio_create_open_drain_output(GPIO_LCD_RESET1, gpio_HIGH);
}


int lcd_device_reset(void)
{

  // Send reset signal
  microwait(50);
  gpio_set_value(RESET_PIN1, 0);
  microwait(50);
  gpio_set_value(RESET_PIN1, 1);
  microwait(50);

  return 0;
}


static void lcd_run_script(const INIT_SCRIPT* script)
{
  int idx;
  for (idx = 0; script[idx].cmd; idx++) {
    lcd_spi_transfer(true, 1, &script[idx].cmd);
    lcd_spi_transfer(false, script[idx].data_bytes, script[idx].data);
    usleep(script[idx].delay_ms * 1000);
  }
}

void lcd_device_init(void)
{
  // Init registers and put the display in sleep mode
  lcd_run_script(init_scr);

  // Clear lcd memory before turning display on
  // as the contents of memory are set randomly on
  // power on
  lcd_clear_screen();

  // Turn display on
  lcd_run_script(display_on_scr);
}

void lcd_device_sleep(void)
{
  if (spi_fd) {
    static const uint8_t SLEEP = 0x10;
    lcd_spi_transfer(true, 1, &SLEEP);
    close(spi_fd);
  }

}
