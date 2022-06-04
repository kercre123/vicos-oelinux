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
#include "lcd.h"

void microwait(long microsec);

/************** LCD INTERFACE *****************/

#define GPIO_LCD_WRX   110
#define GPIO_LCD_RESET1 96
#define GPIO_LCD_RESET2 55

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))


#define LCD_FRAME_WIDTH     184
#define LCD_FRAME_HEIGHT    96

#define LCD_FRAME_WIDTH_SANTEK    184
#define LCD_FRAME_HEIGHT_SANTEK   96
#define LCD_FRAME_WIDTH_MIDAS     160
#define LCD_FRAME_HEIGHT_MIDAS    80

#define PXL_CNT_SANTEK  (LCD_FRAME_WIDTH_SANTEK * LCD_FRAME_HEIGHT_SANTEK)
#define PXL_CNT_MIDAS   (LCD_FRAME_WIDTH_SANTEK * LCD_FRAME_HEIGHT_MIDAS)
#define OLP_CNT  (OLD_FRAME_WIDTH * OLD_FRAME_HEIGHT)

#define RSHIFT 0x1C
#define XSHIFT 0x0
#define YSHIFT 0x18

#define MSB(a) ((a) >> 8)
#define LSB(a) ((a) & 0xff)

typedef struct LcdFrame_t {
  uint16_t data[LCD_FRAME_WIDTH * LCD_FRAME_HEIGHT];
} LcdFrame;

typedef struct {
  uint8_t cmd;
  uint8_t data_bytes;
  uint8_t data[64];
  uint32_t delay_ms;
} INIT_SCRIPT;

static const INIT_SCRIPT init_scr_santek[] = {
  { 0x10, 1, { 0x00 }, 120}, // Sleep in
  { 0x2A, 4, { 0x00, RSHIFT, (LCD_FRAME_WIDTH_SANTEK + RSHIFT - 1) >> 8, (LCD_FRAME_WIDTH_SANTEK + RSHIFT - 1) & 0xFF } }, // Column address set
  { 0x2B, 4, { 0x00, 0x00, (LCD_FRAME_HEIGHT_SANTEK -1) >> 8, (LCD_FRAME_HEIGHT_SANTEK -1) & 0xFF } }, // Row address set
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
  { 0x21, 1, { 0x00 }, 0 }, // Display inversion on
  { 0 }
};

static const INIT_SCRIPT init_scr_midas[] =
  {
    { 0x01, 0, { 0x00}, 150}, // Software reset
    { 0x11, 0, { 0x00}, 500}, // Sleep out
    { 0x20, 0, { 0x00}, 0}, // Display inversion off
    { 0x36, 1, { 0xA8}, 0}, //exchange rotate and reverse order in each of 2 dim
    { 0x3A, 1, { 0x05}, 0}, // Interface pixel format (16 bit/pixel 65k RGB data)

    { 0xE0, 16, { 0x07, 0x0e, 0x08, 0x07, 0x10, 0x07, 0x02, 0x07, 0x09, 0x0f, 0x25, 0x36, 0x00, 0x08, 0x04, 0x10 }, 0},  //
    { 0xE1, 16, { 0x0a, 0x0d, 0x08, 0x07, 0x0f, 0x07, 0x02, 0x07, 0x09, 0x0f, 0x25, 0x35, 0x00, 0x09, 0x04, 0x10 }, 0},

    { 0xFC, 1, { 128+64}, 0},

    { 0x13, 0, { 0x00}, 100}, // Normal Display Mode On
    // { 0x21, 0, { 0x00 }, 10 }, // Display inversion on
    // { 0x20, 0, { 0x00 }, 10 }, // Display inversion off
    { 0x26, 1, {0x02} , 10}, // Set Gamma
    { 0x29, 0, { 0x00}, 10}, // Display On


    { 0x2A, 4, { MSB(XSHIFT), LSB(XSHIFT), MSB(LCD_FRAME_WIDTH_MIDAS + XSHIFT - 1), LSB(LCD_FRAME_WIDTH_MIDAS + XSHIFT - 1) } }, // Column address set
    { 0x2B, 4, { MSB(YSHIFT), LSB(YSHIFT), MSB(LCD_FRAME_HEIGHT_MIDAS + YSHIFT -1), LSB(LCD_FRAME_HEIGHT_MIDAS + YSHIFT -1) } }, // Row address set

    { 0 }
  };


static const INIT_SCRIPT sleep_in_santek[] = {
  { 0x10, 1, { 0x00 }, 5 },
  { 0 }
};

static const INIT_SCRIPT sleep_in_midas[] = {
  { 0x10, 1, { 0x00 }, 5 },
  { 0 }
};

static const INIT_SCRIPT display_on_scr_santek[] = {
  { 0x11, 1, { 0x00 }, 120 }, // Sleep out
  { 0x29, 1, { 0x00 }, 120 }, // Display on
  {0}
};

static const INIT_SCRIPT display_on_scr_midas[] = {
  { 0x11, 1, { 0x00 }, 120 }, // Sleep out
  { 0x29, 1, { 0x00 }, 120 }, // Display on
  {0}
};


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

typedef enum {
  MIDAS,
  SANTEK,
    INVALID
} display_version_t;

static display_version_t display_version() {
    return MIDAS;
}

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

int lcd_spi_readback(int bytes,  const uint8_t* outbuf, uint8_t result[])
{

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
    printf("lcd.spi.cmd_error: %d %s\n", errno, strerror(errno));
    return -1;
  }
  mesg.tx_buf = (unsigned long)(&outbuf[1]);
  mesg.len = bytes - 1;
  gpio_set_value(DnC_PIN, gpio_HIGH);
  err = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &mesg);
  if (err < 1) {
    printf("lcd.spi.dat_error: %d %s\n", errno, strerror(errno));
    return -1;
  }
  return 0;
}


int lcd_device_read_status(void)
{
  uint8_t message[] = {0x0F, 0x00, 0x00};
  uint8_t result[sizeof(message)] = {1, 2, 3};
  memset(result, 0, sizeof(result));
  if (lcd_spi_readback(sizeof(message), message, result)) {
    printf("lcd.read_status: %s\n", "readback error");
  }
  printf("lcd.read_status: Result = %x %x %x\n", result[0], result[1], result[2]);
  return result[2] == 0;
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
  brightness = MIN(brightness, 10);
  brightness = MAX(brightness, 0);
  for (l = 0; l < 3; ++l) {
    _led_set_brightness(brightness, BACKLIGHT_DEVICES[l]);
  }
}


// Here we jsut crop the images cutting off top/bottom/and sides
// Because rampost is running inside a ram filesystem we can't
// load up a bunch of files for alternate images, and we don't
// want to increase the size of this binary by creating files like
// anki_dev_unit_v2 with the raw data stored as a struct.
void lcd_draw_frame2_midas(const uint16_t* frame, size_t size)
{
  static const uint8_t WRITE_RAM = 0x2C;

  lcd_spi_transfer(true, 1, &WRITE_RAM);

  uint16_t new_row[LCD_FRAME_WIDTH_MIDAS];

  int i,j;
  for(i=0;i<LCD_FRAME_HEIGHT_MIDAS;i++) {
    const uint16_t* row = (uint16_t*)frame + ((i+12) * LCD_FRAME_WIDTH_SANTEK) + 8;
    for(j=0;j<LCD_FRAME_WIDTH_MIDAS;j++) {
      new_row[j] = __builtin_bswap16(row[j]);
    }
    lcd_spi_transfer(false, (LCD_FRAME_WIDTH_MIDAS )* 2, new_row);
  }

}

void lcd_draw_frame2_santek(const uint16_t* frame, size_t size)
{
  static const uint8_t WRITE_RAM = 0x2C;
  lcd_spi_transfer(true, 1, &WRITE_RAM);
  lcd_spi_transfer(false, size, frame);
}



void lcd_draw_frame2(const uint16_t* frame, size_t size)
{
  if(display_version() == MIDAS) {
    lcd_draw_frame2_midas(frame,size);
  } else {
    lcd_draw_frame2_santek(frame,size);
  }
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
  if (RESET_PIN2) {
    gpio_close(RESET_PIN2);
  }
}


void lcd_gpio_setup(void)
{
  // IO Setup
  DnC_PIN = gpio_create(GPIO_LCD_WRX, gpio_DIR_OUTPUT, gpio_HIGH);

  RESET_PIN1 = gpio_create_open_drain_output(GPIO_LCD_RESET1, gpio_HIGH);
  if (display_version() == MIDAS) {
    RESET_PIN2 = gpio_create(GPIO_LCD_RESET2,gpio_DIR_OUTPUT,  gpio_HIGH);
  } else {
    RESET_PIN2 = gpio_create_open_drain_output(GPIO_LCD_RESET2, gpio_HIGH);
  }
}


int lcd_device_reset(void)
{

  // Send reset signal
  microwait(50);
  gpio_set_value(RESET_PIN1, 0);
  gpio_set_value(RESET_PIN2, 0);
  microwait(50);
  gpio_set_value(RESET_PIN1, 1);
  gpio_set_value(RESET_PIN2, 1);
  microwait(50);

  static const uint8_t SLEEPOUT = 0x11;
  lcd_spi_transfer(true, 1, &SLEEPOUT);  //sleep off
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
  if(display_version() == MIDAS) {
    lcd_run_script(init_scr_midas);
  } else {
    lcd_run_script(init_scr_santek);
  }

  // Clear lcd memory before turning display on
  // as the contents of memory are set randomly on
  // power on
  lcd_clear_screen();

  // Turn display on
  if(display_version() == MIDAS) {
    lcd_run_script(display_on_scr_midas);
  } else {
    lcd_run_script(display_on_scr_santek);
  }
}

void lcd_device_sleep(void)
{
  if (spi_fd) {
    static const uint8_t SLEEP = 0x10;
    lcd_spi_transfer(true, 1, &SLEEP);
    close(spi_fd);
  }

}
