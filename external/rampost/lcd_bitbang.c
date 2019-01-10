#include <stdio.h>
#include <stdint.h>
#include <time.h>


#include "gpio.h"
#include "das.h"
#include "rampost.h"
#include "lcd.h"

#define milliwait(ms) microwait(1000L*(ms))

/************** LCD INTERFACE *****************/

#define GPIO_LCD_WRX   110
#define GPIO_LCD_RESET1 55

#define GPIO_LCD_MOSI 12
#define GPIO_LCD_MISO 13
#define GPIO_LCD_SELECT 14
#define GPIO_LCD_CLOCK 15


static GPIO DnC_PIN;

static GPIO SELECT_PIN;
static GPIO CLOCK_PIN;
static GPIO MOSI_PIN;
static GPIO MISO_PIN;


void gpio_destroy(GPIO gp);

static void lcd_bitbang_teardown(void)
{
  if (SELECT_PIN) { gpio_destroy(SELECT_PIN); }
  if (CLOCK_PIN)  { gpio_destroy(CLOCK_PIN);  }
  if (MOSI_PIN)   { gpio_destroy(MOSI_PIN);   }
  if (MISO_PIN)   { gpio_destroy(MISO_PIN);   }
  //destroying this one clears the screen, so just close
  if (DnC_PIN)    { gpio_close(DnC_PIN);    }
}

static void lcd_bitbang_setup(void)
{
  // IO Setup
  DnC_PIN = gpio_create(GPIO_LCD_WRX, gpio_DIR_OUTPUT, gpio_HIGH);
  SELECT_PIN = gpio_create(GPIO_LCD_SELECT, gpio_DIR_OUTPUT, gpio_HIGH);
  CLOCK_PIN = gpio_create(GPIO_LCD_CLOCK, gpio_DIR_OUTPUT, gpio_LOW);
  MOSI_PIN = gpio_create(GPIO_LCD_MOSI, gpio_DIR_OUTPUT, gpio_LOW);
  MISO_PIN = gpio_create(GPIO_LCD_MISO, gpio_DIR_INPUT, gpio_LOW);
}


static inline void lcd_bitbang_clockout(uint8_t byte)
{
  for (int bitcount = 0; bitcount < 8; bitcount++) {
    gpio_set_value(MOSI_PIN, (byte & 0x80) ? gpio_HIGH : gpio_LOW);
    gpio_set_value(CLOCK_PIN, gpio_LOW);
    byte <<= 1;
    gpio_set_value(CLOCK_PIN, gpio_HIGH);
  }
}

static inline uint8_t lcd_bitbang_clockin(void)
{
  uint8_t byte = 0;
  for (int bitcount = 0; bitcount < 8; bitcount++) {
    gpio_set_value(CLOCK_PIN, gpio_LOW);
    byte <<= 1;
    byte |= gpio_get_value(MISO_PIN);
    gpio_set_value(CLOCK_PIN, gpio_HIGH);
  }
  return byte;
}

static void lcd_bitbang_write(int cmd, const uint8_t outbuf[], size_t data_bytes)
{
  gpio_set_value(DnC_PIN, gpio_LOW);
  gpio_set_value(SELECT_PIN, gpio_LOW);

  lcd_bitbang_clockout(cmd);

  //toggle data/command pin
  gpio_set_value(DnC_PIN, gpio_HIGH);

  while (data_bytes-- > 0) {
    lcd_bitbang_clockout(*outbuf++);
  }

  gpio_set_value(MOSI_PIN, gpio_LOW);
  gpio_set_value(SELECT_PIN, gpio_HIGH);
}

static void lcd_bitbang_read(uint8_t cmd, uint8_t result[], size_t bytes_expected)
{
  gpio_set_value(DnC_PIN, gpio_LOW);
  gpio_set_value(SELECT_PIN, gpio_LOW);

  //shift out command
  lcd_bitbang_clockout(cmd);

  gpio_set_value(DnC_PIN, gpio_HIGH);
  gpio_set_value(MOSI_PIN, gpio_HIGH);

  if (bytes_expected > 1)  {
    //we need one extra clock cycle for multi-byte reads.
    gpio_set_value(CLOCK_PIN, gpio_LOW);
    gpio_set_value(CLOCK_PIN, gpio_HIGH);
  }

  //shift in result:
  while (bytes_expected-- > 0) {
    *result++ = lcd_bitbang_clockin();
  }

  gpio_set_value(MOSI_PIN, gpio_LOW);
  gpio_set_value(SELECT_PIN, gpio_HIGH);

}

enum {
  cmd_GetDeviceId = 0x04,
  cmd_GetStatus   = 0x0F,
  cmd_SleepIn     = 0x10,
  cmd_SleepOut    = 0x11
};


int lcd_status_check(void)
{

  /* Preconditions: Device must have been reset
     device SPI should not be initialized
  */
  uint8_t id[3];
  uint8_t status[1];
  uint8_t zerobyte[1] = {0};

  lcd_bitbang_setup();

  lcd_bitbang_read(cmd_GetDeviceId, id, sizeof(id));

  //we need to issue SleepOut in order for status to be meaningful
  //we need to wait 120 ms before status is available (ST7789V pg 181)
  lcd_bitbang_write(cmd_SleepOut, zerobyte, sizeof(zerobyte));
  milliwait(120);

  lcd_bitbang_read(cmd_GetStatus, status, sizeof(status));

  lcd_bitbang_write(cmd_SleepIn, zerobyte, sizeof(zerobyte));
  //"It will be necessary to wait 5msec before sending any new commands..." (ST7789V pg 179)
  milliwait(5);

  DAS_LOG(DAS_DEBUG, "lcd.read_status", "id = %02x%02x%02x,  result = %x",
          id[0], id[1], id[2], status[0]);

  lcd_bitbang_teardown();

  //check against the magic numbers from the manufacturer. (ST7789V datasheet pg 162)
  int good_id = ((id[0] == 0x85) && (id[1] == 0x85) && (id[2] == 0x52));
  // check self-Diagnostic bits. 2 high bits are `11`. (ST7789V datasheet pg 177)
  int good_selfcheck = ((status[0] & 0xC0) == 0xC0);
  return (good_id && good_selfcheck);
}
