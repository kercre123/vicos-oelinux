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



/************** LCD size selection *****************/
#undef LCD_FRAME
#if 0   //list of frames that can be selected
#define LCD_FRAME 77890    //ORIGIN
#define LCD_FRAME 3022     //NV3022
#define LCD_FRAME 7789     //ST7789
#define LCD_FRAME 7735     //ST7735
#define LCD_FRAME MIDAS    //hybrid kinda
#endif
#define LCD_FRAME MIDAS 
#undef INIT_PROG

/************** LCD size selection *****************/
#define OLD_FRAME_WIDTH     184
#define OLD_FRAME_HEIGHT    96
#define LCD_FRAME_WIDTH     184
#define LCD_FRAME_HEIGHT    96

#ifdef LCD_FRAME
#if LCD_FRAME == MIDAS
#undef LCD_FRAME
#define LCD_FRAME 3022
#endif
#if LCD_FRAME == 77890
#undef LCD_FRAME_WIDTH
#undef LCD_FRAME_HEIGHT
#define LCD_FRAME_WIDTH     184
#define LCD_FRAME_HEIGHT    96
#define INIT_PROG st77890_init_scr
#endif
#if LCD_FRAME == 7789
#undef LCD_FRAME_WIDTH
#undef LCD_FRAME_HEIGHT
#define LCD_FRAME_WIDTH     240
#define LCD_FRAME_HEIGHT    135
#define INIT_PROG st7789_init_scr
#endif
#if LCD_FRAME == 7735
#undef LCD_FRAME_WIDTH
#undef LCD_FRAME_HEIGHT
#define LCD_FRAME_WIDTH     160
#define LCD_FRAME_HEIGHT    80
#define INIT_PROG st7735_init_scr
#endif
#if LCD_FRAME == 3022
#undef LCD_FRAME_WIDTH
#undef LCD_FRAME_HEIGHT
#define LCD_FRAME_WIDTH     160
#define LCD_FRAME_HEIGHT    80
#define INIT_PROG nv3022_init_scr
#endif
#endif

#define PXL_CNT  (LCD_FRAME_WIDTH * LCD_FRAME_HEIGHT)
#define OLP_CNT  (OLD_FRAME_WIDTH * OLD_FRAME_HEIGHT)
/************** END LCD selection *****************/



/************** LCD INTERFACE *****************/

#define GPIO_LCD_WRX   13
//<RevI>
#undef GPIO_LCD_WRX
#define GPIO_LCD_WRX   110
//<!RevI>
#define GPIO_LCD_RESET1 96
#define GPIO_LCD_RESET2 55

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))


#define LCD_FRAME_WIDTH     160
#define LCD_FRAME_HEIGHT    80
#define OLD_FRAME_WIDTH     184
#define OLD_FRAME_HEIGHT    96
#define PXL_CNT  (LCD_FRAME_WIDTH * LCD_FRAME_HEIGHT)
#define OLP_CNT  (OLD_FRAME_WIDTH * OLD_FRAME_HEIGHT)

#define RSHIFT 0x1C
#define XSHIFT 0x01
#define YSHIFT 0x1A

typedef struct LcdFrame_t {
  uint16_t data[LCD_FRAME_WIDTH*LCD_FRAME_HEIGHT];
} LcdFrame;

typedef struct {
  uint8_t cmd;
  uint8_t data_bytes;
  uint8_t data[32];
  uint32_t delay_ms;
} INIT_SCRIPT;

static const INIT_SCRIPT st77890_init_scr[] =
{
  { 0x10, 1, { 0x00 }, 120}, // Sleep in
  { 0x2A, 4, { 0x00, RSHIFT, (LCD_FRAME_WIDTH + RSHIFT - 1) >> 8, (LCD_FRAME_WIDTH + RSHIFT - 1) & 0xFF } }, // Column address set
  { 0x2B, 4, { 0x00, 0x00, (LCD_FRAME_HEIGHT -1) >> 8, (LCD_FRAME_HEIGHT -1) & 0xFF } }, // Row address set
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
static const INIT_SCRIPT st7789_init_scr[] =
{
  { 0x01, 1, { 0x00 }, 150}, // RESET
  { 0x11, 1, { 0x00 }, 10}, // Sleep out
  { 0x3A, 1, { 0x55 }, 10 }, // Interface pixel format (16 bit/pixel 65k RGB data)
  { 0x36, 1, { 0x08 }, 0 }, // Interface pixel format (16 bit/pixel 65k RGB data)
  { 0x2A, 4, { 0x00,0,0,240 }, 0 }, // Interface pixel format (16 bit/pixel 65k RGB data)
  { 0x2B, 4, { 0x00,0,0,135 }, 0 }, // Interface pixel format (16 bit/pixel 65k RGB data)

  { 0x21, 1, { 0x00 }, 10 }, // Display inversion on
  { 0x13, 1, { 0x00 }, 10 }, // Display normal on
  { 0x29, 1, { 0x00 }, 10 }, // Display on
  { 0 }
};
static const INIT_SCRIPT st7735_init_scr[] =
{
        { 0x01, 0, { 0x00}, 150},
        { 0x11, 0, { 0x00}, 500},
        { 0xB1, 3, { 0x01, 0x2c, 0x2d}, 0},
        { 0xB2, 3, { 0x01, 0x2c, 0x2d}, 0},
        { 0xB3, 6, { 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d}, 0},
        { 0xB4, 1, { 0x07}, 0},
        { 0xC0, 3, { 0xa2, 0x02, 0x84}, 0},
        { 0xC1, 1, { 0xc5}, 0},
        { 0xC2, 2, { 0x0a, 0x00}, 0},
        { 0xC3, 2, { 0x8a, 0x2a}, 0},
        { 0xC4, 2, { 0x8a, 0xee}, 0},
        { 0xC5, 1, { 0x0e}, 0},
        { 0x20, 0, { 0x00}, 0},
        { 0x36, 1, { 0xE0}, 0}, //exchange rotate and reverse order in each of 2 dim
        { 0x3A, 1, { 0x05}, 0}, // Interface pixel format (16 bit/pixel 65k RGB data)
        { 0xE0, 16, { 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10}, 0}, 
        { 0xE1, 16, { 0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d, 0x2e, 0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10}, 0}, 

        { 0x13, 0, { 0x00}, 100},
        { 0x21, 1, { 0x00 }, 0 }, // Display inversion on
        { 0x29, 0, { 0x00}, 10},

#if 0
	{ 0x2A, 4, { XSHIFT >> 8, XSHIFT & 0xFF, (LCD_FRAME_WIDTH + XSHIFT -1) >> 8, (LCD_FRAME_WIDTH + XSHIFT -1) & 0xFF } }, // Column address set
	{ 0x2B, 4, { YSHIFT >> 8, YSHIFT & 0xFF, (LCD_FRAME_HEIGHT + YSHIFT -1) >> 8, (LCD_FRAME_HEIGHT + YSHIFT -1) & 0xFF } }, // Row address set
	{ 0x2D ,128, { 
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 
		0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F 
		}, 0},
#else
	{ 0x2A, 4, { 0x00, 0x00, 0x00, 0x4F } }, //CASET
	{ 0x2B, 4, { 0x00, 0x00, 0x00, 0x9F } }, //RASET
        { 0x36, 1, { 0xC8 }, 0}, //MADCTL
        { 0x36, 1, { 0xC0 }, 0}, //MADCTL
        { 0x36, 1, { 0xE0 }, 0}, //MADCTL
#endif
{ 0 } 
};

static const INIT_SCRIPT nv3022_init_scr[] =
{
        { 0x01, 0, { 0x00}, 150},
        { 0x11, 0, { 0x00}, 500},
        { 0x20, 0, { 0x00}, 0},
        { 0x36, 1, { 0xE0}, 0}, //exchange rotate and reverse order in each of 2 dim
        { 0x3A, 1, { 0x65}, 0}, // Interface pixel format (16 bit/pixel 65k RGB data)

        { 0x13, 0, { 0x00}, 100},
        { 0x21, 1, { 0x00 }, 0 }, // Display inversion on
        { 0x29, 0, { 0x00}, 10},

	{ 0x2A, 4, { XSHIFT >> 8, XSHIFT & 0xFF, (LCD_FRAME_WIDTH + XSHIFT -1) >> 8, (LCD_FRAME_WIDTH + XSHIFT -1) & 0xFF } }, // Column address set
	{ 0x2B, 4, { YSHIFT >> 8, YSHIFT & 0xFF, (LCD_FRAME_HEIGHT + YSHIFT -1) >> 8, (LCD_FRAME_HEIGHT + YSHIFT -1) & 0xFF } }, // Row address set

{ 0 } 
};


static const INIT_SCRIPT display_on_scr[] = {
  { 0x11, 0, { 0x00 }, 120 }, // Sleep out
  { 0x29, 0, { 0x00 }, 120 }, // Display on
  { 0 }
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


void lcd_draw_frame2(const uint16_t* frame, size_t size) {
   static const uint8_t WRITE_RAM = 0x2C;
	int totfr = 0;
	uint8_t *frame2, *ifr_p, *ofr_p;
	int i, j, k, size2 = 0;

	if (!size)
		return;
	ifr_p = (uint8_t *) frame;
	totfr = size / (OLP_CNT*2);
	if (size % (OLP_CNT*2) )
		totfr++;
	size2 = totfr*PXL_CNT*2;
	frame2 = (uint8_t *) malloc(size2);
	ofr_p = frame2;
	for (i = 0; i < totfr; i++)
	{
		ifr_p += 2*3*OLD_FRAME_WIDTH;
		for ( j = 0; j < OLD_FRAME_HEIGHT-6; j++)
		{
			if ( j % 9 )
			{
				ifr_p += 2*2;
				for (k = 0; k < (OLD_FRAME_WIDTH-4)*2; k++)
				{
					if ( k % 18 )
					{
						if (ifr_p < (uint8_t *) frame + size)
							*ofr_p = *ifr_p;
						ofr_p++;
						ifr_p++;
					}
					else
					{
						ifr_p+=2;
						k++;
					}
				}
				ifr_p += 2*2;
			}
			else 
			{
				ifr_p += 2*OLD_FRAME_WIDTH;
			}
		}
		ifr_p += 2*3*OLD_FRAME_WIDTH;
	}

   	lcd_spi_transfer(true, 1, &WRITE_RAM);
   	lcd_spi_transfer(false, size2, frame2);
	free(frame2);
}

 void lcd_clear_screen(void) {
   const LcdFrame frame={{0}};
   lcd_draw_frame2(frame.data, sizeof(frame.data));
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
  usleep(50);
  gpio_set_value(RESET_PIN1, 1);
  gpio_set_value(RESET_PIN2, 1);
  usleep(250);

  static const uint8_t SLEEPOUT = 0x11;
  lcd_spi_transfer(true, 1,&SLEEPOUT);   //sleep off
  microwait(50);

  return 0;
}


static void lcd_run_script(const INIT_SCRIPT* script)
{
  int idx;
  for (idx = 0; script[idx].cmd; idx++) {
    lcd_spi_transfer(true, 1, &script[idx].cmd);
    lcd_spi_transfer(false, script[idx].data_bytes, script[idx].data);
    usleep(script[idx].delay_ms*1000);
  }
}



void lcd_device_init()
{
  // Init registers and put the display in sleep mode
  lcd_run_script(INIT_PROG);

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
