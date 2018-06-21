#ifndef LCD_H_
#define LCD_H_

#define LCD_FRAME_HEIGHT 184
#define LCD_FRAME_WIDTH  196


#ifdef __cplusplus
extern "C" {
#endif

typedef struct LcdFrame_t {
  uint16_t data[LCD_FRAME_WIDTH*LCD_FRAME_HEIGHT];
} LcdFrame;

enum LcdColor {
  lcd_BLACK = 0x0000,
  lcd_BLUE    = 0x001F,
  lcd_GREEN   = 0x07E0,
  lcd_RED     = 0xF800,
  lcd_WHITE = 0xFFFF,
};

int lcd_init(void);
void lcd_set_brightness(int b); //0..20

int lcd_spi_init(void);

int lcd_device_read_status(void);

void lcd_gpio_teardown(void);
void lcd_gpio_setup(void);
int lcd_device_reset(void);

void lcd_device_sleep(void);

void lcd_draw_frame(const LcdFrame* frame);

#ifdef __cplusplus
}
#endif


#endif//LCD_H_
