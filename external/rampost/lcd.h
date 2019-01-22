#ifndef LCD_H_
#define LCD_H_


int lcd_spi_init(void);
void lcd_device_init(void);
int lcd_device_read_status(void);

void lcd_gpio_teardown(void);
void lcd_gpio_setup(void);
int lcd_device_reset(void);

void lcd_set_brightness(int brightness);
void lcd_draw_frame2(const uint16_t* frame, size_t size);

void lcd_device_sleep(void);

#endif//LCD_H_
