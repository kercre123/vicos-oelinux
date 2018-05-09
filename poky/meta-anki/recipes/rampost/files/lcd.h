#ifndef LCD_H_
#define LCD_H_


int lcd_spi_init(void);

int lcd_device_read_status(void);

void lcd_gpio_teardown(void);
void lcd_gpio_setup(void);
int lcd_device_reset(void);

void lcd_device_sleep(void);

#endif//LCD_H_
