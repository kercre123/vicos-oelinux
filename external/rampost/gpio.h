#ifndef RAMPOST_GPIO_H
#define RAMPOST_GPIO_H

enum Gpio_Dir {
  gpio_DIR_INPUT,
  gpio_DIR_OUTPUT
};

enum Gpio_Level {
  gpio_LOW,
  gpio_HIGH
};

struct GPIO_t;
typedef struct GPIO_t* GPIO;



/************* GPIO Interface ***************/

GPIO gpio_create(int gpio_number, enum Gpio_Dir isOutput, enum Gpio_Level initial_value);

GPIO gpio_create_open_drain_output(int gpio_number, enum Gpio_Level initial_value);

void gpio_set_direction(GPIO gp, enum Gpio_Dir isOutput);

void gpio_set_value(GPIO gp, enum Gpio_Level value);

enum Gpio_Level gpio_get_value(GPIO gp);

void gpio_close(GPIO gp);


#endif//RAMPOST_GPIO_H
