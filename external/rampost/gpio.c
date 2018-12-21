/************* GPIO Interface ***************/

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <stdbool.h>
#include <ctype.h>

#include "rampost.h"

enum Gpio_Dir {
  gpio_DIR_INPUT,
  gpio_DIR_OUTPUT
};

enum Gpio_Level {
  gpio_LOW,
  gpio_HIGH
};


static int GPIO_BASE_OFFSET = -1;

struct GPIO_t {
  int pin;
  int fd;
  bool isOpenDrain;
};
typedef struct GPIO_t* GPIO;


int gpio_get_base_offset()
{
  if (GPIO_BASE_OFFSET < 0) {
    // gpio pinctrl for msm8909:
    // oe-linux/3.18: /sys/devices/soc/1000000.pinctrl/gpio/gpiochip0/base     -> 0
    // android/3.10:  /sys/devices/soc.0/1000000.pinctrl/gpio/gpiochip911/base -> 911

    // Assume we are on an OE-linux system
    int fd = open("/sys/devices/soc/1000000.pinctrl/gpio/gpiochip0/base", O_RDONLY);
    if (fd < 0) {
      // Fallback to Android
      fd = open("/sys/devices/soc.0/1000000.pinctrl/gpio/gpiochip911/base", O_RDONLY);
    }

    if (fd < 0) {
      error_exit(err_GPIOCHIP_BASE);
    }

    char base_buf[5] = {0};
    int r = read(fd, base_buf, sizeof(base_buf));
    if (r < 0) {
      error_exit(err_GPIOCHIP_BASE);
    }

    if (isdigit(base_buf[0]) == 0) {
      error_exit(err_GPIOCHIP_BASE);
    }

    GPIO_BASE_OFFSET = atoi(base_buf);
  }

  return GPIO_BASE_OFFSET;
}

static inline enum Gpio_Dir gpio_drain_direction(enum Gpio_Level value)
{
  return value == gpio_LOW ? gpio_DIR_OUTPUT : gpio_DIR_INPUT;
}


void gpio_set_direction(GPIO gp, enum Gpio_Dir direction)
{
  assert(gp != NULL);
  char ioname[40];
  snprintf(ioname, 40, "/sys/class/gpio/gpio%d/direction", gp->pin + gpio_get_base_offset());
  int fd =  open(ioname, O_WRONLY);
  if (fd <  0) {
    error_exit(err_GPIO_DIRECTION);
  }
  if (direction == gpio_DIR_OUTPUT) {
    write(fd, "out", 3);
  }
  else {
    write(fd, "in", 2);
  }
  close(fd);
}

void gpio_set_value(GPIO gp, enum Gpio_Level value)
{
  assert(gp != NULL);
  if (gp->isOpenDrain) {
    gpio_set_direction(gp, gpio_drain_direction(value));
    return;
  }
  static const char* trigger[] = {"0", "1"};
  write(gp->fd, trigger[value != 0], 1);
}

GPIO gpio_create(int gpio_number, enum Gpio_Dir direction, enum Gpio_Level initial_value)
{
  char ioname[32];
  GPIO gp  = malloc(sizeof(struct GPIO_t));
  if (!gp) { error_exit(err_MEMORY_ERROR); }

  //create io
  int fd = open("/sys/class/gpio/export", O_WRONLY);
  if (fd < 0) {
    free(gp);
    error_exit(err_GPIO_CREATE);
  }
  snprintf(ioname, 32, "%d\n", gpio_number + gpio_get_base_offset());
  write(fd, ioname, strlen(ioname));
  close(fd);

  gp->pin = gpio_number;
  gp->isOpenDrain = false;

  //set direction
  gpio_set_direction(gp, direction);

  //open value fd
  snprintf(ioname, 32, "/sys/class/gpio/gpio%d/value", gpio_number + gpio_get_base_offset());
  fd = open(ioname, O_WRONLY | O_CREAT, S_IWUSR);

  if (fd < 0) {
    free(gp);
    error_exit(err_GPIO_CREATE);
  }
  gp->fd = fd;
  if (fd > 0) {
    gpio_set_value(gp, initial_value);
  }
  gp->isOpenDrain = false;
  return gp;
}

GPIO gpio_create_open_drain_output(int gpio_number, enum Gpio_Level initial_value)
{
  enum Gpio_Dir initial_dir = gpio_drain_direction(initial_value);
  GPIO gp = gpio_create(gpio_number, initial_dir, gpio_LOW);
  gp->isOpenDrain = true;
  return gp;
}

void gpio_close(GPIO gp)
{
  assert(gp != NULL);
  if (gp->fd > 0) {
    close(gp->fd);
  }
  free(gp);
}
