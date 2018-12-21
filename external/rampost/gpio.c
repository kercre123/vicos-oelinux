/************* GPIO Interface ***************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>

#include "rampost.h"
#include "gpio.h"

struct GPIO_t {
  int pin;
  int fd;
  bool isOpenDrain;
};
typedef struct GPIO_t* GPIO;


static inline enum Gpio_Dir gpio_drain_direction(enum Gpio_Level value)
{
  return value == gpio_LOW ? gpio_DIR_OUTPUT : gpio_DIR_INPUT;
}


void gpio_set_direction(GPIO gp, enum Gpio_Dir direction)
{
  assert(gp != NULL);
  char ioname[40];
  snprintf(ioname, 40, "/sys/class/gpio/gpio%d/direction", gp->pin);
  int fd =  open(ioname, O_WRONLY );
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

enum Gpio_Level  gpio_get_value(GPIO gp)
{
  assert(gp != NULL);
  char buf[2] = {0};
  lseek(gp->fd, 0, SEEK_SET);
  read(gp->fd, buf, 1);
  return (buf[0] == '1') ? gpio_HIGH : gpio_LOW;
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
  snprintf(ioname, 32, "%d\n", gpio_number);
  write(fd, ioname, strlen(ioname));
  close(fd);

  gp->pin = gpio_number;
  gp->isOpenDrain = false;

  //set direction
  gpio_set_direction(gp, direction);

  //open value fd
  snprintf(ioname, 32, "/sys/class/gpio/gpio%d/value", gpio_number);
  fd = open(ioname, O_RDWR | O_CREAT, S_IWUSR);

  if (fd < 0) {
    free(gp);
    error_exit(err_GPIO_CREATE);
  }
  gp->fd = fd;
  if  (fd > 0) {
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

void gpio_destroy(GPIO gp)
{
  char ioname[32];
  //set direction to input to ensure it is no longer driven
  gpio_set_direction(gp, gpio_DIR_INPUT);

  //close the value file handle
  assert(gp != NULL);
  if (gp->fd > 0) {
    close(gp->fd);
  }

  //unexport this pin
  int fd = open("/sys/class/gpio/unexport", O_WRONLY);
  if (fd < 0) {
    free(gp);
    error_exit(err_GPIO_DESTROY);
  }
  snprintf(ioname, 32, "%d\n", gp->pin);
  write(fd, ioname, strlen(ioname));
  close(fd);

  free(gp);
}

void gpio_close(GPIO gp)
{
  assert(gp != NULL);
  if (gp->fd > 0) {
    close(gp->fd);
  }
  free(gp);
}

