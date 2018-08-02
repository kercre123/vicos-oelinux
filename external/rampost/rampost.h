#ifndef __RAMPOST_H
#define __RAMPOST_H

#include <stdint.h>

uint64_t steady_clock_now(void);

typedef enum rampost_err_t {
  err_OK,
  err_GPIOCHIP_BASE = 1,
  err_GPIO_DIRECTION,
  err_MEMORY_ERROR,
  err_GPIO_CREATE,
  err_SPI_INIT,
  err_ALREADY_OPEN,
  err_CANT_OPEN_FILE,
  err_TERMIOS_FAIL,
  err_DFU_FILE_OPEN,
  err_DFU_FILE_READ,
  err_DFU_SEND,
  err_SYSCON_READ,
  err_SYSCON_ERASE,
  err_SYSCON_VALIDATE,
  err_SYSCON_WRITE,

} RampostErr;

int error_exit(RampostErr err);

#endif
