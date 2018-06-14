


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


} RampostErr;

int error_exit(RampostErr err);
