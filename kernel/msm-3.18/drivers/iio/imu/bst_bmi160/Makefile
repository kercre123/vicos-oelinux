#
# Makefile for Bosch BMI160 IMU
#
obj-$(CONFIG_BMI160) += bmi160_core.o
obj-$(CONFIG_BMI160_I2C) += bmi160_i2c.o
obj-$(CONFIG_BMI160_SPI) += bmi160_spi.o
obj-$(CONFIG_SPI_TEST) += spi_test.o
BMI_CFLAGS += -g -DDEBUG
ccflags-y += ${BMI_CFLAGS}
CC += ${BMI_CFLAGS}
