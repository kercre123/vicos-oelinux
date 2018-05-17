#ifndef IMU_H_
#define IMU_H_

#define DEFAULT_IMU_SPI_DEVICE "/dev/spidev0.0"

#define IMU_ACC_RANGE 2.0    // g       [2.11.12: acc_range 2 => +- 2g ]
#define MAX_16BIT_POSITIVE 0x7FFF
#define IMU_ACCEL_SCALE_G ((double)(IMU_ACC_RANGE)/MAX_16BIT_POSITIVE)
#define MIN_INVERTED_G  ((long)(0.5 / IMU_ACCEL_SCALE_G))

#define IMU_MAX_SAMPLES_PER_READ 3

typedef struct __attribute__((packed)) IMURawData_t {
   int16_t gyro[3];
   int16_t acc[3];
   uint32_t timestamp;
   int16_t temperature;
} IMURawData;

#endif//imu_h_
