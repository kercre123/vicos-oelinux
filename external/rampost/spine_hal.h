#ifndef SPINE_HAL_H
#define SPINE_HAL_H

#include <stdint.h>
typedef int SpineErr;

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_SERIAL_POLL_INTERVAL_US 200


//opens UART, does setup
int hal_init(const char* devicename, long baudrate);

//Spins until valid frame of `type` is recieved, or `timeout_ms` elapses. returns whole frame
const void* hal_get_frame(uint16_t type, int32_t timeout_ms);

//Spins until any valid frame is recieved, or `timeout_ms` elapses. returns whole frame
const void* hal_get_next_frame(const int32_t timeout_ms);

//Spins until valid frame of `type` is recieved, or timeout returns whole frame
const void* hal_wait_for_frame(const uint16_t type, const int32_t timeout_ms);

//Spins until valid frame of `type` is recieved, or ACK frame with negative payload or timeout returns whole frame
const void* hal_wait_for_frame_or_nack(const uint16_t type, const int32_t timeout_ms);

//Sends given frame
void hal_send_frame(uint16_t type, const void* data, int len);

//low level
int hal_serial_send(const uint8_t* buffer, int len);


enum RobotMode { //todo: mode is a dummy value. If ever needed, this should be in clad file.
  RobotMode_IDLE,
  RobotMode_RUN,
};

void hal_set_mode(int new_mode);

void hal_exit(void);

#ifdef __cplusplus
}
#endif

#endif//SPINE_HAL_H
