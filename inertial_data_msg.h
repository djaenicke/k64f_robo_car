#ifndef INERTIAL_DATA_MSG_H_
#define INERTIAL_DATA_MSG_H_

#include "mbed.h"

#define SCALING_RECIPROCAL (1000.0f)
#define OFFSET  (32.7675f)

#define ENCODE_SIGNAL(s) (uint16_t)(roundf((s + OFFSET) * SCALING_RECIPROCAL))

typedef struct {
  uint32_t ts;
  uint16_t l_speed_sp;
  uint16_t r_speed_sp;
  uint16_t l_speed_fb;
  uint16_t r_speed_fb;
  uint16_t fxos_ax;
  uint16_t fxos_ay;
  uint16_t fxos_az;
  uint16_t mpu_ax;
  uint16_t mpu_ay;
  uint16_t mpu_az;
  uint16_t mpu_gz;
} Inertial_Data_Msg_T;

#endif  // INERTIAL_DATA_MSG_H_
