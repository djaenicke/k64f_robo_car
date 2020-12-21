#ifndef CONFIG_H_
#define CONFIG_H_

#include "io_abstraction.h"
#include "macros.h"
#include "tb6612.h"

// Common constants
#define STATE_MSG_RATE_MS (50)
#define CYCLE_TIME_MS (25)
#define CYCLE_TIME_S (CYCLE_TIME_MS * 0.001f)

#define OPEN_LOOP (0)
#define ROS_ENABLED (1)

// Battery monitor
#define R1 (3.30f)   // 3.3k nominal
#define R2 (0.986f)  // 1.0k nominal

// Motor controller constants
#define R_MOTOR tb6612::MOTOR_A
#define L_MOTOR tb6612::MOTOR_B

#define MOTOR_A_POLARITY tb6612::REVERSED
#define MOTOR_B_POLARITY tb6612::STANDARD

#define R_Ke 0.419f
#define L_Ke 0.420f

#define PULSES_PER_REV (960)
#define MAX_MOTOR_VOLTAGE (12.0f)

#endif  // CONFIG_H_
