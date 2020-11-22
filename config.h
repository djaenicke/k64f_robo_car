#ifndef CONFIG_H_
#define CONFIG_H_

#include "io_abstraction.h"
#include "macros.h"
#include "tb6612.h"

/* Common constants */
#define ROBO_CAR_VERSION     (2)
#define OPEN_LOOP            (0)
#define STATE_MSG_RATE       (0.050f)
#define CYCLE_TIME           (0.025f)
#define TUNE                 (1)

#define ROS_ENABLED          (0)
#define IMUS_ENABLED         (1)

/* RoboCar version 1 constants */
#if (1 == ROBO_CAR_VERSION)

/* Battery monitor */
#define R1               (4.65f)  // 4.7k nominal
#define R2               (2.161f) // 2.2k nominal

/* Physical dimensions */

/* Motor controller constants */
#define R_MOTOR tb6612::MOTOR_A
#define L_MOTOR tb6612::MOTOR_B

#define MOTOR_A_POLARITY tb6612::STANDARD
#define MOTOR_B_POLARITY tb6612::STANDARD

#define R_Ke 0.255f
#define R_Kp 1.45f
#define R_Ki 3.5f
#define R_Kd 0.05f

#define L_Ke 0.215f
#define L_Kp 1.65f
#define L_Ki 3.9f
#define L_Kd 0.05f

#define PULSES_PER_REV         (192)
#define WHEEL_SPEED_FILT_ALPHA (0.2f)
#define MAX_MOTOR_VOLTAGE      (6.0f)

/* RoboCar version 2 constants */
#elif (2 == ROBO_CAR_VERSION)

/* Battery monitor */
#define R1               (3.30f)  // 3.3k nominal
#define R2               (0.986f) // 1.0k nominal

/* Physical dimensions */


/* Motor controller constants */
#define R_MOTOR tb6612::MOTOR_A
#define L_MOTOR tb6612::MOTOR_B

#define MOTOR_A_POLARITY tb6612::REVERSED
#define MOTOR_B_POLARITY tb6612::STANDARD

#define R_Ke 0.419f
#define R_Kp 0.735f
#define R_Ki 7.5f
#define R_Kd 0.00f

#define L_Ke 0.420f
#define L_Kp 0.735f
#define L_Ki 7.5f
#define L_Kd 0.00f

#define PULSES_PER_REV         (960)
#define WHEEL_SPEED_FILT_ALPHA (1.0f)  // Disabled with 1
#define MAX_MOTOR_VOLTAGE      (12.0f)
#else
  #error "Unknown RoboCar version!"
#endif // ROBO_CAR_VERSION

#endif /* CONFIG_H_ */
