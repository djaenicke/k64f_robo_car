#ifndef CONFIG_H_
#define CONFIG_H_

#include "io_abstraction.h"

/* Common constants */
#define ROBO_CAR_VERSION     (2)
#define OPEN_LOOP            (0)
#define CYCLE_TIME           (0.025f)
#define TUNE                 (1)

#define ROS_ENABLED          (1)

#if 1 == ROBO_CAR_VERSION

/* Battery monitor */
#define R1               (4.65f)  // 4.7k nominal
#define R2               (2.161f) // 2.2k nominal

/* Physical dimensions */

/* Motor controller constants */
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

#elif 2 == ROBO_CAR_VERSION

/* Battery monitor */
#define R1               (3.30f)  // 3.3k nominal
#define R2               (0.986f) // 1.0k nominal

/* Physical dimensions */


/* Motor controller constants */
#define R_Ke 0.419f
#define R_Kp 0.735f
#define R_Ki 7.5f
#define R_Kd 0.00f

#define L_Ke 0.420f
#define L_Kp 0.735f
#define L_Ki 7.5f
#define L_Kd 0.00f

#define PULSES_PER_REV         (240)
#define WHEEL_SPEED_FILT_ALPHA (0.2f)
#define MAX_MOTOR_VOLTAGE      (12.0f)
#else

#endif

#endif /* CONFIG_H_ */
