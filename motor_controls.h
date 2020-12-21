#ifndef MOTOR_CONTROLS_H_
#define MOTOR_CONTROLS_H_

#include <oscar_pi/cmd.h>

typedef struct
{
  float r;
  float l;
} WheelAngV;

extern void initMotorControls(void);
extern void runMotorControls(void);
extern void updateMotorControllerInputs(const oscar_pi::cmd& cmd_msg);
extern void getWheelAngVSp(WheelAngV* dest);
extern void getWheelAngV(WheelAngV* dest);

#endif /* MOTOR_CONTROLS_H_ */
