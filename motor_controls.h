#ifndef MOTOR_CONTROLS_H_
#define MOTOR_CONTROLS_H_

#include <oscar_pi/cmd.h>

typedef struct {
  float r;
  float l;
} Wheel_Ang_V_T;

extern void InitMotorControls(void);
extern void RunMotorControls(void);
extern void UpdateMotorControllerInputs(const oscar_pi::cmd& cmd_msg);
extern void StopMotors(void);
extern void GetWheelAngVSp(Wheel_Ang_V_T *dest);
extern void GetWheelAngV(Wheel_Ang_V_T *dest);

#endif /* MOTOR_CONTROLS_H_ */
