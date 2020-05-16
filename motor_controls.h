#ifndef MOTOR_CONTROLS_H_
#define MOTOR_CONTROLS_H_

typedef struct {
  float r;
  float l;
} Wheel_Ang_V_T;

extern void InitMotorControls(void);
extern void RunMotorControls(void);
extern void UpdateWheelAngV(Wheel_Ang_V_T *sp, bool reset_pid);
extern void StopMotors(void);
extern void GetWheelAngVSp(Wheel_Ang_V_T *dest);
extern void GetWheelAngV(Wheel_Ang_V_T *dest);

#endif /* MOTOR_CONTROLS_H_ */
