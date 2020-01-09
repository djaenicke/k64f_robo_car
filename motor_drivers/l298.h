#ifndef MOTOR_DRIVERS_L298_H_
#define MOTOR_DRIVERS_L298_H_

#include "mbed.h"

namespace l298 {

static const float vdrop = 2.0;

typedef enum {
  REVERSE = -1,
  UNKNOWN_DIR = 0,
  FORWARD = 1
} Direction_T;

typedef enum {
  MOTOR_A = 0,
  MOTOR_B
} Motor_Id_T;

class L298 {
 public:
  L298(PinName en_a, PinName en_b, PinName in1, \
       PinName in2, PinName in3, PinName in4);
  void SetPWMPeriod(int period_ms);
  void SetDirection(Motor_Id_T motor, Direction_T new_dir);
  Direction_T GetDirection(Motor_Id_T motor);
  void SetDC(Motor_Id_T motor, uint8_t percent);
  void Stop(Motor_Id_T motor);
  void Freewheel(void);

 private:
  DigitalOut in1_;
  DigitalOut in2_;
  DigitalOut in3_;
  DigitalOut in4_;
  PwmOut en_a_;
  PwmOut en_b_;
  Direction_T motor_a_dir_;
  Direction_T motor_b_dir_;
};

}  // namespace l298

#endif  // MOTOR_DRIVERS_L298_H_
