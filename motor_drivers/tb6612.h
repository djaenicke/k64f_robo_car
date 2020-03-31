#ifndef MOTOR_DRIVERS_TB6612_H_
#define MOTOR_DRIVERS_TB6612_H_

#include "mbed.h"

namespace tb6612 {

static const float vdrop = 0.1;

typedef enum {
  REVERSE = -1,
  UNKNOWN_DIR = 0,
  FORWARD = 1
} Direction_T;

typedef enum {
  MOTOR_A = 0,
  MOTOR_B
} Motor_Id_T;

class TB6612 {
 public:
  TB6612(PinName pwm_a, PinName pwm_b, PinName a_in1, \
         PinName a_in2, PinName b_in1, PinName b_in2);
  void SetPWMPeriod(float period_s);
  void SetDirection(Motor_Id_T motor, Direction_T new_dir);
  Direction_T GetDirection(Motor_Id_T motor);
  void SetDC(Motor_Id_T motor, uint8_t percent);
  void Stop(Motor_Id_T motor);
  void Freewheel(void);

 private:
  DigitalOut a_in1_;
  DigitalOut a_in2_;
  DigitalOut b_in1_;
  DigitalOut b_in2_;
  PwmOut pwm_a_;
  PwmOut pwm_b_;
  Direction_T motor_a_dir_;
  Direction_T motor_b_dir_;
};

}  // namespace tb6612

#endif  // MOTOR_DRIVERS_TB6612_H_
