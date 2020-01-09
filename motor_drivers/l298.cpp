#include "l298.h"

namespace l298 {

L298::L298(PinName en_a, PinName en_b, PinName in1, \
           PinName in2, PinName in3, PinName in4):
           en_a_(en_a), en_b_(en_b), in1_(in1),
           in2_(in2), in3_(in3), in4_(in4) {
}

void L298::SetPWMPeriod(int period_ms) {
    en_a_.period_ms(period_ms);
    en_b_.period_ms(period_ms);
}

void L298::SetDirection(Motor_Id_T motor, Direction_T new_dir) {
  switch (new_dir) {
  case FORWARD:
    switch (motor) {
      case MOTOR_A:
        motor_a_dir_ = FORWARD;
        in1_ = 0;
        in2_ = 1;
        break;
      case MOTOR_B:
        motor_b_dir_ = FORWARD;
        in3_ = 1;
        in4_ = 0;
        break;
      default:
        MBED_ASSERT(false);
    }
    break;
  case REVERSE:
    switch (motor) {
      case MOTOR_A:
        motor_a_dir_ = REVERSE;
        in1_ = 1;
        in2_ = 0;
        break;
      case MOTOR_B:
        motor_b_dir_ = REVERSE;
        in3_ = 0;
        in4_ = 1;
        break;
      default:
        MBED_ASSERT(false);
    }
    break;
      default:
      MBED_ASSERT(false);
  }
}

Direction_T L298::GetDirection(Motor_Id_T motor) {
  Direction_T dir;

  switch (motor) {
    case MOTOR_A:
      dir = motor_a_dir_;
      break;
    case MOTOR_B:
      dir = motor_b_dir_;
      break;
    default:
      MBED_ASSERT(false);
  }

  return (dir);
}

void L298::SetDC(Motor_Id_T motor, uint8_t percent) {
  switch (motor) {
    case MOTOR_A:
      en_a_ = percent/100.0f;
      break;
    case MOTOR_B:
      en_b_ = percent/100.0f;
      break;
    default:
      MBED_ASSERT(false);
  }
}

void L298::Stop(Motor_Id_T motor) {
  switch (motor) {
    case MOTOR_A:
      in1_ = 0;
      in2_ = 0;
      break;
    case MOTOR_B:
      in3_ = 0;
      in4_ = 0;
      break;
    default:
      MBED_ASSERT(false);
  }
}

void L298::Freewheel(void) {
  SetDC(MOTOR_A, 0);
  SetDC(MOTOR_B, 0);
}

}  // namespace l298
