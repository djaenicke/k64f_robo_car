#include "dc_motor.h"
#include "io_abstraction.h"

namespace dc_motor {

static PwmOut l_pwm_(MOTOR_ENA);
static PwmOut r_pwm_(MOTOR_ENB);

static DigitalOut in1_(MOTOR_IN1);
static DigitalOut in2_(MOTOR_IN2);
static DigitalOut in3_(MOTOR_IN3);
static DigitalOut in4_(MOTOR_IN4);

DC_Motor::DC_Motor(Location_T new_loc) {
  /* Set the motor position on the robot (right side or left side) */
  switch (new_loc) {
    case LEFT_SIDE:
        loc_ = LEFT_SIDE;
        break;
    case RIGHT_SIDE:
        loc_ = RIGHT_SIDE;
        break;
    default:
        loc_ = UNKNOWN_LOC;
  }
}

void DC_Motor::SetDirection(Direction_T new_dir) {
  switch (new_dir) {
  case FORWARD:
    stopped = false;
    direction_ = FORWARD;
    switch (loc_) {
      case LEFT_SIDE:
        in1_ = 0;
        in2_ = 1;
        break;
      case RIGHT_SIDE:
        in3_ = 1;
        in4_ = 0;
        break;
      default:
        MBED_ASSERT(false);
    }
    break;
  case REVERSE:
    stopped = false;
    direction_ = REVERSE;
    switch (loc_) {
      case LEFT_SIDE:
        in1_ = 1;
        in2_ = 0;
        break;
      case RIGHT_SIDE:
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

Direction_T DC_Motor::GetDirection(void) {
  return (direction_);
}

void DC_Motor::SetDC(uint8_t percent) {
  switch (loc_) {
    case LEFT_SIDE:
      l_pwm_ = percent/100.0f;
      break;
    case RIGHT_SIDE:
      r_pwm_ = percent/100.0f;
      break;
    default:
      MBED_ASSERT(false);
  }
}

void DC_Motor::Stop(void) {
  stopped = true;

  switch (loc_) {
    case LEFT_SIDE:
      in1_ = 0;
      in2_ = 0;
      break;
    case RIGHT_SIDE:
      in3_ = 0;
      in4_ = 0;
      break;
    default:
      MBED_ASSERT(false);
  }
}

void DC_Motor::Freewheel(void) {
  stopped = false;
  SetDC(0);
}

}  // namespace dc_motor
