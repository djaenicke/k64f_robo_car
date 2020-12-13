#include "hc_sr04.h"

namespace hc_sr04 {

static const int TRIGGER_PULSE_WIDTH_US = 10;
static const float PULSE_WIDTH_US_2_M = (1 / 5800.0f);

HC_SR04::HC_SR04(PinName trigger_pin, PinName echo_pin):
  trigger_(trigger_pin, false),
  echo_(echo_pin) {

  echo_.rise(Callback<void()>(this, &HC_SR04::EchoRisingEdgeISR));
  echo_.fall(Callback<void()>(this, &HC_SR04::EchoFallingEdgeISR));

  t_.start();
  state_ = RANGING_ERROR;
  echo_.enable_irq();
}

void HC_SR04::Trigger(void) {
  state_ = WAITING_FOR_ECHO_RISING_EDGE;

  trigger_ = true;
  const int trigger_start_t = t_.read_us();
  while ((t_.read_us() - trigger_start_t) <= TRIGGER_PULSE_WIDTH_US);
  trigger_ = false;
}

float HC_SR04::GetDist2Obj(void) {
  if (RANGING_COMPLETE == state_) {
    t_.reset();
    return (echo_pulse_end_t_us_ - echo_pulse_start_t_us_) * PULSE_WIDTH_US_2_M;
  }
  return 0.0f;
}

void HC_SR04::EchoRisingEdgeISR(void) {
  if (WAITING_FOR_ECHO_RISING_EDGE == state_) {
    state_ = WAITING_FOR_ECHO_FALLING_EDGE;
    echo_pulse_start_t_us_ = t_.read_us();
  } else {
    state_ = RANGING_ERROR;
  }
}

void HC_SR04::EchoFallingEdgeISR(void) {
  if (WAITING_FOR_ECHO_FALLING_EDGE == state_) {
    state_ = RANGING_COMPLETE;
    echo_pulse_end_t_us_ = t_.read_us();
  } else {
    state_ = RANGING_ERROR;
  }
}

}  // namespace hc_sr04
