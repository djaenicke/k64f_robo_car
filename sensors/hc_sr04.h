#ifndef SENSORS_HC_SR04_H_
#define SENSORS_HC_SR04_H_

#include "mbed.h"

namespace hc_sr04 {

typedef enum {
  RANGING_ERROR,
  WAITING_FOR_ECHO_RISING_EDGE,
  WAITING_FOR_ECHO_FALLING_EDGE,
  RANGING_COMPLETE,
} HC_SR04_State_T;

class HC_SR04 {
 private:
  HC_SR04_State_T state_;
  int echo_pulse_start_t_us_;
  int echo_pulse_end_t_us_;
  Timer t_;
  DigitalOut trigger_;
  InterruptIn echo_;

  void EchoRisingEdgeISR(void);
  void EchoFallingEdgeISR(void);

 public:
  HC_SR04(PinName trigger_pin, PinName echo_pin);
  void Trigger(void);
  float GetDist2Obj(void);
};

}  // namespace hc_sr04

#endif  // SENSORS_HC_SR04_H_
