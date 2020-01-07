#ifndef SENSORS_ENCODER_H_
#define SENSORS_ENCODER_H_

#include "mbed.h"

namespace encoder {

#define MAX_MEASUREMENTS 50

typedef struct {
  uint8_t  meas_type;
  uint8_t  num_meas;
  int start;
  int meas[MAX_MEASUREMENTS];
} Period_T;

class Encoder {
 public:
  Encoder(PinName pin, PinMode mode, int pulses_per_rev);
  ~Encoder(void);
  void Init(void);
  float GetWheelSpeed(void);
  void ZeroWheelSpeed(void);

 private:
  Timer t_;
  PinName pin_;
  PinMode mode_;
  int pulses_per_rev_;
  Period_T period_;
  InterruptIn * this_interrupt_;
  void MeasurePeriod(void);
  void ResetPeriodMeas(void);
};

}   // namespace encoder

#endif  // SENSORS_ENCODER_H_
