#include "encoder.h"
#include "io_abstraction.h"

namespace encoder {

#define RAD_PER_REV      (6.2831853f)
#define START            ((uint8_t)0)
#define END              ((uint8_t)1)

Encoder::Encoder(PinName pin, PinMode mode, int pulses_per_rev) {
  pulses_per_rev_ = pulses_per_rev;
  period_.meas_type = START;
  t_.start();

  this_interrupt_ = new InterruptIn(pin, mode);

  if (this_interrupt_) // Null pointer check
  {
    this_interrupt_->rise(Callback<void()>(this, &Encoder::MeasurePeriod));
    this_interrupt_->enable_irq();
  }
}

Encoder::~Encoder(void) {
  delete this_interrupt_;
}

void Encoder::MeasurePeriod(void) {
  if (START == period_.meas_type) {
    period_.start = t_.read_us();
    period_.meas_type = END;
  } else {
    period_.meas[period_.num_meas] = t_.read_us() - period_.start;
    period_.num_meas = period_.num_meas < (MAX_MEASUREMENTS - 1) ? \
                       period_.num_meas + 1 : 0;
    period_.meas_type = START;
  }
}

void Encoder::ResetPeriodMeas(void) {
  period_.meas_type = START;
  period_.num_meas = 0;
}

float Encoder::GetWheelSpeed(void) {
  float period_sum, period_avg, speed;

  this_interrupt_->disable_irq();

  /* Compute the average period since last call to ResetPeriodMeas */
  for (uint8_t i=0; i < period_.num_meas; i++) {
    period_sum += period_.meas[i];
  }

  if (0 != period_.num_meas) {  //  Protect against divide by 0
    period_avg = period_sum/period_.num_meas;
    speed = RAD_PER_REV/(pulses_per_rev_*period_avg*1e-6);
  }

  ResetPeriodMeas();

  this_interrupt_->enable_irq();

  return(speed);
}

void Encoder::ZeroWheelSpeed(void) {
  ResetPeriodMeas();
}

}   // namespace encoder

