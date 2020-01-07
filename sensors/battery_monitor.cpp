#include "battery_monitor.h"
#include "io_abstraction.h"
#include "mbed.h"

#define PERCENT_TO_VOLTS (3.3f)
#define R1               (4.65f)
#define R2               (2.161f)
#define SCALING          ((R1+R2)/R2)

static AnalogIn ain(VBATT_ADC);

float ReadBatteryVoltage(void) {
  return (ain.read() * PERCENT_TO_VOLTS * SCALING);
}

