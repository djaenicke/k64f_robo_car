#include "mbed.h"

#include "battery_monitor.h"
#include "config.h"
#include "io_abstraction.h"

#define PERCENT_TO_VOLTS (3.3f)
#define SCALING          ((R1+R2)/R2)

static AnalogIn ain(VBATT_ADC);

float ReadBatteryVoltage(void) {
  return (ain.read() * PERCENT_TO_VOLTS * SCALING);
}

