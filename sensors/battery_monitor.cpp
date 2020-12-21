#include "battery_monitor.h"
#include "config.h"
#include "io_abstraction.h"
#include "mbed.h"

static const float AIN_SCALING = 3.3f * ((R1 + R2) / R2);
static AnalogIn ain(VBATT_ADC);

float readBatteryVoltage(void)
{
  return (ain.read() * AIN_SCALING);
}
