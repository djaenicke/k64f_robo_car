#include "mbed.h"
#include "dc_motor.h"
#include "battery_monitor.h"

static DigitalOut init_complete_led(LED_BLUE);
static Serial debug_out(USBTX, USBRX, 115200);

static DC_Motor r_motor(RIGHT_SIDE);
static DC_Motor l_motor(LEFT_SIDE);

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */
  debug_out.printf("Battery voltage = %.2f\r\n", ReadBatteryVoltage());
  init_complete_led.write(0);

  while (true) {
    ThisThread::sleep_for(1000);
  }
}

