#include "mbed.h"
#include "battery_monitor.h"

static DigitalOut init_complete_led(LED_BLUE);
static Serial debug_out(USBTX, USBRX, 115200);

// main() runs in its own thread in the OS
int main()
{
  /* Initialization code */
  init_complete_led.write(1);

  while (true) 
  {
    debug_out.printf("Battery voltage = %.2f\r\n", ReadBatteryVoltage());
    ThisThread::sleep_for(1000);
  }
}

