#include "mbed.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"
#include "serial_ctrl.h"

static Serial debug_out(USBTX, USBRX, 115200);
static DigitalOut init_complete_led(LED_BLUE);

static Timeout print_timeout;
static volatile bool print_speeds;
static Wheel_Ang_V_T ang_v;

void set_print_speeds(void);
void go_forward(void);

void set_print_speeds(void) {
  print_speeds = true;
  print_timeout.attach(&set_print_speeds, 2.0);
}

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */
  init_complete_led.write(0);
  print_timeout.attach(&set_print_speeds, 2.0);

  InitMotorControls();
  Bluetooth_Serial_Init();

  while (true) {
    RunMotorControls();

    if (print_speeds) {
      GetWheelAngV(&ang_v);
      debug_out.printf("\r\nRight wheel angular velocity = %.2f\r\n", ang_v.r);
      debug_out.printf("Left angular velocity = %.2f\r\n", ang_v.l);
      print_speeds = false;
    }

    ThisThread::sleep_for(25);
  }
}

