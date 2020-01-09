#include "mbed.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"

static DigitalOut init_complete_led(LED_BLUE);
static Serial debug_out(USBTX, USBRX, 115200);
static InterruptIn sw2(SW2);

/* The following objects are here temporarily for testing */
static Timeout print_timeout;
static volatile bool print_speeds;
static Wheel_Ang_V_T ang_v;

void set_print_speeds(void);
void go_forward(void);

void set_print_speeds(void) {
  print_speeds = true;
  print_timeout.attach(&set_print_speeds, 2.0);
}

void go_forward(void) {
  static bool enabled = false;

  if (!enabled) {
    UpdateWheelAngV(15.0, 15.0, true);
    enabled = true;
  } else {
    StopMotors();
    enabled = false;
  }
}

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */
  debug_out.printf("Battery voltage = %.2f\r\n", ReadBatteryVoltage());
  init_complete_led.write(0);
  print_timeout.attach(&set_print_speeds, 2.0);
  sw2.rise(&go_forward);
  InitMotorControls();

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

