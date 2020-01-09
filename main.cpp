#include "mbed.h"
#include "l298.h"
#include "encoder.h"
#include "io_abstraction.h"
#include "battery_monitor.h"

/* Redefine motors ids to locations for code readability */
#define L_MOTOR l298::MOTOR_A
#define R_MOTOR l298::MOTOR_B

#define PULSES_PER_REV   (192)

static DigitalOut init_complete_led(LED_BLUE);
static Serial debug_out(USBTX, USBRX, 115200);

/* The following objects are here temporarily for testing */
static l298::L298 motor_driver(MOTOR_ENA, MOTOR_ENB, MOTOR_IN1, 
                               MOTOR_IN2, MOTOR_IN3, MOTOR_IN4);

static encoder::Encoder r_encoder(R_ENCODER, PullUp, PULSES_PER_REV, 50);
static encoder::Encoder l_encoder(L_ENCODER, PullUp, PULSES_PER_REV, 50);

static Timeout print_timeout;
static volatile bool print_speeds;
static float r_wheel_ang_v;
static float l_wheel_ang_v;

void set_print_speeds(void);

void set_print_speeds(void) {
  print_speeds = true;
  print_timeout.attach(&set_print_speeds, 2.0);
}

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */
  debug_out.printf("Battery voltage = %.2f\r\n", ReadBatteryVoltage());
  init_complete_led.write(0);

  motor_driver.SetDirection(L_MOTOR, l298::REVERSE);
  motor_driver.SetDirection(R_MOTOR, l298::FORWARD);

  motor_driver.SetDC(L_MOTOR, 75);
  motor_driver.SetDC(R_MOTOR, 75);

  print_timeout.attach(&set_print_speeds, 2.0);

  while (true) {
    r_wheel_ang_v = r_encoder.GetWheelSpeed() * motor_driver.GetDirection(R_MOTOR);
    l_wheel_ang_v = l_encoder.GetWheelSpeed() * motor_driver.GetDirection(L_MOTOR);

    if (print_speeds) {
      debug_out.printf("\r\nRight wheel angular velocity = %.2f\r\n", r_wheel_ang_v);
      debug_out.printf("Left angular velocity = %.2f\r\n", l_wheel_ang_v);
      print_speeds = false;
    }

    ThisThread::sleep_for(25);
  }
}

