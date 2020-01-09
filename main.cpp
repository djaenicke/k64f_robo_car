#include "mbed.h"
#include "dc_motor.h"
#include "encoder.h"
#include "io_abstraction.h"
#include "battery_monitor.h"

#define PULSES_PER_REV   (192)

static DigitalOut init_complete_led(LED_BLUE);
static Serial debug_out(USBTX, USBRX, 115200);

/* The following objects are here temporarily for testing */
static dc_motor::DC_Motor r_motor(dc_motor::RIGHT_SIDE);
static dc_motor::DC_Motor l_motor(dc_motor::LEFT_SIDE);

static encoder::Encoder r_encoder(R_ENCODER, PullUp, PULSES_PER_REV, 50);
static encoder::Encoder l_encoder(L_ENCODER, PullUp, PULSES_PER_REV, 50);

static Timeout print_timeout;
static volatile bool print_speeds;
static float r_wheel_speed;
static float l_wheel_speed;

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

  r_motor.SetDirection(dc_motor::FORWARD);
  r_motor.SetDC(50);

  l_motor.SetDirection(dc_motor::FORWARD);
  l_motor.SetDC(50);

  print_timeout.attach(&set_print_speeds, 2.0);

  while (true) {
    r_wheel_speed = r_encoder.GetWheelSpeed();
    l_wheel_speed = l_encoder.GetWheelSpeed();

    if (print_speeds) {
      debug_out.printf("\r\nRight wheel speed = %.2f\r\n", r_wheel_speed);
      debug_out.printf("Left wheel speed = %.2f\r\n", l_wheel_speed);
      print_speeds = false;
    }

    ThisThread::sleep_for(25);
  }
}

