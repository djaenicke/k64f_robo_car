#include "mbed.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"
#include "serial_ctrl.h"

#include "mpu6050.h"

static DigitalOut init_complete_led(LED_BLUE);

static Thread motor_controls_thread(osPriorityRealtime);

static mpu6050::MPU6050 imu(MPU6050_SDA, MPU6050_SCL);
static mpu6050::Accel_Data_T accel_data;

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */
  init_complete_led.write(0);

  InitMotorControls();
  Bluetooth_Serial_Init();
  imu.Init();

  /* Start the threads */
  motor_controls_thread.start(RunMotorControls);

  while (true) {
    imu.ReadAccelData(&accel_data);
    ThisThread::sleep_for(1000);
  }
}

