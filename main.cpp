#include "mbed.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"
#include "serial_ctrl.h"

#include "mpu6050.h"
#include "fxos8700.h"

static DigitalOut init_complete_led(LED_BLUE);

static Thread motor_controls_thread(osPriorityRealtime);

static mpu6050::MPU6050 imu1(MPU6050_SDA, MPU6050_SCL);
static fxos8700::FXOS8700 imu2(FXOS8700_SDA, FXOS8700_SCL);

static mpu6050::Accel_Data_T mpu_accel_data;
static mpu6050::Gyro_Data_T  mpu_gyro_data;
static fxos8700::Sensor_Data_T fxos_data;

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */
  init_complete_led.write(0);

  InitMotorControls();
  Bluetooth_Serial_Init();
  
  imu1.Init();
  imu2.Init();

  /* Start the threads */
  motor_controls_thread.start(RunMotorControls);

  while (true) {
    imu1.ReadAccelData(&mpu_accel_data);
    imu1.ReadGyroData(&mpu_gyro_data);
    imu2.ReadData(&fxos_data);
    ThisThread::sleep_for(1000);
  }
}

