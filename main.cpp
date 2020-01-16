#include "mbed.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"
#include "serial_ctrl.h"

#include "mpu6050.h"
#include "fxos8700.h"

#include <ros.h>
#include <std_msgs/String_ROS.h>

/* ROS objects/variables */
static ros::NodeHandle nh;
static std_msgs::String str_msg;
static ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

static DigitalOut red_led(LED_RED);
static DigitalOut green_led(LED_GREEN);
static DigitalOut blue_led(LED_BLUE);

static Thread motor_controls_thread(osPriorityRealtime);

static mpu6050::MPU6050 imu1(MPU6050_SDA, MPU6050_SCL);
static fxos8700::FXOS8700 imu2(FXOS8700_SDA, FXOS8700_SCL);

static mpu6050::Accel_Data_T mpu_accel_data;
static mpu6050::Gyro_Data_T  mpu_gyro_data;
static fxos8700::Sensor_Data_T fxos_data;

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */

  /* Green LED means init is in progress */
  red_led.write(1);
  green_led.write(0);
  blue_led.write(1);

  InitMotorControls();
  Bluetooth_Serial_Init();
  
  imu1.Init();
  imu2.Init();

  nh.initNode();
  nh.advertise(chatter);

  /* Blue LED means init was successful */
  red_led.write(1);
  green_led.write(1);
  blue_led.write(0);

  /* Start the threads */
  motor_controls_thread.start(RunMotorControls);

  while (true) {
    imu1.ReadAccelData(&mpu_accel_data);
    imu1.ReadGyroData(&mpu_gyro_data);
    imu2.ReadData(&fxos_data);

    chatter.publish(&str_msg);
    nh.spinOnce();

    ThisThread::sleep_for(1000);
  }
}

