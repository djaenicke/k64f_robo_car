#include "mbed.h"

#include "config.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"

#include "mpu6050.h"
#include "fxos8700.h"

#include <ros.h>

static Serial pcdebug(USBTX, USBRX, 115200);

static Timer t;
static int t_start;

/* ROS objects/variables */
static ros::NodeHandle nh;

static DigitalOut red_led(LED_RED);
static DigitalOut green_led(LED_GREEN);
static DigitalOut blue_led(LED_BLUE);

static Thread motor_controls_thread(osPriorityRealtime);

static mpu6050::MPU6050 imu1(I2C_SDA, I2C_SCL);
static fxos8700::FXOS8700 imu2(I2C_SDA, I2C_SCL);

Wheel_Ang_V_T wheel_speed_sp;
Wheel_Ang_V_T wheel_speed_fb;
mpu6050::Accel_Data_T mpu_accel_data;
mpu6050::Gyro_Data_T  mpu_gyro_data;
fxos8700::Sensor_Data_T fxos_data;

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */

  /* Green LED means init is in progress */
  red_led.write(1);
  green_led.write(0);
  blue_led.write(1);

  InitMotorControls();

  /* Wait 2 seconds before calibrating the IMUs so 
     the user doesn't affect the calibration process by touching the robot*/
  ThisThread::sleep_for(2000);

  imu1.Init();
  imu2.Init();

#if ROS_ENABLED
  nh.initNode();
#endif

  /* Blue LED means init was successful */
  red_led.write(1);
  green_led.write(1);
  blue_led.write(0);

  /* Start the threads */
  motor_controls_thread.start(RunMotorControls);

  t.start();

  while (true) {
    t_start = t.read_ms();

    GetWheelAngVSp(&wheel_speed_sp); /* Setpoint */
    GetWheelAngV(&wheel_speed_fb);   /* Feedback */

    imu2.ReadData(&fxos_data);
    //fxos_ax = -1.0f * fxos_data.ay;
    //fxos_ay = fxos_data.ax;

    imu1.ReadAccelData(&mpu_accel_data);
    //mpu_ax = -1.0f * mpu_accel_data.ax;
    //mpu_ay = -1.0f * mpu_accel_data.ay;

    imu1.ReadGyroData(&mpu_gyro_data);

#if ROS_ENABLED
    nh.getHardware()->get_recv_data();
    nh.spinOnce();
#endif
    ThisThread::sleep_for(50-(t.read_ms()-t_start));
  }
}
