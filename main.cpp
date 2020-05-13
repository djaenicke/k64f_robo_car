#include "mbed.h"

#include "config.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"
#include "serial_ctrl.h"
#include "inertial_data_msg.h"

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

static Inertial_Data_Msg_T Inertial_Data_Msg;

static void PopulateImuMsgs(void);
static void LogInertialData(void);

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */

  /* Green LED means init is in progress */
  red_led.write(1);
  green_led.write(0);
  blue_led.write(1);

  InitMotorControls();
  InitSerialCtrl();

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

#if ROS_ENABLED
    nh.getHardware()->get_recv_data();
    nh.spinOnce();
#endif
#if USE_XBEE
    XbeeProcessRxData();
    LogInertialData();
#endif
    ThisThread::sleep_for(50-(t.read_ms()-t_start));
  }
}

void LogInertialData(void) {
  Wheel_Ang_V_T speeds;
  mpu6050::Accel_Data_T mpu_accel_data;
  mpu6050::Gyro_Data_T  mpu_gyro_data;
  fxos8700::Sensor_Data_T fxos_data;

  Inertial_Data_Msg.ts = t.read_us();

  GetWheelAngVSp(&speeds); /* Setpoint */
  Inertial_Data_Msg.l_speed_sp = ENCODE_SIGNAL(speeds.l);
  Inertial_Data_Msg.r_speed_sp = ENCODE_SIGNAL(speeds.r);

  GetWheelAngV(&speeds); /* Feedback */
  Inertial_Data_Msg.l_speed_fb = ENCODE_SIGNAL(speeds.l);
  Inertial_Data_Msg.r_speed_fb = ENCODE_SIGNAL(speeds.r);

  imu2.ReadData(&fxos_data);
  Inertial_Data_Msg.fxos_ax = ENCODE_SIGNAL(-1.0f * fxos_data.ay);
  Inertial_Data_Msg.fxos_ay = ENCODE_SIGNAL(fxos_data.ax);
  Inertial_Data_Msg.fxos_az = ENCODE_SIGNAL(fxos_data.az);

  imu1.ReadAccelData(&mpu_accel_data);
  Inertial_Data_Msg.mpu_ax = ENCODE_SIGNAL(-1.0f * mpu_accel_data.ax);
  Inertial_Data_Msg.mpu_ay = ENCODE_SIGNAL(-1.0f * mpu_accel_data.ay);
  Inertial_Data_Msg.mpu_az = ENCODE_SIGNAL(mpu_accel_data.az);

  imu1.ReadGyroData(&mpu_gyro_data);
  Inertial_Data_Msg.mpu_gz = ENCODE_SIGNAL(mpu_gyro_data.gz);

  XbeeTxData((char*)(&Inertial_Data_Msg), sizeof(Inertial_Data_Msg));
}
