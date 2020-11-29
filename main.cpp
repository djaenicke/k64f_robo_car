#include "mbed.h"

#include "config.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"

#include "mpu6050.h"
#include "fxos8700.h"

#include <ros.h>
#include <oscar_pi/state.h>
#include <oscar_pi/cmd.h>

static Timer t;
static int t_start;

/* ROS objects/variables */
#if ROS_ENABLED
static ros::NodeHandle nh;
#endif
static oscar_pi::state state_msg;
static oscar_pi::cmd cmd_msg;

void Cmd_Msg_Callback(const oscar_pi::cmd& msg);

static ros::Publisher state_msg_pub("robot_state", &state_msg);
static ros::Subscriber<oscar_pi::cmd> state_cmd_sub("robot_cmd", &Cmd_Msg_Callback);
/* End - ROS objects/variables */

static DigitalOut red_led(LED_RED);
static DigitalOut green_led(LED_GREEN);
static DigitalOut blue_led(LED_BLUE);

#if TUNE
static InterruptIn go_button(SW2);
static InterruptIn stop_button(SW3);
#endif

static Thread motor_controls_thread(osPriorityRealtime);

#if IMUS_ENABLED
static mpu6050::MPU6050 imu1(I2C_SDA, I2C_SCL);
static fxos8700::FXOS8700 imu2(I2C_SDA, I2C_SCL);
#endif

Wheel_Ang_V_T wheel_speed_sp;
Wheel_Ang_V_T wheel_speed_fb;
mpu6050::Accel_Data_T mpu_accel_data;
mpu6050::Gyro_Data_T  mpu_gyro_data;
fxos8700::Sensor_Data_T fxos_data;

static void Populate_State_Msg(void);
#if TUNE
static void Go(void);
#endif

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */

  /* Green LED means init is in progress */
  red_led.write(1);
  green_led.write(0);
  blue_led.write(1);

  InitMotorControls();

  /* Wait 2 seconds before calibrating the IMUs so 
     the user doesn't affect the calibration process by touching the robot */
  ThisThread::sleep_for(2000);

#if IMUS_ENABLED
  imu1.Init();
  imu2.Init();
#endif

#if ROS_ENABLED
  nh.initNode();
  nh.advertise(state_msg_pub);
  nh.subscribe(state_cmd_sub);
#endif

  /* Blue LED means init was successful */
  red_led.write(1);
  green_led.write(1);
  blue_led.write(0);

#if TUNE
  go_button.rise(&Go);
  stop_button.rise(&StopMotors);
#endif

  /* Start the threads */
  motor_controls_thread.start(RunMotorControls);

  t.start();

  while (true) {
    if ((t.read_ms() - t_start) >= STATE_MSG_RATE_MS)
    {
      t_start += STATE_MSG_RATE_MS;
#if ROS_ENABLED
      Populate_State_Msg();
      state_msg_pub.publish(&state_msg);
#endif
    }
#if ROS_ENABLED
    nh.spinOnce();
#endif
  }
}

void Populate_State_Msg(void) {
    state_msg.timestamp = t.read_us();
    state_msg.vbatt = ReadBatteryVoltage();

    GetWheelAngVSp(&wheel_speed_sp); /* Setpoint */
    state_msg.l_wheel_sp = wheel_speed_sp.l;
    state_msg.r_wheel_sp = wheel_speed_sp.r;

    GetWheelAngV(&wheel_speed_fb);   /* Feedback */
    state_msg.l_wheel_fb = wheel_speed_fb.l;
    state_msg.r_wheel_fb = wheel_speed_fb.r;

#if IMUS_ENABLED
    imu2.ReadData(&fxos_data);
    state_msg.fxos_ax = -1.0f * fxos_data.ay;
    state_msg.fxos_ay = fxos_data.ax;
    state_msg.fxos_az = fxos_data.az;
    state_msg.fxos_mx = fxos_data.mx;
    state_msg.fxos_my = fxos_data.my;
    state_msg.fxos_mz = fxos_data.mz;

    imu1.ReadAccelData(&mpu_accel_data);
    state_msg.mpu_ax = -1.0f * mpu_accel_data.ax;
    state_msg.mpu_ay = -1.0f * mpu_accel_data.ay;
    state_msg.mpu_az = mpu_accel_data.az;

    imu1.ReadGyroData(&mpu_gyro_data);
    state_msg.mpu_gx = mpu_gyro_data.gx;
    state_msg.mpu_gy = mpu_gyro_data.gy;
    state_msg.mpu_gz = mpu_gyro_data.gz;
#endif
}

void Cmd_Msg_Callback(const oscar_pi::cmd& msg) {
  Wheel_Ang_V_T sp;
  if (0 == msg.stop) {
    sp.r = msg.r_wheel_sp;
    sp.l = msg.l_wheel_sp;
    UpdateWheelAngV(&sp, true);
  } else {
    StopMotors();
  }
}

#if TUNE
void Go(void) {
  Wheel_Ang_V_T sp;
  sp.r = 15.0f;
  sp.l = 15.0f;
  UpdateWheelAngV(&sp, true);
}
#endif
