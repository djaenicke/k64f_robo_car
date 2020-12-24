#include <oscar_pi/cmd.h>
#include <oscar_pi/state.h>
#include <ros.h>
#include <ros/time_ros.h>
#include <sensor_msgs/Range.h>

#include "battery_monitor.h"
#include "config.h"
#include "hc_sr04.h"
#include "io_abstraction.h"
#include "mbed.h"
#include "motor_controls.h"
#include "mpu6050.h"
#include "rgb_led.h"

// ROS objects/variables
static ros::NodeHandle nh;
static oscar_pi::state state_msg;
static sensor_msgs::Range fwd_uss_range_msg;
static ros::Publisher state_msg_pub("robot_state", &state_msg);
static ros::Publisher range_msg_pub("fwd_uss", &fwd_uss_range_msg);
static ros::Subscriber<oscar_pi::cmd> cmd_sub("robot_cmd", &updateMotorControllerInputs);

static rgb_led::RgbLed status_indicator(LED_RED, LED_GREEN, LED_BLUE);

static Thread motor_controls_thread(osPriorityRealtime);

// Sensor objects/variables
static mpu6050::MPU6050 imu(I2C_SDA, I2C_SCL, mpu6050::AFS_2G, mpu6050::GFS_250DPS);
static hc_sr04::HC_SR04 fwd_uss(FWD_USS_TRIGGER, FWD_USS_ECHO);
static mpu6050::AccelData mpu_accel_data;
static mpu6050::GyroData mpu_gyro_data;
static WheelAngV wheel_speed_sp;
static WheelAngV wheel_speed_fb;

static const uint32_t IMU_CALIBRATION_DELAY_MS = 2000;
static const float LOW_VOLTAGE_THRESHOLD = 11.0;

// Static function declarations
static void populateStateMsg(void);

int main()
{
  // Yellow means init is in progress
  status_indicator.setColor(rgb_led::YELLOW);

  initMotorControls();

  // Wait 2 seconds before calibrating the IMUs so
  // the user doesn't affect the calibration process by touching the robot
  ThisThread::sleep_for(IMU_CALIBRATION_DELAY_MS);

  imu.init();

#if ROS_ENABLED
  nh.initNode();
  nh.advertise(state_msg_pub);
  nh.advertise(range_msg_pub);
  nh.subscribe(cmd_sub);
#endif

  // Green means init was successful
  status_indicator.setColor(rgb_led::GREEN);

  // Initialize the range message
  fwd_uss_range_msg.header.frame_id = "fwd_uss";
  fwd_uss_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  fwd_uss_range_msg.field_of_view = hc_sr04::HC_SR04::FOV_RAD;
  fwd_uss_range_msg.min_range = hc_sr04::HC_SR04::MIN_RANGE_M;
  fwd_uss_range_msg.max_range = hc_sr04::HC_SR04::MAX_RANGE_M;

  // Start the motor controls thread
  motor_controls_thread.start(runMotorControls);

  while (true)
  {
    fwd_uss_range_msg.range = fwd_uss.getDist2Obj();
    fwd_uss.trigger();
    populateStateMsg();

#if ROS_ENABLED
    fwd_uss_range_msg.header.stamp = nh.now();
    range_msg_pub.publish(&fwd_uss_range_msg);
    fwd_uss_range_msg.header.seq++;

    state_msg.header.stamp = nh.now();
    state_msg_pub.publish(&state_msg);
    state_msg.header.seq++;

    nh.spinOnce();
#endif
    ThisThread::sleep_for(STATE_MSG_RATE_MS);
  }
}

void populateStateMsg(void)
{
  state_msg.vbatt = readBatteryVoltage();

  if (state_msg.vbatt > LOW_VOLTAGE_THRESHOLD)
  {
    status_indicator.setColor(rgb_led::GREEN);
  }
  else
  {
    status_indicator.setColor(rgb_led::YELLOW);
  }

  getWheelAngVSp(&wheel_speed_sp);  // Setpoint
  state_msg.l_wheel_sp = wheel_speed_sp.l;
  state_msg.r_wheel_sp = wheel_speed_sp.r;

  getWheelAngV(&wheel_speed_fb);  // Feedback
  state_msg.l_wheel_fb = wheel_speed_fb.l;
  state_msg.r_wheel_fb = wheel_speed_fb.r;

  imu.readAccelData(&mpu_accel_data);
  state_msg.mpu_ax = -1.0F * mpu_accel_data.ax;
  state_msg.mpu_ay = -1.0F * mpu_accel_data.ay;
  state_msg.mpu_az = mpu_accel_data.az;

  imu.readGyroData(&mpu_gyro_data);
  state_msg.mpu_gx = mpu_gyro_data.gx;
  state_msg.mpu_gy = mpu_gyro_data.gy;
  state_msg.mpu_gz = mpu_gyro_data.gz;
}
