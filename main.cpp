#include "mbed.h"

#include "config.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"
#include "serial_ctrl.h"

#include "mpu6050.h"
#include "fxos8700.h"

#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

static Serial pcdebug(USBTX, USBRX, 115200);

static Timer t;
static int t_start;

/* ROS objects/variables */
static ros::NodeHandle nh;

static sensor_msgs::Imu imu_msg_mpu;
static sensor_msgs::Imu imu_msg_fxos;
static nav_msgs::Odometry odo_msg;

static ros::Publisher imu_mpu("imu_data_mpu", &imu_msg_mpu);
static ros::Publisher imu_fxos("imu_data_fxos", &imu_msg_fxos);
static ros::Publisher odo("odo_data", &odo_msg);

static DigitalOut red_led(LED_RED);
static DigitalOut green_led(LED_GREEN);
static DigitalOut blue_led(LED_BLUE);

static Thread motor_controls_thread(osPriorityRealtime);

static mpu6050::MPU6050 imu1(I2C_SDA, I2C_SCL);
static fxos8700::FXOS8700 imu2(I2C_SDA, I2C_SCL);

static char log_buffer[200];
static int last_t = 0, current_t = 0, dt = 0;

static void PopulateImuMsgs(void);

// main() runs in its own thread in the OS
int main() {
  /* Initialization code */

  /* Green LED means init is in progress */
  red_led.write(1);
  green_led.write(0);
  blue_led.write(1);

  InitMotorControls();
  InitSerialCtrl();

  imu1.Init();
  imu2.Init();

#if ROS_ENABLED
  nh.initNode();

  MBED_ASSERT(nh.advertise(imu_mpu));
  MBED_ASSERT(nh.advertise(imu_fxos));
  MBED_ASSERT(nh.advertise(odo));
#endif

  /* Configure the different frame ids */
  imu_msg_mpu.header.frame_id = "odom";
  imu_msg_fxos.header.frame_id = "odom";
  odo_msg.header.frame_id = "odom";

  /* Blue LED means init was successful */
  red_led.write(1);
  green_led.write(1);
  blue_led.write(0);

  /* Start the threads */
  motor_controls_thread.start(RunMotorControls);

  t.start();

  while (true) {
    t_start = t.read_ms();

    PopulateImuMsgs();
#if ROS_ENABLED
    imu_mpu.publish(&imu_msg_mpu);
    imu_fxos.publish(&imu_msg_fxos);
    odo.publish(&odo_msg);

    nh.getHardware()->get_recv_data();
    nh.spinOnce();
#endif
    current_t = t.read_us();
    dt = current_t-last_t;
    last_t = current_t;

    sprintf(log_buffer, "%d,%.2f,%.2f,%.2f,%.2f,%.2f", dt, \
                                                     imu_msg_mpu.angular_velocity.z,     \
                                                     imu_msg_mpu.linear_acceleration.x,  \
                                                     imu_msg_fxos.linear_acceleration.x, \
                                                     imu_msg_mpu.linear_acceleration.y,  \
                                                     imu_msg_fxos.linear_acceleration.y  \

    );
#if USE_XBEE
    XbeeProcessRxData();
    XbeeTxData(log_buffer, strlen(log_buffer));
#endif
    ThisThread::sleep_for(50-(t.read_ms()-t_start));
  }
}

void PopulateImuMsgs(void) {
  mpu6050::Accel_Data_T mpu_accel_data;
  mpu6050::Gyro_Data_T  mpu_gyro_data;
  fxos8700::Sensor_Data_T fxos_data;

  imu_msg_mpu.header.stamp = nh.now();
  imu_msg_fxos.header.stamp = imu_msg_mpu.header.stamp;

  imu1.ReadAccelData(&mpu_accel_data);
  imu_msg_mpu.linear_acceleration.x = -1.0f * mpu_accel_data.ax;
  imu_msg_mpu.linear_acceleration.y = -1.0f * mpu_accel_data.ay;
  imu_msg_mpu.linear_acceleration.z = mpu_accel_data.az;

  imu1.ReadGyroData(&mpu_gyro_data);
  imu_msg_mpu.angular_velocity.x = mpu_gyro_data.gx;
  imu_msg_mpu.angular_velocity.y = mpu_gyro_data.gy;
  imu_msg_mpu.angular_velocity.z = mpu_gyro_data.gz;

  imu2.ReadData(&fxos_data);
  imu_msg_fxos.linear_acceleration.x = -1.0f * fxos_data.ay;
  imu_msg_fxos.linear_acceleration.y = fxos_data.ax;
  imu_msg_fxos.linear_acceleration.z = fxos_data.az;
}
