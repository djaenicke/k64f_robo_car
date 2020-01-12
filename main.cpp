#include "mbed.h"
#include "io_abstraction.h"
#include "battery_monitor.h"
#include "motor_controls.h"
#include "serial_ctrl.h"

#include "mpu6050.h"
#include "fxos8700.h"
#include "ESP8266Interface.h"

static DigitalOut red_led(LED_RED);
static DigitalOut green_led(LED_GREEN);
static DigitalOut blue_led(LED_BLUE);

static Thread motor_controls_thread(osPriorityRealtime);

static ESP8266Interface wifi(ESP8266_TX, ESP8266_RX);
static mpu6050::MPU6050 imu1(MPU6050_SDA, MPU6050_SCL);
static fxos8700::FXOS8700 imu2(FXOS8700_SDA, FXOS8700_SCL);

static mpu6050::Accel_Data_T mpu_accel_data;
static mpu6050::Gyro_Data_T  mpu_gyro_data;
static fxos8700::Sensor_Data_T fxos_data;

// main() runs in its own thread in the OS
int main() {
  UDPSocket socket;
  SocketAddress socket_addr(ROS_SERVER_IP, ROS_SERVER_PORT);
  nsapi_error_t ret;
  int wifi_connect_status;

  /* Initialization code */

  /* Green LED means init is in progress */
  red_led.write(1);
  green_led.write(0);
  blue_led.write(1);

  wifi_connect_status = wifi.connect(ROS_NETWORK_SSID, ROS_NETWORK_PASSWORD, ROS_NETWORK_SECURITY_TYPE);

  if (0 != wifi_connect_status) {
    /* Red LED means wifi didn't connect */
    red_led.write(0);
    green_led.write(1);
    blue_led.write(1);
    while(1);
  }

  ret = socket.open(&wifi);
  MBED_ASSERT(NSAPI_ERROR_OK == ret);

  ret = socket.connect(socket_addr);
  MBED_ASSERT(NSAPI_ERROR_OK == ret);

  InitMotorControls();
  Bluetooth_Serial_Init();
  
  imu1.Init();
  imu2.Init();

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
    socket.send("Hello!", strlen("Hello!"));
    ThisThread::sleep_for(1000);
  }
}

