#ifndef SENSORS_MPU6050_H_
#define SENSORS_MPU6050_H_

#include "mbed.h"

namespace mpu6050 {

typedef struct {
  float accel[3];
  float gyro[3];
} Meas_Biases_T;

typedef struct {
  float accel;
  float gyro;
} Meas_Scalings_T;

typedef struct {
  float ax;
  float ay;
  float az;
} Accel_Data_T;

typedef struct {
  float gx;
  float gy;
  float gz;
} Gyro_Data_T;

typedef enum {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
} Ascale_T;

typedef enum {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
} Gscale_T;

class MPU6050 {
 private:
  bool init_complete_;
  Ascale_T a_scale_;
  Gscale_T g_scale_;
  Meas_Biases_T biases_;
  Meas_Scalings_T scalings_;

  I2C i2c_;
  Timer t_;

  void Reset(void);
  void Delay(uint16_t delay_ms);
  void WriteByte(uint8_t sub_addr, uint8_t data);
  uint8_t ReadByte(uint8_t sub_addr);
  void ReadBytes(uint8_t sub_addr, uint8_t cnt, uint8_t * buffer);
  bool TestConnection(void);
  void Calibrate(void);
  void RunSelfTest(void);

 public:
  MPU6050(PinName sda, PinName scl, \
          Ascale_T ascale = AFS_2G, Gscale_T gscale = GFS_250DPS);
  void Init(void);
  void ReadGyroData(Gyro_Data_T * destination);
  void ReadAccelData(Accel_Data_T * destination);
};

}  // namespace mpu6050

#endif  // SENSORS_MPU6050_H_
