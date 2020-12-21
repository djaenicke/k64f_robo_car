#ifndef SENSORS_MPU6050_H_
#define SENSORS_MPU6050_H_

#include "mbed.h"

namespace mpu6050
{
typedef struct
{
  float accel[3];
  float gyro[3];
} MeasBias;

typedef struct
{
  float accel;
  float gyro;
} MeasScaling;

typedef struct
{
  float ax;
  float ay;
  float az;
} AccelData;

typedef struct
{
  float gx;
  float gy;
  float gz;
} GyroData;

typedef enum
{
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
} Ascale;

typedef enum
{
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
} Gscale;

class MPU6050
{
 private:
  bool init_complete_;
  Ascale a_scale_;
  Gscale g_scale_;
  MeasBias biases_;
  MeasScaling scalings_;

  I2C i2c_;
  Timer t_;

  void reset(void);
  void delay(uint16_t delay_ms);
  void writeByte(uint8_t sub_addr, uint8_t data);
  uint8_t readByte(uint8_t sub_addr);
  int readBytes(uint8_t sub_addr, uint8_t cnt, uint8_t* buffer);
  bool testConnection(void);
  void calibrate(void);
  void runSelfTest(void);

 public:
  MPU6050(PinName sda, PinName scl, Ascale ascale = AFS_2G, Gscale gscale = GFS_250DPS);
  void init(void);
  bool readGyroData(GyroData* destination);
  bool readAccelData(AccelData* destination);
};

}  // namespace mpu6050

#endif  // SENSORS_MPU6050_H_
