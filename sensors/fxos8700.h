#ifndef SENSORS_FXOS8700CQ_H_
#define SENSORS_FXOS8700CQ_H_

#include "mbed.h"

namespace fxos8700
{
typedef struct
{
  float accel[3];
  float magno[3];
} MeasBias;

typedef struct
{
  float accel;
  float magno;
} MeasScaling;

typedef struct
{
  bool data_valid;
  float ax;
  float ay;
  float az;
  float mx;
  float my;
  float mz;
} SensorData;

typedef enum
{
  FXOS_2G = 0,
  FXOS_4G,
  FXOS_8G,
} Ascale;

typedef enum
{
  FXOS_STANDBY = 0x00,
  FXOS_ACTIVE = 0x01
} Mode;

typedef union
{
  struct
  {
    uint8_t active : 1;
    uint8_t f_read : 1;
    uint8_t lnoise : 1;
    uint8_t odr : 3;
    uint8_t aslp_rate : 2;
  };
  uint8_t byte;
} Ctrl1;

typedef enum
{
  FXOS_400HZ = 0,
  FXOS_200HZ = 1,
  FXOS_100HZ = 2,
  FXOS_50HZ = 3,
  FXOS_25HZ = 4,
  FXOS_6p25HZ = 5,
  FXOS_3p125HZ = 6,
  FXOS_0p7813HZ = 7,
} OdrHybrid;

class FXOS8700
{
 private:
  bool init_complete_;
  Ascale a_scale_;
  MeasBias biases_;
  MeasScaling scalings_;
  uint16_t accel_sensitivity_;

  I2C i2c_;
  Timer t_;

  void delay(uint16_t delay_ms);
  void writeByte(uint8_t sub_addr, uint8_t data);
  uint8_t readByte(uint8_t sub_addr);
  int readBytes(uint8_t sub_addr, uint8_t cnt, uint8_t* buffer);

  bool testConnection(void);
  void setMode(Mode mode);
  void basicSetup(void);
  void setAscale(void);
  void setODR(OdrHybrid odr);
  void enableReducedNoise(void);
  void calibrate(void);
  Ctrl1 readCtrlReg1(void);

 public:
  FXOS8700(PinName sda, PinName scl, Ascale ascale = FXOS_2G);
  void init(void);
  bool readData(SensorData* destination);
};

}  // namespace fxos8700

#endif  // SENSORS_FXOS8700CQ_H_