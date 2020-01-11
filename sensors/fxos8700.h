#ifndef SENSORS_FXOS8700CQ_H_
#define SENSORS_FXOS8700CQ_H_

#include "mbed.h"

namespace fxos8700 {

typedef struct {
  float accel[3];
  float magno[3];
} Meas_Biases_T;

typedef struct {
  float accel;
  float magno;
} Meas_Scalings_T;

typedef struct {
  bool data_valid;
  float ax;
  float ay;
  float az;
  float mx;
  float my;
  float mz;
} Sensor_Data_T;

typedef enum {
  FXOS_2G = 0,
  FXOS_4G,
  FXOS_8G,
} Ascale_T;

typedef enum {
  FXOS_STANDBY = 0x00,
  FXOS_ACTIVE  = 0x01
} Mode_T;

typedef union {
  struct {
  uint8_t active:1;
  uint8_t f_read:1;
  uint8_t lnoise:1;
  uint8_t odr:3;
  uint8_t aslp_rate:2;
  };
  uint8_t byte;
} CTRL_1_T;

typedef enum {
  FXOS_400HZ    = 0,
  FXOS_200HZ    = 1,
  FXOS_100HZ    = 2,
  FXOS_50HZ     = 3,
  FXOS_25HZ     = 4,
  FXOS_6p25HZ   = 5,
  FXOS_3p125HZ  = 6,
  FXOS_0p7813HZ = 7,
} ODR_Hybrid_T;

class FXOS8700 {
 private:
  bool init_complete_;
  Ascale_T a_scale_;
  Meas_Biases_T biases_;
  Meas_Scalings_T scalings_;
  uint16_t accel_sensitivity_;

  I2C i2c_;
  Timer t_;

  void Delay(uint16_t delay_ms);
  void WriteByte(uint8_t sub_addr, uint8_t data);
  uint8_t ReadByte(uint8_t sub_addr);
  int ReadBytes(uint8_t sub_addr, uint8_t cnt, uint8_t * buffer);

  bool TestConnection(void);
  void SetMode(Mode_T mode);
  void BasicSetup(void);
  void SetAscale(void);
  void SetODR(ODR_Hybrid_T odr);
  void EnableReducedNoise(void);
  void Calibrate(void);
  CTRL_1_T ReadCTRLREG1(void);

 public:
  FXOS8700(PinName sda, PinName scl, Ascale_T ascale = FXOS_2G);
  void Init(void);
  bool ReadData(Sensor_Data_T * destination);
};

}  // namespace fxos8700

#endif  // SENSORS_FXOS8700CQ_H_