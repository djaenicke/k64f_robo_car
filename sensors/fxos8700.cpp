#include "fxos8700.h"

namespace fxos8700 {

#include "fxos8700_regs.h"

//#define DEBUG_FXOS8700

#define X 0
#define Y 1
#define Z 2

#define UINT14_MAX  (0x3FFF)
#define NUM_ACCEL_BIAS_SAMPLES ((uint8_t)100)

#define SENSITIVITY_2G 4096
#define SENSITIVITY_4G 2048
#define SENSITIVITY_8G 1024

#define Normalize_14Bits(x) (((x) > (UINT14_MAX/2)) ? (x - UINT14_MAX):(x))
#define Get_14bit_Signed_Val(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb) >> 2))
#define Get_16bit_Signed_Val(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb)))

#ifdef DEBUG_FXOS8700
  static Serial debug_out(USBTX, USBRX, 115200);
#endif

static const char fxos8700_addr_opts[] = {0x1CU<<1, 0x1DU<<1, 0x1EU<<1, 0x1FU<<1};
static char fxos8700_addr = 0x00;
static const char who_am_i_success = 0xC7;
static const float pi = 3.14159265359f;
static const float g  = 9.81;

FXOS8700::FXOS8700(PinName sda, PinName scl, Ascale_T ascale):i2c_(sda, scl) {
  switch (ascale) {
  case FXOS_2G:
    accel_sensitivity_ = SENSITIVITY_2G;
    break;
  case FXOS_4G:
    accel_sensitivity_ = SENSITIVITY_4G;
    break;
  case FXOS_8G:
    accel_sensitivity_ = SENSITIVITY_8G;
    break;
  default:
    MBED_ASSERT(false);
    break;
  }

  scalings_.accel = (1.0/accel_sensitivity_) * g;
}

void FXOS8700::Delay(uint16_t delay_ms) {
  float start_ms = t_.read_ms();
  while ((t_.read_ms()-start_ms) < delay_ms) {}
}

void FXOS8700::WriteByte(uint8_t sub_addr, uint8_t data) {
  char wd[2] = {0, 0};
  int ret;

  wd[0] = static_cast<char>(sub_addr);
  wd[1] = static_cast<char>(data);

  ret = i2c_.write(fxos8700_addr, wd, 2);

  /* I2C.write returns 0 on success */
  MBED_ASSERT(0 == ret);
}

uint8_t FXOS8700::ReadByte(uint8_t sub_addr) {
  char data_out;

  i2c_.write(fxos8700_addr, reinterpret_cast<char*>(&sub_addr), 1, true);
  i2c_.read(fxos8700_addr, &data_out, 1);

  return static_cast<uint8_t>(data_out);
}

int FXOS8700::ReadBytes(uint8_t sub_addr, uint8_t cnt, uint8_t * buffer) {
  int ret = 0;

  /* I2C.write returns 0 on success */
  ret |= i2c_.write(fxos8700_addr, reinterpret_cast<char*>(&sub_addr), 1, true);

  /* I2C.read returns 0 on success */
  ret |= i2c_.read(fxos8700_addr, reinterpret_cast<char*>(buffer), cnt);

  return(ret);
}

void FXOS8700::Init(void) {
  bool connected = false;

  i2c_.frequency(400000);
  t_.start();

  for (uint8_t i = 0; i < 4; i++) {
    fxos8700_addr = fxos8700_addr_opts[i];
    if (true == TestConnection()) {
      connected = true;
  #ifdef DEBUG_FXOS8700
      debug_out.printf("\n\rFXOS8700 connected!\n\r");
  #endif
      break;
    }
  }

  MBED_ASSERT(connected);

  BasicSetup();
  SetODR(FXOS_50HZ);
  EnableReducedNoise();
  Calibrate();
  init_complete_ = true;
}

bool FXOS8700::ReadData(Sensor_Data_T * destination) {
  uint8_t raw_data[12];
  int16_t temp;
  int status = 0;

  /* NULL pointer check */
  MBED_ASSERT(destination);

  /* All data can be read since M_CTRL_REG2[hyb_autoinc_mode] = 1 */
  status = ReadBytes(OUT_X_MSB_REG, 12, raw_data);

  if (0 == status) {
    /* Get the accel data from the sensor data structure in 14 bit left format data */
    temp = Normalize_14Bits(Get_14bit_Signed_Val(raw_data[0], raw_data[1]));
    destination->ax = (temp * scalings_.accel) - biases_.accel[X];

    temp = Normalize_14Bits(Get_14bit_Signed_Val(raw_data[2], raw_data[3]));
    destination->ay = (temp * scalings_.accel) - biases_.accel[Y];

    temp = Normalize_14Bits(Get_14bit_Signed_Val(raw_data[4], raw_data[5]));
    destination->az = (temp * scalings_.accel) - biases_.accel[Z];

    destination->mx = Get_16bit_Signed_Val(raw_data[6], raw_data[7]);
    destination->my = Get_16bit_Signed_Val(raw_data[8], raw_data[9]);
    destination->mz = Get_16bit_Signed_Val(raw_data[10], raw_data[11]);

#ifdef DEBUG_FXOS8700
    debug_out.printf("FXOS8700 - ax, ay, az = %.2f, %.2f, %.2f\n\r", \
                     destination->ax, destination->ay, destination->az);
#endif
  }

  return(0 == status ? true : false);
}

bool FXOS8700::TestConnection(void) {
  bool test_passed = false;
  uint8_t response = 0;

  response = ReadByte(WHO_AM_I_REG);

  if (who_am_i_success == response) {
    test_passed = true;
  }

  return (test_passed);
}

void FXOS8700::SetMode(Mode_T mode) {
  CTRL_1_T desired_ctrl_reg1;

  desired_ctrl_reg1.byte = ReadByte(CTRL_REG1);
  desired_ctrl_reg1.active = mode;
  WriteByte(CTRL_REG1, desired_ctrl_reg1.byte);
  
  do {
    desired_ctrl_reg1.byte = ReadByte(CTRL_REG1);
  } while (mode != desired_ctrl_reg1.active);
}

void FXOS8700::BasicSetup(void) {

    SetMode(FXOS_STANDBY);

    /* Disable the FIFO */
    WriteByte(F_SETUP_REG, F_MODE_DISABLED);

    /* Enable auto-sleep, low power in sleep, high res in wake */
    WriteByte(CTRL_REG2, MOD_HIGH_RES);

    /* Set up mag OSR and Hybrid mode using M_CTRL_REG1, use default for Acc */
    WriteByte(M_CTRL_REG1, (M_RST_MASK | M_OSR_MASK | M_HMS_MASK));

    /* Enable hyrid mode auto increment using M_CTRL_REG2 */
    WriteByte(M_CTRL_REG2, (M_HYB_AUTOINC_MASK));

    /* Default to 2g mode */
    WriteByte(XYZ_DATA_CFG_REG, FULL_SCALE_2G);

    /* Setup the ODR for 50 Hz and activate the accelerometer */
    WriteByte(CTRL_REG1, (HYB_DATA_RATE_200HZ | ACTIVE_MASK));

    SetMode(FXOS_ACTIVE);
}

void FXOS8700::SetAscale(void) {
   uint8_t g_sensor_range;

   SetMode(FXOS_STANDBY);

   WriteByte(XYZ_DATA_CFG_REG, (uint8_t) a_scale_);

   do {
      Delay(1);
      g_sensor_range = ReadByte(XYZ_DATA_CFG_REG);
   } while (g_sensor_range != (uint8_t) a_scale_);

   SetMode(FXOS_ACTIVE);
}

void FXOS8700::SetODR(ODR_Hybrid_T odr) {
  CTRL_1_T desired_ctrl_reg1;

  SetMode(FXOS_STANDBY);

  desired_ctrl_reg1.byte = ReadByte(CTRL_REG1);
  desired_ctrl_reg1.odr = odr;
  WriteByte(CTRL_REG1, desired_ctrl_reg1.byte);

  SetMode(FXOS_ACTIVE);
}

void FXOS8700::EnableReducedNoise(void) {
  CTRL_1_T desired_ctrl_reg1;

  SetMode(FXOS_STANDBY);

  desired_ctrl_reg1.byte = ReadByte(CTRL_REG1);
  desired_ctrl_reg1.lnoise = 1;
  WriteByte(CTRL_REG1, desired_ctrl_reg1.byte);

  SetMode(FXOS_ACTIVE);
}

void FXOS8700::Calibrate(void) {
   uint8_t raw_data[6];
   int32_t accel_bias[3] = {0, 0, 0};
   int status = 0;

   for (uint8_t i=0; i < NUM_ACCEL_BIAS_SAMPLES; i++) {
      status = ReadBytes(OUT_X_MSB_REG, 6, raw_data);

      /* No point in continuing if the calibration fails */
      MBED_ASSERT(0 == status);

      /* Get the accel data from the sensor data structure in 14 bit left format data */
      accel_bias[X] += Normalize_14Bits(Get_14bit_Signed_Val(raw_data[0], raw_data[1]));
      accel_bias[Y] += Normalize_14Bits(Get_14bit_Signed_Val(raw_data[2], raw_data[3]));
      accel_bias[Z] += Normalize_14Bits(Get_14bit_Signed_Val(raw_data[4], raw_data[5]));

      Delay(1);
   }

   accel_bias[X] /= NUM_ACCEL_BIAS_SAMPLES;
   accel_bias[Y] /= NUM_ACCEL_BIAS_SAMPLES;
   accel_bias[Z] /= NUM_ACCEL_BIAS_SAMPLES;

   biases_.accel[X] =  accel_bias[X] * scalings_.accel;
   biases_.accel[Y] =  accel_bias[Y] * scalings_.accel;
   biases_.accel[Z] = (accel_bias[Z] * scalings_.accel) - g;
}

CTRL_1_T FXOS8700::ReadCTRLREG1(void) {
  CTRL_1_T ctrl_reg1;

  ctrl_reg1.byte = ReadByte(CTRL_REG1);

  return(ctrl_reg1);
}

}  // namespace fxos8700
