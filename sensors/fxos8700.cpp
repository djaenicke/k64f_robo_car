#include "fxos8700.h"

namespace fxos8700
{
#include "fxos8700_regs.h"

//#define DEBUG_FXOS8700

#define X 0
#define Y 1
#define Z 2

#define UINT14_MAX (0x3FFF)
#define NUM_ACCEL_BIAS_SAMPLES ((uint8_t)100)

#define SENSITIVITY_2G 4096
#define SENSITIVITY_4G 2048
#define SENSITIVITY_8G 1024

#define NORMALIZE_14Bits(x) (((x) > (UINT14_MAX / 2)) ? (x - UINT14_MAX) : (x))
#define GET_14BIT_SIGNED_VAL(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb) >> 2))
#define GET_16BIT_SIGNED_VAL(msb, lsb) ((int16_t)(((uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb)))

#ifdef DEBUG_FXOS8700
static Serial debug_out(USBTX, USBRX, 115200);
#endif

static const char fxos8700_addr_opts[] = {0x1CU << 1, 0x1DU << 1, 0x1EU << 1, 0x1FU << 1};
static char fxos8700_addr = 0x00;
static const char who_am_i_success = 0xC7;
static const float pi = 3.14159265359f;
static const float g = 9.81;

FXOS8700::FXOS8700(PinName sda, PinName scl, Ascale ascale) : i2c_(sda, scl)
{
  switch (ascale)
  {
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

  scalings_.accel = (1.0 / accel_sensitivity_) * g;
}

void FXOS8700::delay(uint16_t delay_ms)
{
  float start_ms = t_.read_ms();
  while ((t_.read_ms() - start_ms) < delay_ms)
  {
  }
}

void FXOS8700::writeByte(uint8_t sub_addr, uint8_t data)
{
  char wd[2] = {0, 0};
  int ret;

  wd[0] = static_cast<char>(sub_addr);
  wd[1] = static_cast<char>(data);

  ret = i2c_.write(fxos8700_addr, wd, 2);

  /* I2C.write returns 0 on success */
  MBED_ASSERT(0 == ret);
}

uint8_t FXOS8700::readByte(uint8_t sub_addr)
{
  char data_out;

  i2c_.write(fxos8700_addr, reinterpret_cast<char*>(&sub_addr), 1, true);
  i2c_.read(fxos8700_addr, &data_out, 1);

  return static_cast<uint8_t>(data_out);
}

int FXOS8700::readBytes(uint8_t sub_addr, uint8_t cnt, uint8_t* buffer)
{
  int ret = 0;

  /* I2C.write returns 0 on success */
  ret |= i2c_.write(fxos8700_addr, reinterpret_cast<char*>(&sub_addr), 1, true);

  /* I2C.read returns 0 on success */
  ret |= i2c_.read(fxos8700_addr, reinterpret_cast<char*>(buffer), cnt);

  return ret;
}

void FXOS8700::init(void)
{
  bool connected = false;

  i2c_.frequency(400000);
  t_.start();

  for (uint8_t i = 0; i < 4; i++)
  {
    fxos8700_addr = fxos8700_addr_opts[i];
    if (true == testConnection())
    {
      connected = true;
#ifdef DEBUG_FXOS8700
      debug_out.printf("\n\rFXOS8700 connected!\n\r");
#endif
      break;
    }
  }

  MBED_ASSERT(connected);

  basicSetup();
  setODR(FXOS_50HZ);
  enableReducedNoise();
  calibrate();
  init_complete_ = true;
}

bool FXOS8700::readData(SensorData* destination)
{
  uint8_t raw_data[12];
  int16_t temp;
  int status = 0;

  /* NULL pointer check */
  MBED_ASSERT(destination);

  /* All data can be read since M_CTRL_REG2[hyb_autoinc_mode] = 1 */
  status = readBytes(OUT_X_MSB_REG, 12, raw_data);

  if (0 == status)
  {
    /* Get the accel data from the sensor data structure in 14 bit left format data */
    temp = NORMALIZE_14Bits(GET_14BIT_SIGNED_VAL(raw_data[0], raw_data[1]));
    destination->ax = (temp * scalings_.accel) - biases_.accel[X];

    temp = NORMALIZE_14Bits(GET_14BIT_SIGNED_VAL(raw_data[2], raw_data[3]));
    destination->ay = (temp * scalings_.accel) - biases_.accel[Y];

    temp = NORMALIZE_14Bits(GET_14BIT_SIGNED_VAL(raw_data[4], raw_data[5]));
    destination->az = (temp * scalings_.accel) - biases_.accel[Z];

    destination->mx = GET_16BIT_SIGNED_VAL(raw_data[6], raw_data[7]);
    destination->my = GET_16BIT_SIGNED_VAL(raw_data[8], raw_data[9]);
    destination->mz = GET_16BIT_SIGNED_VAL(raw_data[10], raw_data[11]);

#ifdef DEBUG_FXOS8700
    debug_out.printf("FXOS8700 - ax, ay, az = %.2f, %.2f, %.2f\n\r", destination->ax, destination->ay, destination->az);
#endif
  }

  return (0 == status) ? true : false;
}

bool FXOS8700::testConnection(void)
{
  bool test_passed = false;

  if (who_am_i_success == readByte(WHO_AM_I_REG))
  {
    test_passed = true;
  }

  return test_passed;
}

void FXOS8700::setMode(Mode mode)
{
  Ctrl1 desired_ctrl_reg1;

  desired_ctrl_reg1.byte = readByte(CTRL_REG1);
  desired_ctrl_reg1.active = mode;
  writeByte(CTRL_REG1, desired_ctrl_reg1.byte);

  do
  {
    desired_ctrl_reg1.byte = readByte(CTRL_REG1);
  } while (mode != desired_ctrl_reg1.active);
}

void FXOS8700::basicSetup(void)
{
  setMode(FXOS_STANDBY);

  /* Disable the FIFO */
  writeByte(F_SETUP_REG, F_MODE_DISABLED);

  /* Enable auto-sleep, low power in sleep, high res in wake */
  writeByte(CTRL_REG2, MOD_HIGH_RES);

  /* Set up mag OSR and Hybrid mode using M_CTRL_REG1, use default for Acc */
  writeByte(M_CTRL_REG1, (M_RST_MASK | M_OSR_MASK | M_HMS_MASK));

  /* Enable hyrid mode auto increment using M_CTRL_REG2 */
  writeByte(M_CTRL_REG2, (M_HYB_AUTOINC_MASK));

  /* Default to 2g mode */
  writeByte(XYZ_DATA_CFG_REG, FULL_SCALE_2G);

  /* Setup the ODR for 50 Hz and activate the accelerometer */
  writeByte(CTRL_REG1, (HYB_DATA_RATE_200HZ | ACTIVE_MASK));

  setMode(FXOS_ACTIVE);
}

void FXOS8700::setAscale(void)
{
  uint8_t g_sensor_range;

  setMode(FXOS_STANDBY);

  writeByte(XYZ_DATA_CFG_REG, (uint8_t)a_scale_);

  do
  {
    delay(1);
    g_sensor_range = readByte(XYZ_DATA_CFG_REG);
  } while (g_sensor_range != (uint8_t)a_scale_);

  setMode(FXOS_ACTIVE);
}

void FXOS8700::setODR(OdrHybrid odr)
{
  Ctrl1 desired_ctrl_reg1;

  setMode(FXOS_STANDBY);

  desired_ctrl_reg1.byte = readByte(CTRL_REG1);
  desired_ctrl_reg1.odr = odr;
  writeByte(CTRL_REG1, desired_ctrl_reg1.byte);

  setMode(FXOS_ACTIVE);
}

void FXOS8700::enableReducedNoise(void)
{
  Ctrl1 desired_ctrl_reg1;

  setMode(FXOS_STANDBY);

  desired_ctrl_reg1.byte = readByte(CTRL_REG1);
  desired_ctrl_reg1.lnoise = 1;
  writeByte(CTRL_REG1, desired_ctrl_reg1.byte);

  setMode(FXOS_ACTIVE);
}

void FXOS8700::calibrate(void)
{
  uint8_t raw_data[6];
  int32_t accel_bias[3] = {0, 0, 0};
  int status = 0;

  for (uint8_t i = 0; i < NUM_ACCEL_BIAS_SAMPLES; i++)
  {
    status = readBytes(OUT_X_MSB_REG, 6, raw_data);

    /* No point in continuing if the calibration fails */
    MBED_ASSERT(0 == status);

    /* Get the accel data from the sensor data structure in 14 bit left format data */
    accel_bias[X] += NORMALIZE_14Bits(GET_14BIT_SIGNED_VAL(raw_data[0], raw_data[1]));
    accel_bias[Y] += NORMALIZE_14Bits(GET_14BIT_SIGNED_VAL(raw_data[2], raw_data[3]));
    accel_bias[Z] += NORMALIZE_14Bits(GET_14BIT_SIGNED_VAL(raw_data[4], raw_data[5]));

    delay(1);
  }

  accel_bias[X] /= NUM_ACCEL_BIAS_SAMPLES;
  accel_bias[Y] /= NUM_ACCEL_BIAS_SAMPLES;
  accel_bias[Z] /= NUM_ACCEL_BIAS_SAMPLES;

  biases_.accel[X] = accel_bias[X] * scalings_.accel;
  biases_.accel[Y] = accel_bias[Y] * scalings_.accel;
  biases_.accel[Z] = (accel_bias[Z] * scalings_.accel) - g;
}

Ctrl1 FXOS8700::readCtrlReg1(void)
{
  Ctrl1 ctrl_reg1;

  ctrl_reg1.byte = readByte(CTRL_REG1);

  return (ctrl_reg1);
}

}  // namespace fxos8700
