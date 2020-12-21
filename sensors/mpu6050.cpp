#include "mpu6050.h"

#include <cmath>

namespace mpu6050
{
#include "mpu6050_regs.h"

//#define DEBUG_MPU6050

#ifdef DEBUG_MPU6050
static Serial debug_out(USBTX, USBRX, 115200);
#endif

static const char mpu6050_addr = 0x68 << 1;
static const float PI = 3.14159265359f;
static const float G = 9.81;
static const float SELFTEST_PASS_THRESHOLD = 1.0f;  // (%)

MPU6050::MPU6050(PinName sda, PinName scl, Ascale ascale, Gscale gscale) : i2c_(sda, scl)
{
  init_complete_ = false;
  (void)memset(&biases_, 0, sizeof(biases_));

  a_scale_ = ascale;
  g_scale_ = gscale;

  switch (ascale)
  {
    case AFS_2G:
      scalings_.accel = 2.0f / 32768.0f * G;
      break;
    case AFS_4G:
      scalings_.accel = 4.0f / 32768.0f * G;
      break;
    case AFS_8G:
      scalings_.accel = 8.0f / 32768.0f * G;
      break;
    case AFS_16G:
      scalings_.accel = 16.0f / 32768.0f * G;
      break;
    default:
      MBED_ASSERT(false);
      break;
  }

  switch (gscale)
  {
    case GFS_250DPS:
      scalings_.gyro = 250.0 / 32768.0 * (PI / 180);
      break;
    case GFS_500DPS:
      scalings_.gyro = 500.0 / 32768.0 * (PI / 180);
      break;
    case GFS_1000DPS:
      scalings_.gyro = 1000.0 / 32768.0 * (PI / 180);
      break;
    case GFS_2000DPS:
      scalings_.gyro = 2000.0 / 32768.0 * (PI / 180);
      break;
    default:
      MBED_ASSERT(false);
      break;
  }
}

void MPU6050::delay(uint16_t delay_ms)
{
  const float start_ms = t_.read_ms();
  while ((t_.read_ms() - start_ms) < delay_ms)
    ;
}

void MPU6050::writeByte(uint8_t sub_addr, uint8_t data)
{
  char wd[2] = {0, 0};

  wd[0] = static_cast<char>(sub_addr);
  wd[1] = static_cast<char>(data);

  i2c_.write(mpu6050_addr, wd, 2);
}

uint8_t MPU6050::readByte(uint8_t sub_addr)
{
  char data_out;

  i2c_.write(mpu6050_addr, reinterpret_cast<char*>(&sub_addr), 1, true);
  i2c_.read(mpu6050_addr, &data_out, 1);

  return static_cast<uint8_t>(data_out);
}

int MPU6050::readBytes(uint8_t sub_addr, uint8_t cnt, uint8_t* buffer)
{
  int ret = 0;

  /* I2C.write returns 0 on success */
  i2c_.write(mpu6050_addr, reinterpret_cast<char*>(&sub_addr), 1, true);

  /* I2C.read returns 0 on success */
  i2c_.read(mpu6050_addr, reinterpret_cast<char*>(buffer), cnt);

  return (ret);
}

bool MPU6050::readGyroData(GyroData* destination)
{
  uint8_t raw_data[6];
  int16_t G[3];
  int status = 0;

  MBED_ASSERT(init_complete_);

  status = readBytes(GYRO_XOUT_H, 6, &raw_data[0]);

  if (0 == status)
  {
    /* Turn the MSB and LSB into a signed 16-bit value */
    G[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    G[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    G[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    destination->gx = (G[0] * scalings_.gyro);
    destination->gy = (G[1] * scalings_.gyro);
    destination->gz = (G[2] * scalings_.gyro);

#ifdef DEBUG_MPU6050
    debug_out.printf("MPU6050 - gx, gy, gz = %.2f, %.2f, %.2f\n\r", destination->gx, destination->gy, destination->gz);
#endif
  }

  return (0 == status ? true : false);
}

bool MPU6050::readAccelData(AccelData* destination)
{
  uint8_t raw_data[6];
  int16_t a[3];
  int status = 0;

  MBED_ASSERT(init_complete_);

  status = readBytes(ACCEL_XOUT_H, 6, &raw_data[0]);

  if (0 == status)
  {
    /* Turn the MSB and LSB into a signed 16-bit value */
    a[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    a[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    a[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    destination->ax = (a[0] * scalings_.accel);
    destination->ay = (a[1] * scalings_.accel);
    destination->az = (a[2] * scalings_.accel);

#ifdef DEBUG_MPU6050
    debug_out.printf("MPU6050 - ax, ay, az = %.2f, %.2f, %.2f\n\r", destination->ax, destination->ay, destination->az);
#endif
  }
  return (0 == status ? true : false);
}

void MPU6050::reset(void)
{
  // Reset the device
  writeByte(PWR_MGMT_1, 0x80);
  delay(100);
}

void MPU6050::init(void)
{
  uint8_t c;

  t_.start();
  i2c_.frequency(100000);
  reset();

  if (testConnection())
  {
#ifdef DEBUG_MPU6050
    debug_out.printf("\n\rMPU6050 connected!\n\r");
#endif
    runSelfTest();
    calibrate();
  }
  else
  {
    MBED_ASSERT(false);
  }

  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(PWR_MGMT_1, 0x01);
  delay(50);

  // Configure Gyro and Accelerometer
  // Disable FSYNC
  // Set accelerometer and gyro bandwidth to 260 and 256 Hz, respectively
  // Accelerometer sample rate = 1kHz, delay = 0ms, no LPF
  // Gyroscope sample rate = 8kHz, delay = 0.98ms, no LPF
  // DLPF_CFG = bits 2:0 = 000
  writeByte(CONFIG, 0x00);
  c = readByte(CONFIG);

  // Set sample rate = gyroscope output rate / (1 + SMPLRT_DIV)
  // Use a 8000 Hz rate; the same rate set in CONFIG above
  // Accelerometer sample rate is still 1kHz
  writeByte(SMPLRT_DIV, 0);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3,
  // so 2-bit values are left-shifted into positions 4:3
  c = readByte(GYRO_CONFIG);
  writeByte(GYRO_CONFIG, c & ~0xE0);            // Clear self-test bits [7:5]
  writeByte(GYRO_CONFIG, c & ~0x18);            // Clear AFS bits [4:3]
  writeByte(GYRO_CONFIG, c | (g_scale_ << 3));  // Set gyro scale

  // Set accelerometer configuration
  c = readByte(ACCEL_CONFIG);
  writeByte(ACCEL_CONFIG, c & ~0xE0);            // Clear self-test bits [7:5]
  writeByte(ACCEL_CONFIG, c & ~0x18);            // Clear AFS bits [4:3]
  writeByte(ACCEL_CONFIG, c | (a_scale_ << 3));  // Set the accelerometer scale

  // Configure Interrupts and Bypass Enable
  writeByte(INT_PIN_CFG, 0x22);
  writeByte(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(50);

  // Wait for data to be available
  while (!(readByte(INT_STATUS) & 0x01))
    ;

  t_.stop();
  init_complete_ = true;
}

bool MPU6050::testConnection(void)
{
  bool test_passed = false;

  if ((mpu6050_addr >> 1) == readByte(WHO_AM_I_MPU6050))
  {
    test_passed = true;
  }

  return test_passed;
}

void MPU6050::calibrate(void)
{
  uint8_t data[12];  // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device, reset all regs, clear gyro and accelerometer bias regs
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(PWR_MGMT_1, 0x80);
  delay(100);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope ref, bits 2:0 = 001
  writeByte(PWR_MGMT_1, 0x01);
  writeByte(PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(INT_ENABLE, 0x00);    // Disable all interrupts
  writeByte(FIFO_EN, 0x00);       // Disable FIFO
  writeByte(PWR_MGMT_1, 0x00);    // Turn on internal clock source
  writeByte(I2C_MST_CTRL, 0x00);  // Disable I2C master
  writeByte(USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
  writeByte(USER_CTRL, 0x0C);     // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculations
  // Set low-pass filter to 188 Hz
  writeByte(CONFIG, 0x01);
  // Set sample rate to 1 kHz
  writeByte(SMPLRT_DIV, 0x00);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(GYRO_CONFIG, 0x00);
  // Set accelerometer full-scale to 2 G, maximum sensitivity
  writeByte(ACCEL_CONFIG, 0x00);

  uint16_t gyrosensitivity = 131;     // = 131 LSB/degrees/sec
  uint16_t accelsensitivity = 16384;  // = 16384 LSB/G

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  // Enable FIFO
  writeByte(USER_CTRL, 0x40);
  // Enable gyro and accelerometer sensors for FIFO
  // (max size 1024 bytes in MPU-6050)
  writeByte(FIFO_EN, 0x78);
  delay(80);  // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read_ms
  // Disable gyro and accelerometer sensors for FIFO
  writeByte(FIFO_EN, 0x00);
  readBytes(FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12;

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(FIFO_R_W, 12, &data[0]);  // read data for averaging

    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
    accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
    accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
    gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
    gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
    gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

    // Sum the 16-bit biases
    accel_bias[0] += (int32_t)accel_temp[0];
    accel_bias[1] += (int32_t)accel_temp[1];
    accel_bias[2] += (int32_t)accel_temp[2];
    gyro_bias[0] += (int32_t)gyro_temp[0];
    gyro_bias[1] += (int32_t)gyro_temp[1];
    gyro_bias[2] += (int32_t)gyro_temp[2];
  }

  // Normalize sums to get average count biases
  accel_bias[0] /= (int32_t)packet_count;
  accel_bias[1] /= (int32_t)packet_count;
  accel_bias[2] /= (int32_t)packet_count;
  gyro_bias[0] /= (int32_t)packet_count;
  gyro_bias[1] /= (int32_t)packet_count;
  gyro_bias[2] /= (int32_t)packet_count;

  if (accel_bias[2] > 0L)
  {
    // Remove gravity from the z-axis accelerometer bias calculation
    accel_bias[2] -= (int32_t)accelsensitivity;
  }
  else
  {
    accel_bias[2] += (int32_t)accelsensitivity;
  }

  // Construct the gyro biases to push to the hardware gyro bias registers
  // Divide by 4 to get 32.9 LSB per deg/s
  data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
  data[1] = (-gyro_bias[0] / 4) & 0xFF;
  data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4) & 0xFF;
  data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4) & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(XG_OFFS_USRH, data[0]);
  writeByte(XG_OFFS_USRL, data[1]);
  writeByte(YG_OFFS_USRH, data[2]);
  writeByte(YG_OFFS_USRL, data[3]);
  writeByte(ZG_OFFS_USRH, data[4]);
  writeByte(ZG_OFFS_USRL, data[5]);

  // construct gyro bias in deg/s for later manual subtraction
  biases_.gyro[0] = static_cast<float>(gyro_bias[0]) / static_cast<float>(gyrosensitivity);
  biases_.gyro[1] = static_cast<float>(gyro_bias[1]) / static_cast<float>(gyrosensitivity);
  biases_.gyro[2] = static_cast<float>(gyro_bias[2]) / static_cast<float>(gyrosensitivity);

  // Construct the accelerometer biases for push to the hardware accelerometer
  // bias registers. These registers contain factory trim values which must be
  // added to the calculated accelerometer biases; on boot up these registers
  // will hold non-zero values. In addition, bit 0 of the lower byte must be
  // preserved since it is used for temperature compensation calculations.
  // Accelerometer bias registers expect bias input as 2048 LSB per G, so that
  // the accelerometer biases calculated above must be divided by 8.

  // A place to hold the factory accelerometer trim biases
  int32_t accel_bias_reg[3] = {0, 0, 0};

  // Read factory accelerometer trim values
  readBytes(XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
  readBytes(YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
  readBytes(ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

  // Define mask for temp compensation bit 0 of lower byte of accel bias regs
  uint32_t mask = 1uL;

  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};

  for (ii = 0; ii < 3; ii++)
  {
    if (accel_bias_reg[ii] & mask)
    {
      // If temperature compensation bit is set, record that fact in mask_bit
      mask_bit[ii] = 0x01;
    }
  }

  // Construct total accel bias, including calculated avg accel bias from above
  // Subtract calculated avg accel bias scaled to 2048 LSB/G (16 G full scale)
  accel_bias_reg[0] -= (accel_bias[0] / 8);
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0]) & 0xFF;
  // preserve temperature compensation bit when writing back to accel bias regs
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1]) & 0xFF;
  // preserve temperature compensation bit when writing back to accel bias regs
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2]) & 0xFF;
  data[5] = data[5] | mask_bit[2];

  // Push accelerometer biases to hardware registers
  writeByte(XA_OFFSET_H, data[0]);
  writeByte(XA_OFFSET_L_TC, data[1]);
  writeByte(YA_OFFSET_H, data[2]);
  writeByte(YA_OFFSET_L_TC, data[3]);
  writeByte(ZA_OFFSET_H, data[4]);
  writeByte(ZA_OFFSET_L_TC, data[5]);

  // Save scaled accelerometer biases
  biases_.accel[0] = static_cast<float>(accel_bias[0]) / static_cast<float>(accelsensitivity);
  biases_.accel[1] = static_cast<float>(accel_bias[1]) / static_cast<float>(accelsensitivity);
  biases_.accel[2] = static_cast<float>(accel_bias[2]) / static_cast<float>(accelsensitivity);

#ifdef DEBUG_MPU6050
  debug_out.printf("MPU6050 calibration complete.\n\r");
#endif
}

// Accelerometer and gyroscope self test;
// check calibration wrt factory settings
// Should return percent deviation from factory trim values,
// +/- 14 or less deviation is a pass
void MPU6050::runSelfTest(void)
{
  uint8_t rawData[4];
  uint8_t selfTest[6];
  float factoryTrim[6], results[6];

  // Configure the accelerometer for self-test
  // Enable self test on all three axes and set accelerometer range to +/- 8 G
  writeByte(ACCEL_CONFIG, 0xF0);
  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  writeByte(GYRO_CONFIG, 0xE0);
  delay(250);  // delay a while to let the device execute the self-test

  rawData[0] = readByte(SELF_TEST_X);  // X-axis self-test results
  rawData[1] = readByte(SELF_TEST_Y);  // Y-axis self-test results
  rawData[2] = readByte(SELF_TEST_Z);  // Z-axis self-test results
  rawData[3] = readByte(SELF_TEST_A);  // Mixed-axis self-test results

  // Extract the acceleration test results first
  // XA_TEST result is a five-bit unsigned integer
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4;
  // YA_TEST result is a five-bit unsigned integer
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2;
  // ZA_TEST result is a five-bit unsigned integer
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03);

  // Extract the gyration test results first
  selfTest[3] = rawData[0] & 0x1F;  // XG_TEST result is a five-bit uint
  selfTest[4] = rawData[1] & 0x1F;  // YG_TEST result is a five-bit uint
  selfTest[5] = rawData[2] & 0x1F;  // ZG_TEST result is a five-bit uint

  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (4096.0 * 0.34) * (pow((0.92 / 0.34), ((selfTest[0] - 1.0f) / 30.0)));  // FT[Xa]
  factoryTrim[1] = (4096.0 * 0.34) * (pow((0.92 / 0.34), ((selfTest[1] - 1.0f) / 30.0)));  // FT[Ya]
  factoryTrim[2] = (4096.0 * 0.34) * (pow((0.92 / 0.34), ((selfTest[2] - 1.0f) / 30.0)));  // FT[Za]
  factoryTrim[3] = (25.0 * 131.0) * (pow(1.046, (selfTest[3] - 1.0)));                     // FT[Xg]
  factoryTrim[4] = (-25.0 * 131.0) * (pow(1.046, (selfTest[4] - 1.0)));                    // FT[Yg]
  factoryTrim[5] = (25.0 * 131.0) * (pow(1.046, (selfTest[5] - 1.0)));                     // FT[Zg]

  // Report results as a ratio of (STR - FT)/FT;
  // the change from Factory Trim of the Self-Test Response
  // To get to percent, must multiply by 100 and subtract result from 100
  for (int i = 0; i < 6; i++)
  {
    results[i] = 100.0 + 100.0 * (selfTest[i] - factoryTrim[i]) / factoryTrim[i];  // Report percent differences
    if (results[i] > SELFTEST_PASS_THRESHOLD)
    {
      MBED_ASSERT(false);
    }
  }
}

}  // namespace mpu6050
