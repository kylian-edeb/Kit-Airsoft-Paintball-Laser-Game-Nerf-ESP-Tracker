#include "compass_mpu.h"
#include "config.h"

#include <Preferences.h>
#include <cmath>

namespace {

constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr uint8_t AK8963_ADDRESS = 0x0C;

constexpr uint8_t MPU_REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t MPU_REG_CONFIG       = 0x1A;
constexpr uint8_t MPU_REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU_REG_INT_PIN_CFG  = 0x37;
constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_REG_USER_CTRL    = 0x6A;
constexpr uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t MPU_REG_WHO_AM_I     = 0x75;

constexpr uint8_t AK_REG_WIA   = 0x00;
constexpr uint8_t AK_REG_ST1   = 0x02;
constexpr uint8_t AK_REG_HXL   = 0x03;
constexpr uint8_t AK_REG_ST2   = 0x09;
constexpr uint8_t AK_REG_CNTL1 = 0x0A;
constexpr uint8_t AK_REG_ASAX  = 0x10;

constexpr float ACCEL_SCALE_2G = 1.0f / 16384.0f;   // g par LSB
constexpr float GYRO_SCALE_250DPS = 1.0f / 131.0f;  // deg/s par LSB
constexpr float MAG_SCALE_16BIT = 0.15f;            // µT par LSB

constexpr char kNvsNamespace[] = "detector";

} // namespace

CompassImu::CompassImu(TwoWire &wire) : _wire(wire) {}

void CompassImu::writeMpuRegister(uint8_t address, uint8_t value)
{
  _wire.beginTransmission(MPU_ADDRESS);
  _wire.write(address);
  _wire.write(value);
  _wire.endTransmission();
}

uint8_t CompassImu::readMpuRegister(uint8_t address)
{
  uint8_t value = 0;
  readMpuRegisters(address, &value, 1);
  return value;
}

void CompassImu::readMpuRegisters(uint8_t address, uint8_t *buffer, uint8_t length)
{
  _wire.beginTransmission(MPU_ADDRESS);
  _wire.write(address);
  _wire.endTransmission(false);

  _wire.requestFrom(static_cast<uint8_t>(MPU_ADDRESS), length);
  for (uint8_t i = 0; i < length && _wire.available(); i++) {
    buffer[i] = _wire.read();
  }
}

void CompassImu::writeAkRegister(uint8_t address, uint8_t value)
{
  _wire.beginTransmission(AK8963_ADDRESS);
  _wire.write(address);
  _wire.write(value);
  _wire.endTransmission();
}

uint8_t CompassImu::readAkRegister(uint8_t address)
{
  uint8_t value = 0;
  readAkRegisters(address, &value, 1);
  return value;
}

void CompassImu::readAkRegisters(uint8_t address, uint8_t *buffer, uint8_t length)
{
  _wire.beginTransmission(AK8963_ADDRESS);
  _wire.write(address);
  _wire.endTransmission(false);

  _wire.requestFrom(static_cast<uint8_t>(AK8963_ADDRESS), length);
  for (uint8_t i = 0; i < length && _wire.available(); i++) {
    buffer[i] = _wire.read();
  }
}

bool CompassImu::initMpu()
{
  writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x80); // reset
  delay(100);
  writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x01); // horloge PLL sur gyro X
  delay(10);

  const uint8_t whoAmI = readMpuRegister(MPU_REG_WHO_AM_I);
  // MPU9250/9255/6500 renvoient 0x71/0x73/0x70 selon le lot de silicium.
  if (whoAmI == 0x00 || whoAmI == 0xFF) {
    return false;
  }

  writeMpuRegister(MPU_REG_CONFIG, 0x03);       // DLPF ~44 Hz
  writeMpuRegister(MPU_REG_SMPLRT_DIV, 0x04);   // 1 kHz / (1+4) = 200 Hz
  writeMpuRegister(MPU_REG_GYRO_CONFIG, 0x00);  // ±250 °/s
  writeMpuRegister(MPU_REG_ACCEL_CONFIG, 0x00); // ±2 g

  writeMpuRegister(MPU_REG_USER_CTRL, 0x00);    // pas de maître I2C interne
  writeMpuRegister(MPU_REG_INT_PIN_CFG, 0x02);  // BYPASS_EN : AK8963 visible sur le bus

  return true;
}

bool CompassImu::initMagnetometer()
{
  const uint8_t wia = readAkRegister(AK_REG_WIA);
  if (wia != 0x48) {
    return false;
  }

  writeAkRegister(AK_REG_CNTL1, 0x00); // power-down
  delay(10);

  writeAkRegister(AK_REG_CNTL1, 0x0F); // accès Fuse ROM
  delay(10);

  uint8_t asa[3];
  readAkRegisters(AK_REG_ASAX, asa, 3);

  // Formule datasheet AK8963 : Hadj = H * ((ASA - 128) * 0.5 / 128 + 1)
  _magAdjX = ((asa[0] - 128) * 0.5f / 128.0f) + 1.0f;
  _magAdjY = ((asa[1] - 128) * 0.5f / 128.0f) + 1.0f;
  _magAdjZ = ((asa[2] - 128) * 0.5f / 128.0f) + 1.0f;

  writeAkRegister(AK_REG_CNTL1, 0x00); // power-down
  delay(10);
  writeAkRegister(AK_REG_CNTL1, 0x16); // 16 bits, mesure continue 100 Hz
  delay(10);

  return true;
}

bool CompassImu::begin()
{
  _wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);

  if (!initMpu()) {
    return false;
  }

  if (!initMagnetometer()) {
    return false;
  }

  loadCalibrationFromNvs();
  return true;
}

ImuSample CompassImu::read()
{
  ImuSample sample;

  uint8_t raw[14];
  readMpuRegisters(MPU_REG_ACCEL_XOUT_H, raw, 14);

  const int16_t accelXRaw = (raw[0] << 8) | raw[1];
  const int16_t accelYRaw = (raw[2] << 8) | raw[3];
  const int16_t accelZRaw = (raw[4] << 8) | raw[5];
  // raw[6..7] = température, non utilisée.
  const int16_t gyroXRaw = (raw[8] << 8) | raw[9];
  const int16_t gyroYRaw = (raw[10] << 8) | raw[11];
  const int16_t gyroZRaw = (raw[12] << 8) | raw[13];

  sample.accelX = accelXRaw * ACCEL_SCALE_2G;
  sample.accelY = accelYRaw * ACCEL_SCALE_2G;
  sample.accelZ = accelZRaw * ACCEL_SCALE_2G;

  sample.gyroX = gyroXRaw * GYRO_SCALE_250DPS;
  sample.gyroY = gyroYRaw * GYRO_SCALE_250DPS;
  sample.gyroZ = gyroZRaw * GYRO_SCALE_250DPS;

  const uint8_t status1 = readAkRegister(AK_REG_ST1);
  if (status1 & 0x01) { // DRDY
    uint8_t magRaw[7];
    readAkRegisters(AK_REG_HXL, magRaw, 7); // 6 data + ST2 (obligatoire à lire)
    const uint8_t status2 = magRaw[6];

    if (!(status2 & 0x08)) { // pas de débordement capteur (HOFL)
      const int16_t magXRaw = (magRaw[1] << 8) | magRaw[0];
      const int16_t magYRaw = (magRaw[3] << 8) | magRaw[2];
      const int16_t magZRaw = (magRaw[5] << 8) | magRaw[4];

      // Repère AK8963 tourné par rapport au repère accel/gyro MPU9250
      // (cf. datasheet InvenSense §"Orientation of Axes") : X_mag = Y_imu,
      // Y_mag = X_imu, Z_mag = -Z_imu. Cette correspondance est appliquée
      // ici pour que cap et inclinaison partagent le même repère.
      const float mx = magYRaw * MAG_SCALE_16BIT * _magAdjY;
      const float my = magXRaw * MAG_SCALE_16BIT * _magAdjX;
      const float mz = -magZRaw * MAG_SCALE_16BIT * _magAdjZ;

      sample.magX = mx - _magOffsetX;
      sample.magY = my - _magOffsetY;
      sample.magZ = mz - _magOffsetZ;
      sample.magValid = true;
    }
  }

  return sample;
}

float CompassImu::headingDegrees(const ImuSample &sample, float declinationDeg) const
{
  if (!sample.magValid) {
    return NAN;
  }

  // Compas tilt-compensé (formule classique type Freescale AN4248),
  // repère X=avant, Y=droite, Z=bas. Si le cap affiché est décalé de
  // 90°/180° ou inversé une fois monté, ce sont ces signes qu'il faut
  // ajuster en fonction de l'orientation physique du module sur le rail.
  const float roll = atan2f(sample.accelY, sample.accelZ);
  const float pitch = atan2f(-sample.accelX,
    sqrtf(sample.accelY * sample.accelY + sample.accelZ * sample.accelZ));

  const float cosRoll = cosf(roll);
  const float sinRoll = sinf(roll);
  const float cosPitch = cosf(pitch);
  const float sinPitch = sinf(pitch);

  const float xh = sample.magX * cosPitch +
                    sample.magY * sinRoll * sinPitch +
                    sample.magZ * cosRoll * sinPitch;
  const float yh = sample.magY * cosRoll - sample.magZ * sinRoll;

  float headingRad = atan2f(-yh, xh);
  float headingDeg = headingRad * 180.0f / static_cast<float>(M_PI);

  headingDeg += declinationDeg;
  headingDeg = fmodf(headingDeg + 360.0f, 360.0f);

  return headingDeg;
}

void CompassImu::startCalibration()
{
  _calibrating = true;
  _calMinX = _calMinY = _calMinZ = 1e6f;
  _calMaxX = _calMaxY = _calMaxZ = -1e6f;
}

void CompassImu::feedCalibrationSample(const ImuSample &sample)
{
  if (!_calibrating || !sample.magValid) {
    return;
  }

  // Les échantillons entrants ont déjà les anciens offsets soustraits :
  // on les ré-ajoute pour calibrer sur la mesure brute.
  const float rawX = sample.magX + _magOffsetX;
  const float rawY = sample.magY + _magOffsetY;
  const float rawZ = sample.magZ + _magOffsetZ;

  _calMinX = min(_calMinX, rawX);
  _calMaxX = max(_calMaxX, rawX);
  _calMinY = min(_calMinY, rawY);
  _calMaxY = max(_calMaxY, rawY);
  _calMinZ = min(_calMinZ, rawZ);
  _calMaxZ = max(_calMaxZ, rawZ);
}

void CompassImu::finishCalibration()
{
  _calibrating = false;

  _magOffsetX = (_calMinX + _calMaxX) / 2.0f;
  _magOffsetY = (_calMinY + _calMaxY) / 2.0f;
  _magOffsetZ = (_calMinZ + _calMaxZ) / 2.0f;

  saveCalibrationToNvs();
}

void CompassImu::loadCalibrationFromNvs()
{
  Preferences prefs;
  prefs.begin(kNvsNamespace, /*readOnly=*/true);

  _magOffsetX = prefs.getFloat("magOffX", 0.0f);
  _magOffsetY = prefs.getFloat("magOffY", 0.0f);
  _magOffsetZ = prefs.getFloat("magOffZ", 0.0f);

  prefs.end();
}

void CompassImu::saveCalibrationToNvs()
{
  Preferences prefs;
  prefs.begin(kNvsNamespace, /*readOnly=*/false);

  prefs.putFloat("magOffX", _magOffsetX);
  prefs.putFloat("magOffY", _magOffsetY);
  prefs.putFloat("magOffZ", _magOffsetZ);

  prefs.end();
}
