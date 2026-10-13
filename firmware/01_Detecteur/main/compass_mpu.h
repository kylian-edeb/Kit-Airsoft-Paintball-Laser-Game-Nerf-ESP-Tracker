#pragma once

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// Pilote IMU pour le module "MPU9265" (compatible registre
// MPU9250/MPU9255 : accéléromètre + gyroscope InvenSense et
// magnétomètre AK8963 accessible en mode bypass I2C).
//
// Le Détecteur n'utilise que le cap magnétique (boussole),
// compensé par l'inclinaison mesurée à l'accéléromètre. Le
// gyroscope est lu mais non fusionné (pas de filtre de Kalman/
// Madgwick ici : un simple compas tilt-compensé suffit pour
// afficher un relèvement, et évite la dérive d'une intégration
// gyroscopique non recalée).
//
// Étalonnage : la magnétométrie bon marché souffre d'un fort
// biais "hard-iron" (aimantation locale de la carte/du boîtier).
// Une routine de calibration (rotation complète à 360°) mesure
// les min/max par axe et les stocke en NVS via Preferences.
// ============================================================

struct ImuSample {
  float accelX, accelY, accelZ; // g
  float gyroX, gyroY, gyroZ;    // deg/s
  float magX, magY, magZ;       // µT, après compensation hard-iron
  bool  magValid = false;
};

class CompassImu {
public:
  explicit CompassImu(TwoWire &wire);

  // Initialise le MPU9250/9265 et le magnétomètre AK8963 en mode
  // bypass. Charge l'étalonnage précédent depuis la NVS s'il existe.
  bool begin();

  // Lit un échantillon complet (accel + gyro + mag si disponible).
  ImuSample read();

  // Cap magnétique compensé par l'inclinaison, en degrés [0, 360).
  // `declinationDeg` est la déclinaison magnétique locale à ajouter
  // (positive vers l'est) pour obtenir un cap vrai plutôt que magnétique.
  float headingDegrees(const ImuSample &sample, float declinationDeg = 0.0f) const;

  // --- Étalonnage du magnétomètre (hard-iron) ---
  void startCalibration();
  void feedCalibrationSample(const ImuSample &sample);
  void finishCalibration(); // calcule les offsets et les sauvegarde en NVS
  bool isCalibrating() const { return _calibrating; }

private:
  TwoWire &_wire;

  float _magOffsetX = 0.0f, _magOffsetY = 0.0f, _magOffsetZ = 0.0f;
  float _magAdjX = 1.0f, _magAdjY = 1.0f, _magAdjZ = 1.0f;

  bool _calibrating = false;
  float _calMinX = 0, _calMaxX = 0;
  float _calMinY = 0, _calMaxY = 0;
  float _calMinZ = 0, _calMaxZ = 0;

  void writeMpuRegister(uint8_t address, uint8_t value);
  uint8_t readMpuRegister(uint8_t address);
  void readMpuRegisters(uint8_t address, uint8_t *buffer, uint8_t length);

  void writeAkRegister(uint8_t address, uint8_t value);
  uint8_t readAkRegister(uint8_t address);
  void readAkRegisters(uint8_t address, uint8_t *buffer, uint8_t length);

  bool initMpu();
  bool initMagnetometer();
  void loadCalibrationFromNvs();
  void saveCalibrationToNvs();
};
