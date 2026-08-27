/*
  ============================================================
  Détecteur — unité principale du Kit de Détection Tactique
  Version Arduino IDE (fichier unique), en attendant que la
  configuration VSCode + Espressif-IDE soit résolue.

  Fonctionnellement identique à la version ESP-IDF du dossier
  firmware/Detecteur/main/ (mêmes fichiers, simplement fusionnés
  ici en un seul .ino puisque c'est ce format qu'attend l'IDE
  Arduino). Toute modification durable devrait à terme être
  reportée dans les deux versions, ou l'une des deux abandonnée
  une fois votre outil définitif choisi.

  ------------------------------------------------------------
  RÉGLAGES ARDUINO IDE (Outils > ...)
  ------------------------------------------------------------
   - Carte            : "ESP32S3 Dev Module" (ou la carte N16R8
                         spécifique si elle apparaît dans la liste
                         après installation du board package)
   - USB CDC On Boot   : "Enabled" (pour voir les logs Serial
                         sans câble UART externe)
   - Flash Size        : "16MB (128Mb)"
   - Partition Scheme   : "Default 4MB with spiffs" convient pour
                         16MB aussi, ou "16M Flash (3MB APP/9.9MB FATFS)"
   - PSRAM             : "OPI PSRAM"   <-- IMPORTANT pour le N16R8
   - Upload Speed       : 921600 (ou plus bas si l'upload échoue)

  ------------------------------------------------------------
  LIBRAIRIES À INSTALLER (Outils > Gérer les bibliothèques)
  ------------------------------------------------------------
   - "Adafruit GFX Library"     (par Adafruit)
   - "Adafruit ILI9341"         (par Adafruit)
   Rien d'autre : le GPS (NMEA), le LoRa (SX1278) et la boussole
   (MPU9250/9265+AK8963) sont pilotés ici par du code maison, sans
   dépendance externe supplémentaire.

  Nécessite le core "esp32" par Espressif Systems en version >= 3.0
  (pour tone()/noTone() et les API SPI/ledc utilisées).
  ============================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>

// ============================================================
// 1) BROCHAGE ET CONSTANTES APPLICATIVES
//    (ESP32-S3 N16R8 : GPIO33-37 réservés au bus PSRAM/flash
//    interne — Octal PSRAM —, ne jamais les utiliser)
// ============================================================

// --- Écran TFT ILI9341 (SPI) ---
constexpr int PIN_TFT_SCLK = 12;
constexpr int PIN_TFT_MOSI = 11;
constexpr int PIN_TFT_MISO = 13;
constexpr int PIN_TFT_CS   = 10;
constexpr int PIN_TFT_DC   = 9;
constexpr int PIN_TFT_RST  = 8;
constexpr int PIN_TFT_LED  = 7;

// --- Encodeur rotatif ---
constexpr int PIN_ENC_A  = 4;
constexpr int PIN_ENC_B  = 5;
constexpr int PIN_ENC_SW = 6;

// --- IMU MPU9265/9250 (I2C) ---
constexpr int PIN_I2C_SDA = 17;
constexpr int PIN_I2C_SCL = 18;
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// --- Radio LoRa-02 / SX1278 (partage le bus SPI de l'écran) ---
constexpr int PIN_LORA_CS   = 21;
constexpr int PIN_LORA_RST  = 14;
constexpr int PIN_LORA_DIO0 = 38;
constexpr float LORA_FREQUENCY_HZ = 433000000.0f; // 433 MHz EU
constexpr uint8_t LORA_SPREADING_FACTOR = 9;       // 6-12
constexpr uint8_t LORA_BANDWIDTH_INDEX  = 7;       // 7 => 125 kHz
constexpr uint8_t LORA_CODING_RATE      = 5;       // 4/5
constexpr uint8_t LORA_SYNC_WORD        = 0x34;    // réseau privé LoRa

// --- GPS u-blox NEO-6M (UART1) ---
// GPIO39/40 sont aussi les broches JTAG par défaut (MTCK/MTDO) de
// l'ESP32-S3 : sans importance si le débogage JTAG matériel n'est pas
// câblé sur la carte, mais à reconsidérer si c'est le cas.
constexpr int PIN_GPS_RX = 39; // relié au TX du GPS
constexpr int PIN_GPS_TX = 40; // relié au RX du GPS
constexpr uint32_t GPS_BAUDRATE = 9600;

// --- Buzzer passif ---
constexpr int PIN_BUZZER = 2;

// --- Batterie (pont diviseur vers ADC1) ---
constexpr int PIN_BATTERY_ADC = 1;
constexpr float BATTERY_DIVIDER_RATIO = 2.0f;   // à ajuster selon le pont réel
constexpr float BATTERY_EMPTY_V = 3.3f;
constexpr float BATTERY_FULL_V  = 4.2f;

constexpr uint8_t  MAX_TRACKED_BEACONS   = 16;
constexpr uint32_t BEACON_TIMEOUT_MS     = 8000;   // balise considérée perdue
constexpr uint32_t BEACON_STALE_MS       = 3000;   // affichage "signal faible"
constexpr uint32_t DISPLAY_REFRESH_MS    = 150;

constexpr uint32_t ENCODER_LONGPRESS_MS    = 600;
constexpr uint32_t CALIBRATION_HOLD_MS     = 3000;
constexpr uint32_t CALIBRATION_DURATION_MS = 15000;

// ============================================================
// 2) PROTOCOLE : format du paquet LoRa balise -> Détecteur
//    (doit rester identique au firmware des balises BEM/BPS)
// ============================================================

enum class BeaconType : uint8_t {
  MobileBeacon = 0, // BEM - Balise à Émission Mobile
  StaticBeacon = 1, // BPS - Balise de Positionnement Statique
};

enum class BeaconStatus : uint8_t {
  Active      = 0,
  Eliminated  = 1,
  LowBattery  = 2,
};

#pragma pack(push, 1)
struct BeaconPacket {
  uint8_t  protocolVersion;   // incrémenté à chaque changement incompatible
  uint8_t  beaconId;          // identifiant unique de la balise
  uint8_t  teamId;            // équipe (0 = neutre/BPS)
  uint8_t  type;              // BeaconType
  uint8_t  status;            // BeaconStatus
  int32_t  latitudeE7;        // degrés * 1e7 (WGS84)
  int32_t  longitudeE7;       // degrés * 1e7 (WGS84)
  int16_t  altitudeM;         // mètres au-dessus du niveau de la mer
  uint8_t  satellites;        // nombre de satellites GPS utilisés par la balise
  uint8_t  batteryPercent;    // 0-100
  uint32_t uptimeSeconds;     // temps depuis le démarrage de la balise
  uint16_t sequenceNumber;    // incrémenté à chaque émission (détection de pertes)
};
#pragma pack(pop)

constexpr uint8_t PROTOCOL_VERSION = 1;

struct GeoPoint {
  double latitudeDeg;
  double longitudeDeg;
};

inline double degToRadGeo(double deg) { return deg * M_PI / 180.0; }
inline double radToDegGeo(double rad) { return rad * 180.0 / M_PI; }

// Distance en mètres (formule haversine).
inline double haversineDistanceMeters(const GeoPoint &from, const GeoPoint &to)
{
  constexpr double EARTH_RADIUS_M = 6371000.0;

  const double lat1 = degToRadGeo(from.latitudeDeg);
  const double lat2 = degToRadGeo(to.latitudeDeg);
  const double dLat = degToRadGeo(to.latitudeDeg - from.latitudeDeg);
  const double dLon = degToRadGeo(to.longitudeDeg - from.longitudeDeg);

  const double a = sin(dLat / 2) * sin(dLat / 2) +
                    cos(lat1) * cos(lat2) *
                    sin(dLon / 2) * sin(dLon / 2);
  const double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS_M * c;
}

// Relèvement initial en degrés [0, 360) de "from" vers "to".
inline double initialBearingDegrees(const GeoPoint &from, const GeoPoint &to)
{
  const double lat1 = degToRadGeo(from.latitudeDeg);
  const double lat2 = degToRadGeo(to.latitudeDeg);
  const double dLon = degToRadGeo(to.longitudeDeg - from.longitudeDeg);

  const double y = sin(dLon) * cos(lat2);
  const double x = cos(lat1) * sin(lat2) -
                    sin(lat1) * cos(lat2) * cos(dLon);

  double bearing = radToDegGeo(atan2(y, x));
  bearing = fmod(bearing + 360.0, 360.0);
  return bearing;
}

// ============================================================
// 3) PILOTE RADIO LoRa-02 (SX1278) — registre direct, réception
//    continue avec CRC matériel.
// ============================================================

struct LoRaReceivedPacket {
  uint8_t data[256];
  uint8_t length;
  int16_t rssi;   // dBm
  float   snr;    // dB
};

class LoRaRadio {
public:
  LoRaRadio(SPIClass &spi, int csPin, int resetPin, int dio0Pin)
    : _spi(spi), _csPin(csPin), _resetPin(resetPin), _dio0Pin(dio0Pin)
  {
  }

  bool begin(float frequencyHz, uint8_t spreadingFactor,
             uint8_t bandwidthIndex, uint8_t codingRate, uint8_t syncWord);
  bool poll(LoRaReceivedPacket &out);

private:
  SPIClass &_spi;
  int _csPin;
  int _resetPin;
  int _dio0Pin;
  bool _lastDio0State = false;

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  void readFifo(uint8_t *buffer, uint8_t length);

  void setFrequency(float frequencyHz);
  void setLoRaMode();
  void setRxContinuousMode();
};

namespace {

constexpr uint8_t REG_FIFO                 = 0x00;
constexpr uint8_t REG_OP_MODE              = 0x01;
constexpr uint8_t REG_FRF_MSB              = 0x06;
constexpr uint8_t REG_FRF_MID              = 0x07;
constexpr uint8_t REG_FRF_LSB              = 0x08;
constexpr uint8_t REG_LNA                  = 0x0C;
constexpr uint8_t REG_FIFO_ADDR_PTR        = 0x0D;
constexpr uint8_t REG_FIFO_TX_BASE_ADDR    = 0x0E;
constexpr uint8_t REG_FIFO_RX_BASE_ADDR    = 0x0F;
constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
constexpr uint8_t REG_IRQ_FLAGS            = 0x12;
constexpr uint8_t REG_RX_NB_BYTES          = 0x13;
constexpr uint8_t REG_PKT_SNR_VALUE        = 0x19;
constexpr uint8_t REG_PKT_RSSI_VALUE       = 0x1A;
constexpr uint8_t REG_MODEM_CONFIG_1       = 0x1D;
constexpr uint8_t REG_MODEM_CONFIG_2       = 0x1E;
constexpr uint8_t REG_PREAMBLE_MSB         = 0x20;
constexpr uint8_t REG_PREAMBLE_LSB         = 0x21;
constexpr uint8_t REG_MODEM_CONFIG_3       = 0x26;
constexpr uint8_t REG_SYNC_WORD            = 0x39;
constexpr uint8_t REG_DIO_MAPPING_1        = 0x40;
constexpr uint8_t REG_VERSION              = 0x42;

constexpr uint8_t MODE_LONG_RANGE_MODE = 0x80;
constexpr uint8_t MODE_SLEEP           = 0x00;
constexpr uint8_t MODE_RX_CONTINUOUS   = 0x05;

constexpr uint8_t IRQ_RX_DONE_MASK           = 0x40;
constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20;

constexpr uint32_t SX1278_XOSC_HZ = 32000000UL;
constexpr uint32_t FREQ_STEP_DENOMINATOR = 1UL << 19;

const SPISettings kLoraSpiSettings(8000000, MSBFIRST, SPI_MODE0);

} // namespace

uint8_t LoRaRadio::readRegister(uint8_t address)
{
  _spi.beginTransaction(kLoraSpiSettings);
  digitalWrite(_csPin, LOW);

  _spi.transfer(address & 0x7F);
  uint8_t value = _spi.transfer(0x00);

  digitalWrite(_csPin, HIGH);
  _spi.endTransaction();

  return value;
}

void LoRaRadio::writeRegister(uint8_t address, uint8_t value)
{
  _spi.beginTransaction(kLoraSpiSettings);
  digitalWrite(_csPin, LOW);

  _spi.transfer(address | 0x80);
  _spi.transfer(value);

  digitalWrite(_csPin, HIGH);
  _spi.endTransaction();
}

void LoRaRadio::readFifo(uint8_t *buffer, uint8_t length)
{
  _spi.beginTransaction(kLoraSpiSettings);
  digitalWrite(_csPin, LOW);

  _spi.transfer(REG_FIFO & 0x7F);
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = _spi.transfer(0x00);
  }

  digitalWrite(_csPin, HIGH);
  _spi.endTransaction();
}

void LoRaRadio::setFrequency(float frequencyHz)
{
  const uint64_t frf =
    (static_cast<uint64_t>(frequencyHz) * FREQ_STEP_DENOMINATOR) /
    SX1278_XOSC_HZ;

  writeRegister(REG_FRF_MSB, static_cast<uint8_t>(frf >> 16));
  writeRegister(REG_FRF_MID, static_cast<uint8_t>(frf >> 8));
  writeRegister(REG_FRF_LSB, static_cast<uint8_t>(frf >> 0));
}

void LoRaRadio::setLoRaMode()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
}

void LoRaRadio::setRxContinuousMode()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

bool LoRaRadio::begin(float frequencyHz, uint8_t spreadingFactor,
                       uint8_t bandwidthIndex, uint8_t codingRate,
                       uint8_t syncWord)
{
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  pinMode(_resetPin, OUTPUT);
  pinMode(_dio0Pin, INPUT);

  // Reset matériel du module (voir datasheet SX1276/78, section reset).
  digitalWrite(_resetPin, LOW);
  delay(10);
  digitalWrite(_resetPin, HIGH);
  delay(10);

  const uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    // 0x12 = SX1276/77/78/79. Autre valeur => câblage ou module invalide.
    return false;
  }

  setLoRaMode();
  setFrequency(frequencyHz);

  writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  const uint8_t bw = bandwidthIndex & 0x0F;
  const uint8_t cr = (codingRate - 4) & 0x07; // 4/5..4/8 -> 1..4
  writeRegister(REG_MODEM_CONFIG_1, (bw << 4) | (cr << 1));

  const uint8_t sf = spreadingFactor & 0x0F;
  constexpr uint8_t CRC_ON = 0x04;
  writeRegister(REG_MODEM_CONFIG_2, (sf << 4) | CRC_ON);

  const bool lowDataRateOptimize = (spreadingFactor >= 11) && (bandwidthIndex <= 7);
  writeRegister(REG_MODEM_CONFIG_3, 0x04 | (lowDataRateOptimize ? 0x08 : 0x00));

  writeRegister(REG_PREAMBLE_MSB, 0x00);
  writeRegister(REG_PREAMBLE_LSB, 0x08);

  writeRegister(REG_SYNC_WORD, syncWord);
  writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 -> RxDone
  writeRegister(REG_IRQ_FLAGS, 0xFF);     // purge des drapeaux résiduels

  setRxContinuousMode();

  return true;
}

bool LoRaRadio::poll(LoRaReceivedPacket &out)
{
  const bool dio0State = digitalRead(_dio0Pin) == HIGH;
  const bool risingEdge = dio0State && !_lastDio0State;
  _lastDio0State = dio0State;

  if (!risingEdge) {
    return false;
  }

  const uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
  writeRegister(REG_IRQ_FLAGS, 0xFF); // acquitte tous les drapeaux

  if (!(irqFlags & IRQ_RX_DONE_MASK)) {
    return false;
  }

  if (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
    return false; // paquet corrompu, ignoré
  }

  const uint8_t length = readRegister(REG_RX_NB_BYTES);
  const uint8_t currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);

  writeRegister(REG_FIFO_ADDR_PTR, currentAddr);
  readFifo(out.data, length);
  out.length = length;

  const uint8_t rawRssi = readRegister(REG_PKT_RSSI_VALUE);
  out.rssi = static_cast<int16_t>(rawRssi) - 164; // offset bande basse <525MHz

  const int8_t rawSnr = static_cast<int8_t>(readRegister(REG_PKT_SNR_VALUE));
  out.snr = rawSnr / 4.0f;

  return true;
}

// ============================================================
// 4) PARSEUR NMEA (GPS u-blox NEO-6M) — sans dépendance externe
// ============================================================

struct GpsFix {
  bool     hasFix         = false;
  double   latitudeDeg    = 0.0;
  double   longitudeDeg   = 0.0;
  float    altitudeM      = 0.0f;
  uint8_t  satellites     = 0;
  float    speedKmh       = 0.0f;
  float    courseDeg      = 0.0f;
  uint32_t lastFixMillis  = 0;
};

class GpsReceiver {
public:
  void begin(HardwareSerial &serial)
  {
    _serial = &serial;
    _lineLen = 0;
  }

  void update();

  const GpsFix &fix() const { return _fix; }

private:
  static constexpr uint8_t kMaxLineLength = 96;

  HardwareSerial *_serial = nullptr;
  char _line[kMaxLineLength];
  uint8_t _lineLen = 0;
  GpsFix _fix;

  void handleLine(char *line);
  bool checksumValid(const char *line) const;
  void parseGGA(char *line);
  void parseRMC(char *line);

  static double nmeaCoordinateToDecimalDegrees(const char *nmeaCoord, char hemisphere);
};

void GpsReceiver::update()
{
  if (_serial == nullptr) {
    return;
  }

  while (_serial->available() > 0) {
    const char c = static_cast<char>(_serial->read());

    if (c == '\n') {
      continue;
    }

    if (c == '\r') {
      if (_lineLen > 0) {
        _line[_lineLen] = '\0';
        handleLine(_line);
        _lineLen = 0;
      }
      continue;
    }

    if (_lineLen < kMaxLineLength - 1) {
      _line[_lineLen++] = c;
    } else {
      _lineLen = 0; // ligne anormalement longue (bruit série) : abandonnée
    }
  }
}

bool GpsReceiver::checksumValid(const char *line) const
{
  if (line[0] != '$') {
    return false;
  }

  const char *star = strchr(line, '*');
  if (star == nullptr) {
    return false;
  }

  uint8_t computed = 0;
  for (const char *p = line + 1; p < star; p++) {
    computed ^= static_cast<uint8_t>(*p);
  }

  const uint8_t received = static_cast<uint8_t>(strtol(star + 1, nullptr, 16));
  return computed == received;
}

void GpsReceiver::handleLine(char *line)
{
  if (!checksumValid(line)) {
    return;
  }

  // Les préfixes talker peuvent être GP (GPS seul) ou GN (multi-GNSS).
  if (strncmp(line + 3, "GGA", 3) == 0) {
    parseGGA(line);
  } else if (strncmp(line + 3, "RMC", 3) == 0) {
    parseRMC(line);
  }
}

double GpsReceiver::nmeaCoordinateToDecimalDegrees(const char *nmeaCoord, char hemisphere)
{
  if (nmeaCoord[0] == '\0') {
    return 0.0;
  }

  const double raw = atof(nmeaCoord);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - (degrees * 100.0);

  double decimal = degrees + (minutes / 60.0);

  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal = -decimal;
  }

  return decimal;
}

// $GxGGA,hhmmss.ss,ddmm.mmmm,N,dddmm.mmmm,E,fix,numSat,hdop,alt,M,geoid,M,,*CS
void GpsReceiver::parseGGA(char *line)
{
  char *fields[15] = { nullptr };
  uint8_t count = 0;

  char *token = strtok(line, ",");
  while (token != nullptr && count < 15) {
    fields[count++] = token;
    token = strtok(nullptr, ",");
  }

  if (count < 10) {
    return;
  }

  const int fixQuality = atoi(fields[6]);
  _fix.hasFix = fixQuality > 0;
  _fix.satellites = static_cast<uint8_t>(atoi(fields[7]));

  if (_fix.hasFix) {
    _fix.latitudeDeg = nmeaCoordinateToDecimalDegrees(fields[2], fields[3][0]);
    _fix.longitudeDeg = nmeaCoordinateToDecimalDegrees(fields[4], fields[5][0]);
    _fix.altitudeM = static_cast<float>(atof(fields[9]));
    _fix.lastFixMillis = millis();
  }
}

// $GxRMC,hhmmss.ss,status,ddmm.mmmm,N,dddmm.mmmm,E,speedKt,course,ddmmyy,...*CS
void GpsReceiver::parseRMC(char *line)
{
  char *fields[13] = { nullptr };
  uint8_t count = 0;

  char *token = strtok(line, ",");
  while (token != nullptr && count < 13) {
    fields[count++] = token;
    token = strtok(nullptr, ",");
  }

  if (count < 9) {
    return;
  }

  const bool valid = (fields[2][0] == 'A');
  if (!valid) {
    return;
  }

  _fix.latitudeDeg = nmeaCoordinateToDecimalDegrees(fields[3], fields[4][0]);
  _fix.longitudeDeg = nmeaCoordinateToDecimalDegrees(fields[5], fields[6][0]);
  _fix.speedKmh = static_cast<float>(atof(fields[7])) * 1.852f; // nœuds -> km/h
  _fix.courseDeg = static_cast<float>(atof(fields[8]));
  _fix.hasFix = true;
  _fix.lastFixMillis = millis();
}

// ============================================================
// 5) BOUSSOLE MPU9265/9250 + AK8963 (compas tilt-compensé,
//    étalonnage hard-iron persistant en NVS)
// ============================================================

struct ImuSample {
  float accelX, accelY, accelZ; // g
  float gyroX, gyroY, gyroZ;    // deg/s
  float magX, magY, magZ;       // µT, après compensation hard-iron
  bool  magValid = false;
};

class CompassImu {
public:
  explicit CompassImu(TwoWire &wire) : _wire(wire) {}

  bool begin();
  ImuSample read();
  float headingDegrees(const ImuSample &sample, float declinationDeg = 0.0f) const;

  void startCalibration();
  void feedCalibrationSample(const ImuSample &sample);
  void finishCalibration();
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
constexpr uint8_t AK_REG_CNTL1 = 0x0A;
constexpr uint8_t AK_REG_ASAX  = 0x10;

constexpr float ACCEL_SCALE_2G = 1.0f / 16384.0f;   // g par LSB
constexpr float GYRO_SCALE_250DPS = 1.0f / 131.0f;  // deg/s par LSB
constexpr float MAG_SCALE_16BIT = 0.15f;            // µT par LSB

constexpr char kNvsNamespace[] = "detector";

} // namespace

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

      // Repère AK8963 tourné par rapport au repère accel/gyro MPU9250 :
      // X_mag = Y_imu, Y_mag = X_imu, Z_mag = -Z_imu.
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

// ============================================================
// 6) BUZZER — alertes sonores non bloquantes (tone()/noTone())
// ============================================================

enum class BuzzerPattern {
  None,
  SingleBeep,   // nouvelle balise détectée
  DoubleBeep,   // cible sélectionnée perdue
  TripleBeep,   // batterie faible / fin de calibration
};

class Buzzer {
public:
  explicit Buzzer(int pin) : _pin(pin) {}

  void begin()
  {
    pinMode(_pin, OUTPUT);
    noTone(_pin);
  }

  void play(BuzzerPattern pattern);
  void update();

private:
  int _pin;

  BuzzerPattern _activePattern = BuzzerPattern::None;
  uint8_t _stepIndex = 0;
  uint32_t _nextStepMillis = 0;
};

namespace {

struct BuzzerPatternSpec {
  uint8_t beepCount;
  uint16_t frequencyHz;
  uint16_t onMs;
  uint16_t offMs;
};

BuzzerPatternSpec buzzerSpecFor(BuzzerPattern pattern)
{
  switch (pattern) {
    case BuzzerPattern::SingleBeep: return { 1, 2400, 80, 80 };
    case BuzzerPattern::DoubleBeep: return { 2, 2000, 100, 100 };
    case BuzzerPattern::TripleBeep: return { 3, 1200, 150, 120 };
    default:                        return { 0, 0, 0, 0 };
  }
}

} // namespace

void Buzzer::play(BuzzerPattern pattern)
{
  if (pattern == BuzzerPattern::None) {
    return;
  }

  // Une alerte déjà en cours n'est pas interrompue par une nouvelle.
  if (_activePattern != BuzzerPattern::None) {
    return;
  }

  _activePattern = pattern;
  _stepIndex = 0;

  const BuzzerPatternSpec spec = buzzerSpecFor(pattern);
  tone(_pin, spec.frequencyHz, spec.onMs);
  _nextStepMillis = millis() + spec.onMs;
}

void Buzzer::update()
{
  if (_activePattern == BuzzerPattern::None) {
    return;
  }
  if (millis() < _nextStepMillis) {
    return;
  }

  const BuzzerPatternSpec spec = buzzerSpecFor(_activePattern);
  const bool completedPhaseWasBeep = (_stepIndex % 2) == 0;

  if (completedPhaseWasBeep) {
    const uint8_t beepsCompleted = (_stepIndex / 2) + 1;

    if (beepsCompleted >= spec.beepCount) {
      _activePattern = BuzzerPattern::None;
      return;
    }

    _stepIndex++;
    _nextStepMillis = millis() + spec.offMs;
  } else {
    _stepIndex++;
    tone(_pin, spec.frequencyHz, spec.onMs);
    _nextStepMillis = millis() + spec.onMs;
  }
}

// ============================================================
// 7) ENCODEUR ROTATIF + BOUTON — navigation IHM
// ============================================================

class EncoderInput {
public:
  void begin(int pinA, int pinB, int pinButton);
  void update();

  int consumeRotationSteps();
  bool consumeShortPress();
  bool consumeLongPress();     // ENCODER_LONGPRESS_MS <= durée < CALIBRATION_HOLD_MS
  bool consumeVeryLongPress(); // durée >= CALIBRATION_HOLD_MS

  bool isButtonHeld() const { return _buttonHeld; }

private:
  int _pinA = -1;
  int _pinB = -1;
  int _pinButton = -1;

  int32_t _lastProcessedTransitions = 0;
  int32_t _accumulator = 0;
  int _pendingSteps = 0;

  bool _buttonHeld = false;
  bool _shortPressPending = false;
  bool _longPressPending = false;
  bool _veryLongPressPending = false;
  uint32_t _pressStartMillis = 0;
  uint32_t _lastButtonChangeMillis = 0;
  bool _previousButtonState = HIGH;

  static void IRAM_ATTR isrTrampoline();
};

namespace {

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t encoderTransitions = 0;
volatile uint8_t previousEncoderRawState = 0;

const int8_t kQuadratureTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

constexpr int32_t TRANSITIONS_PER_DETENT = 4; // 4 pour la plupart des encodeurs mécaniques

} // namespace

void IRAM_ATTR EncoderInput::isrTrampoline()
{
  uint8_t currentState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  uint8_t tableIndex = (previousEncoderRawState << 2) | currentState;
  int8_t movement = kQuadratureTable[tableIndex];

  if (movement != 0) {
    portENTER_CRITICAL_ISR(&encoderMux);
    encoderTransitions += movement;
    portEXIT_CRITICAL_ISR(&encoderMux);
  }

  previousEncoderRawState = currentState;
}

void EncoderInput::begin(int pinA, int pinB, int pinButton)
{
  _pinA = pinA;
  _pinB = pinB;
  _pinButton = pinButton;

  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  pinMode(_pinButton, INPUT_PULLUP);

  previousEncoderRawState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);

  attachInterrupt(digitalPinToInterrupt(_pinA), isrTrampoline, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinB), isrTrampoline, CHANGE);

  _previousButtonState = digitalRead(_pinButton);
}

void EncoderInput::update()
{
  int32_t transitionsCopy;
  portENTER_CRITICAL(&encoderMux);
  transitionsCopy = encoderTransitions;
  portEXIT_CRITICAL(&encoderMux);

  const int32_t difference = transitionsCopy - _lastProcessedTransitions;
  if (difference != 0) {
    _lastProcessedTransitions = transitionsCopy;
    _accumulator += difference;

    while (_accumulator >= TRANSITIONS_PER_DETENT) {
      _accumulator -= TRANSITIONS_PER_DETENT;
      _pendingSteps += 1;
    }
    while (_accumulator <= -TRANSITIONS_PER_DETENT) {
      _accumulator += TRANSITIONS_PER_DETENT;
      _pendingSteps -= 1;
    }
  }

  const bool currentButtonState = digitalRead(_pinButton);
  const uint32_t now = millis();

  if (currentButtonState != _previousButtonState && (now - _lastButtonChangeMillis) > 30) {
    _lastButtonChangeMillis = now;
    _previousButtonState = currentButtonState;

    if (currentButtonState == LOW) {
      _buttonHeld = true;
      _pressStartMillis = now;
    } else {
      _buttonHeld = false;

      // Classification au relâchement, selon la durée totale de l'appui.
      const uint32_t heldDuration = now - _pressStartMillis;

      if (heldDuration >= CALIBRATION_HOLD_MS) {
        _veryLongPressPending = true;
      } else if (heldDuration >= ENCODER_LONGPRESS_MS) {
        _longPressPending = true;
      } else {
        _shortPressPending = true;
      }
    }
  }
}

int EncoderInput::consumeRotationSteps()
{
  const int steps = _pendingSteps;
  _pendingSteps = 0;
  return steps;
}

bool EncoderInput::consumeShortPress()
{
  if (_shortPressPending) { _shortPressPending = false; return true; }
  return false;
}

bool EncoderInput::consumeLongPress()
{
  if (_longPressPending) { _longPressPending = false; return true; }
  return false;
}

bool EncoderInput::consumeVeryLongPress()
{
  if (_veryLongPressPending) { _veryLongPressPending = false; return true; }
  return false;
}

// ============================================================
// 8) TABLE DES BALISES SUIVIES
// ============================================================

struct TrackedBeacon {
  bool         inUse = false;
  BeaconPacket lastPacket{};
  uint32_t     lastSeenMillis = 0;
  int16_t      lastRssi = 0;
  float        lastSnr = 0.0f;

  double distanceMeters = 0.0;
  double bearingDegrees = 0.0; // relèvement absolu depuis la position du Détecteur
  bool   geometryValid = false; // false tant que le Détecteur n'a pas de fix GPS
};

class BeaconTracker {
public:
  bool handlePacket(const BeaconPacket &packet, int16_t rssi, float snr);
  void updateGeometry(const GeoPoint &ownPosition);
  bool expireStaleBeacons();

  int activeCount() const;
  int listSortedByDistance(const TrackedBeacon *out[], int maxCount) const;

  const TrackedBeacon *selectedBeacon() const;
  void selectBeacon(const TrackedBeacon *beacon);
  void selectNext();

private:
  TrackedBeacon _beacons[MAX_TRACKED_BEACONS];
  int8_t _selectedSlot = -1;

  int8_t findSlotById(uint8_t beaconId) const;
  int8_t findFreeOrOldestSlot() const;
};

int8_t BeaconTracker::findSlotById(uint8_t beaconId) const
{
  for (int8_t i = 0; i < MAX_TRACKED_BEACONS; i++) {
    if (_beacons[i].inUse && _beacons[i].lastPacket.beaconId == beaconId) {
      return i;
    }
  }
  return -1;
}

int8_t BeaconTracker::findFreeOrOldestSlot() const
{
  for (int8_t i = 0; i < MAX_TRACKED_BEACONS; i++) {
    if (!_beacons[i].inUse) {
      return i;
    }
  }

  int8_t oldestSlot = 0;
  uint32_t oldestSeen = _beacons[0].lastSeenMillis;
  for (int8_t i = 1; i < MAX_TRACKED_BEACONS; i++) {
    if (_beacons[i].lastSeenMillis < oldestSeen) {
      oldestSeen = _beacons[i].lastSeenMillis;
      oldestSlot = i;
    }
  }
  return oldestSlot;
}

bool BeaconTracker::handlePacket(const BeaconPacket &packet, int16_t rssi, float snr)
{
  int8_t slot = findSlotById(packet.beaconId);
  const bool isNewBeacon = (slot < 0);

  if (isNewBeacon) {
    slot = findFreeOrOldestSlot();
  }

  TrackedBeacon &tracked = _beacons[slot];
  tracked.inUse = true;
  tracked.lastPacket = packet;
  tracked.lastSeenMillis = millis();
  tracked.lastRssi = rssi;
  tracked.lastSnr = snr;

  if (_selectedSlot < 0) {
    _selectedSlot = slot; // première balise vue = cible par défaut
  }

  return isNewBeacon;
}

void BeaconTracker::updateGeometry(const GeoPoint &ownPosition)
{
  for (auto &beacon : _beacons) {
    if (!beacon.inUse) {
      continue;
    }

    const GeoPoint beaconPosition{
      beacon.lastPacket.latitudeE7 / 1e7,
      beacon.lastPacket.longitudeE7 / 1e7,
    };

    beacon.distanceMeters = haversineDistanceMeters(ownPosition, beaconPosition);
    beacon.bearingDegrees = initialBearingDegrees(ownPosition, beaconPosition);
    beacon.geometryValid = true;
  }
}

bool BeaconTracker::expireStaleBeacons()
{
  const uint32_t now = millis();
  bool selectedExpired = false;

  for (int8_t i = 0; i < MAX_TRACKED_BEACONS; i++) {
    if (_beacons[i].inUse && (now - _beacons[i].lastSeenMillis) > BEACON_TIMEOUT_MS) {
      _beacons[i] = TrackedBeacon{};

      if (_selectedSlot == i) {
        _selectedSlot = -1;
        selectedExpired = true;
      }
    }
  }

  if (_selectedSlot < 0) {
    for (int8_t i = 0; i < MAX_TRACKED_BEACONS; i++) {
      if (_beacons[i].inUse) {
        _selectedSlot = i;
        break;
      }
    }
  }

  return selectedExpired;
}

int BeaconTracker::activeCount() const
{
  int total = 0;
  for (const auto &beacon : _beacons) {
    if (beacon.inUse) total++;
  }
  return total;
}

int BeaconTracker::listSortedByDistance(const TrackedBeacon *out[], int maxCount) const
{
  int written = 0;

  for (const auto &beacon : _beacons) {
    if (!beacon.inUse || written >= maxCount) {
      continue;
    }

    int insertAt = written;
    while (insertAt > 0 && out[insertAt - 1]->distanceMeters > beacon.distanceMeters) {
      out[insertAt] = out[insertAt - 1];
      insertAt--;
    }
    out[insertAt] = &beacon;
    written++;
  }

  return written;
}

const TrackedBeacon *BeaconTracker::selectedBeacon() const
{
  if (_selectedSlot < 0 || !_beacons[_selectedSlot].inUse) {
    return nullptr;
  }
  return &_beacons[_selectedSlot];
}

void BeaconTracker::selectBeacon(const TrackedBeacon *beacon)
{
  if (beacon == nullptr) {
    return;
  }

  const auto index = beacon - &_beacons[0];
  if (index >= 0 && index < MAX_TRACKED_BEACONS) {
    _selectedSlot = static_cast<int8_t>(index);
  }
}

void BeaconTracker::selectNext()
{
  if (activeCount() == 0) {
    _selectedSlot = -1;
    return;
  }

  int8_t start = (_selectedSlot < 0) ? 0 : _selectedSlot;
  int8_t next = start;

  do {
    next = (next + 1) % MAX_TRACKED_BEACONS;
  } while (!_beacons[next].inUse && next != start);

  _selectedSlot = next;
}

// ============================================================
// 9) ÉCRAN TFT 320x240 — cadran de visée + liste des balises
// ============================================================

enum class DisplayScreen {
  CompassTarget,
  BeaconList,
};

struct CompassScreenData {
  bool    gpsHasFix = false;
  uint8_t satellites = 0;
  bool    headingValid = false;
  float   ownHeadingDeg = 0.0f;
  uint8_t batteryPercent = 0;
  int     activeBeaconCount = 0;

  bool    hasTarget = false;
  float   relativeBearingDeg = 0.0f; // 0-359, cap cible - cap propre
  double  targetDistanceMeters = 0.0;
  uint8_t targetBeaconId = 0;
  uint8_t targetTeamId = 0;
  BeaconStatus targetStatus = BeaconStatus::Active;
  int16_t targetRssi = 0;
  bool    targetSignalStale = false;
};

class Display {
public:
  explicit Display(SPIClass &spi)
    : _spi(spi),
      _tft(&spi, PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_RST),
      _centerCanvas(320, 130)
  {
  }

  void begin();
  void showCompassScreen(const CompassScreenData &data);
  void showBeaconList(const TrackedBeacon *beacons[], int count, int highlightIndex,
                       bool gpsHasFix, uint8_t satellites, uint8_t batteryPercent);

private:
  SPIClass &_spi;
  Adafruit_ILI9341 _tft;
  GFXcanvas16 _centerCanvas;

  void drawTopBar(bool gpsHasFix, uint8_t satellites, uint8_t batteryPercent,
                   int activeBeaconCount, const char *screenLabel);
};

namespace {

constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

constexpr int TOP_H    = 30;
constexpr int BOTTOM_H = 80;
constexpr int CENTER_H = SCREEN_H - TOP_H - BOTTOM_H; // 130 px
constexpr int BOTTOM_Y = SCREEN_H - BOTTOM_H;         // y = 160

constexpr int DIAL_CX = 160;
constexpr int DIAL_CY = 160;
constexpr int DIAL_RADIUS = 160;

constexpr int OUTER_TRIANGLE_H = 30;
constexpr int LOWER_ZONE_Y = BOTTOM_Y + OUTER_TRIANGLE_H; // 190
constexpr int LOWER_ZONE_H = 50;

constexpr int OUTER_ZONE_W = 80;
constexpr int LEFT_ZONE_X  = 0;
constexpr int RIGHT_ZONE_X = SCREEN_W - OUTER_ZONE_W; // 240
constexpr int MIDDLE_ZONE_X = OUTER_ZONE_W;           // 80
constexpr int MIDDLE_ZONE_W = SCREEN_W - 2 * OUTER_ZONE_W; // 160

constexpr int VALUE_BOX_W = 100;
constexpr int VALUE_BOX_H = 44;
constexpr int VALUE_BOX_X = (SCREEN_W - VALUE_BOX_W) / 2; // 110
constexpr int VALUE_BOX_Y = LOWER_ZONE_Y + 3;             // 193

constexpr int RED_TRIANGLE_HALF_W = 3;
constexpr int RED_TRIANGLE_HALF_H = 5;
constexpr int LEFT_TRIANGLE_X  = VALUE_BOX_X - 13;                   // 97
constexpr int RIGHT_TRIANGLE_X = VALUE_BOX_X + VALUE_BOX_W + 13;     // 223
constexpr int TRIANGLE_Y = LOWER_ZONE_Y + 25;                        // 215

constexpr int FRONT_MIN_ANGLE = 70;
constexpr int FRONT_MAX_ANGLE = 110;

constexpr uint16_t COLOR_BACKGROUND = 0x0841;
constexpr uint16_t COLOR_TOP        = 0x18E3;
constexpr uint16_t COLOR_BOTTOM     = 0x2104;
constexpr uint16_t COLOR_ZONE_SIDE  = 0x2945;
constexpr uint16_t COLOR_BORDER     = ILI9341_DARKGREY;
constexpr uint16_t COLOR_ARC        = ILI9341_CYAN;
constexpr uint16_t COLOR_TICK       = 0xBDF7;
constexpr uint16_t COLOR_NEEDLE     = ILI9341_RED;
constexpr uint16_t COLOR_CENTER     = ILI9341_YELLOW;
constexpr uint16_t COLOR_TEXT       = ILI9341_WHITE;
constexpr uint16_t COLOR_VALUE      = ILI9341_GREEN;
constexpr uint16_t COLOR_STALE      = ILI9341_ORANGE;
constexpr uint16_t COLOR_DISPLAY    = ILI9341_BLACK;
constexpr uint16_t COLOR_TRIANGLE   = ILI9341_RED;
constexpr uint16_t COLOR_ELIMINATED = ILI9341_RED;
constexpr uint16_t COLOR_ROW_SELECT = 0x39C7;

int normalizeAngle360(int angle)
{
  int normalized = angle % 360;
  if (normalized < 0) normalized += 360;
  return normalized;
}

float degToRadDisplay(float degrees) { return degrees * static_cast<float>(M_PI) / 180.0f; }

void drawThickLine(Adafruit_GFX &gfx, int x0, int y0, int x1, int y1, uint16_t color, int thickness)
{
  const int half = thickness / 2;
  for (int offset = -half; offset <= half; offset++) {
    gfx.drawLine(x0 + offset, y0, x1 + offset, y1, color);
    gfx.drawLine(x0, y0 + offset, x1, y1 + offset, color);
  }
}

void drawSemicircle(Adafruit_GFX &gfx)
{
  for (int degree = 0; degree <= 180; degree++) {
    const float rad = degToRadDisplay(static_cast<float>(degree));
    const int x = DIAL_CX - lroundf(cosf(rad) * DIAL_RADIUS);
    const int y = DIAL_CY - lroundf(sinf(rad) * DIAL_RADIUS);
    gfx.drawPixel(x, y, COLOR_ARC);

    const int x2 = DIAL_CX - lroundf(cosf(rad) * (DIAL_RADIUS - 1));
    const int y2 = DIAL_CY - lroundf(sinf(rad) * (DIAL_RADIUS - 1));
    gfx.drawPixel(x2, y2, COLOR_ARC);
  }
}

void drawTicks(Adafruit_GFX &gfx)
{
  for (int degree = 0; degree <= 180; degree += 10) {
    const float rad = degToRadDisplay(static_cast<float>(degree));
    const int tickLength = (degree % 30 == 0) ? 12 : 7;
    const int outerRadius = DIAL_RADIUS - 3;
    const int innerRadius = outerRadius - tickLength;

    const int outerX = DIAL_CX - lroundf(cosf(rad) * outerRadius);
    const int outerY = DIAL_CY - lroundf(sinf(rad) * outerRadius);
    const int innerX = DIAL_CX - lroundf(cosf(rad) * innerRadius);
    const int innerY = DIAL_CY - lroundf(sinf(rad) * innerRadius);

    const uint16_t tickColor = (degree % 30 == 0) ? ILI9341_WHITE : COLOR_TICK;
    gfx.drawLine(innerX, innerY, outerX, outerY, tickColor);
  }
}

void drawNeedle(Adafruit_GFX &gfx, int angle)
{
  const float rad = degToRadDisplay(static_cast<float>(angle));
  const int needleLength = DIAL_RADIUS - 22;
  const int needleX = DIAL_CX - lroundf(cosf(rad) * needleLength);
  const int needleY = DIAL_CY - lroundf(sinf(rad) * needleLength);

  drawThickLine(gfx, DIAL_CX, DIAL_CY, needleX, needleY, COLOR_NEEDLE, 3);
  gfx.fillCircle(DIAL_CX, DIAL_CY, 7, COLOR_CENTER);
  gfx.drawCircle(DIAL_CX, DIAL_CY, 7, ILI9341_WHITE);
}

void drawDialLabels(Adafruit_GFX &gfx)
{
  gfx.setTextSize(1);
  gfx.setTextWrap(false);
  gfx.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);

  gfx.setCursor(6, DIAL_CY - 11);
  gfx.print("G"); // gauche
  gfx.setCursor(300, DIAL_CY - 11);
  gfx.print("D"); // droite
  gfx.setCursor(151, 10);
  gfx.print("^"); // face à la cible
}

void drawTriangleLeft(Adafruit_GFX &gfx, int centerX, int centerY)
{
  gfx.fillTriangle(
    centerX - RED_TRIANGLE_HALF_W, centerY,
    centerX + RED_TRIANGLE_HALF_W, centerY - RED_TRIANGLE_HALF_H,
    centerX + RED_TRIANGLE_HALF_W, centerY + RED_TRIANGLE_HALF_H,
    COLOR_TRIANGLE);
}

void drawTriangleRight(Adafruit_GFX &gfx, int centerX, int centerY)
{
  gfx.fillTriangle(
    centerX + RED_TRIANGLE_HALF_W, centerY,
    centerX - RED_TRIANGLE_HALF_W, centerY - RED_TRIANGLE_HALF_H,
    centerX - RED_TRIANGLE_HALF_W, centerY + RED_TRIANGLE_HALF_H,
    COLOR_TRIANGLE);
}

const char *teamLabel(uint8_t teamId)
{
  static char buffer[6];
  snprintf(buffer, sizeof(buffer), "EQ%u", teamId);
  return buffer;
}

const char *statusLabel(BeaconStatus status)
{
  switch (status) {
    case BeaconStatus::Active:     return "ACTIF";
    case BeaconStatus::Eliminated: return "HORS-JEU";
    case BeaconStatus::LowBattery: return "BATT.FAIBLE";
  }
  return "?";
}

} // namespace

void Display::begin()
{
  pinMode(PIN_TFT_LED, OUTPUT);
  digitalWrite(PIN_TFT_LED, HIGH);

  _spi.begin(PIN_TFT_SCLK, PIN_TFT_MISO, PIN_TFT_MOSI, PIN_TFT_CS);

  _tft.begin(40000000);
  _tft.setRotation(1); // paysage 320x240
  _tft.fillScreen(COLOR_BACKGROUND);
}

void Display::drawTopBar(bool gpsHasFix, uint8_t satellites, uint8_t batteryPercent,
                          int activeBeaconCount, const char *screenLabel)
{
  _tft.fillRect(0, 0, SCREEN_W, TOP_H, COLOR_TOP);
  _tft.drawFastHLine(0, TOP_H - 1, SCREEN_W, COLOR_BORDER);

  _tft.setTextWrap(false);
  _tft.setTextSize(1);

  char gpsText[16];
  snprintf(gpsText, sizeof(gpsText), "GPS:%s %us",
           gpsHasFix ? "OK" : "--", static_cast<unsigned>(satellites));
  _tft.setTextColor(gpsHasFix ? ILI9341_GREEN : ILI9341_RED, COLOR_TOP);
  _tft.setCursor(6, 10);
  _tft.print(gpsText);

  _tft.setTextColor(COLOR_TEXT, COLOR_TOP);
  _tft.setCursor(118, 10);
  _tft.print(screenLabel);

  char rightText[20];
  snprintf(rightText, sizeof(rightText), "B:%u%% N:%d", batteryPercent, activeBeaconCount);
  _tft.setTextColor(COLOR_TEXT, COLOR_TOP);
  _tft.setCursor(232, 10);
  _tft.print(rightText);
}

void Display::showCompassScreen(const CompassScreenData &data)
{
  char screenLabel[16];
  if (data.headingValid) {
    snprintf(screenLabel, sizeof(screenLabel), "CIBLE H:%03d", lroundf(data.ownHeadingDeg));
  } else {
    snprintf(screenLabel, sizeof(screenLabel), "CIBLE H:---");
  }

  drawTopBar(data.gpsHasFix, data.satellites, data.batteryPercent,
             data.activeBeaconCount, screenLabel);

  // ---------------- Cadran central ----------------
  _centerCanvas.fillScreen(COLOR_BACKGROUND);

  if (data.hasTarget) {
    const int relative360 = normalizeAngle360(lroundf(data.relativeBearingDeg));
    const int displayedAngle = (relative360 <= 180) ? relative360 : (360 - relative360);

    drawSemicircle(_centerCanvas);
    drawTicks(_centerCanvas);
    drawDialLabels(_centerCanvas);
    drawNeedle(_centerCanvas, displayedAngle);
  } else {
    _centerCanvas.setTextSize(2);
    _centerCanvas.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
    _centerCanvas.setCursor(70, 55);
    _centerCanvas.print("AUCUNE CIBLE");
  }

  _tft.drawRGBBitmap(0, TOP_H, _centerCanvas.getBuffer(), SCREEN_W, CENTER_H);

  // ---------------- Bandeau inférieur ----------------
  _tft.fillRect(0, BOTTOM_Y, SCREEN_W, BOTTOM_H, COLOR_BACKGROUND);

  _tft.fillTriangle(0, BOTTOM_Y, 0, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_Y, COLOR_ZONE_SIDE);
  _tft.drawTriangle(0, BOTTOM_Y, 0, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_Y, COLOR_BORDER);
  _tft.fillRect(LEFT_ZONE_X, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_H, COLOR_ZONE_SIDE);
  _tft.drawRect(LEFT_ZONE_X, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_H, COLOR_BORDER);

  _tft.fillTriangle(SCREEN_W - 1, BOTTOM_Y, SCREEN_W - 1, LOWER_ZONE_Y, RIGHT_ZONE_X, LOWER_ZONE_Y, COLOR_ZONE_SIDE);
  _tft.drawTriangle(SCREEN_W - 1, BOTTOM_Y, SCREEN_W - 1, LOWER_ZONE_Y, RIGHT_ZONE_X, LOWER_ZONE_Y, COLOR_BORDER);
  _tft.fillRect(RIGHT_ZONE_X, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_H, COLOR_ZONE_SIDE);
  _tft.drawRect(RIGHT_ZONE_X, LOWER_ZONE_Y, OUTER_ZONE_W, LOWER_ZONE_H, COLOR_BORDER);

  _tft.fillRect(MIDDLE_ZONE_X, LOWER_ZONE_Y, MIDDLE_ZONE_W, LOWER_ZONE_H, COLOR_BOTTOM);
  _tft.drawRect(MIDDLE_ZONE_X, LOWER_ZONE_Y, MIDDLE_ZONE_W, LOWER_ZONE_H, COLOR_BORDER);

  // Zone gauche : identité de la cible.
  _tft.setTextWrap(false);
  _tft.setTextSize(1);
  _tft.setTextColor(COLOR_TEXT, COLOR_ZONE_SIDE);
  if (data.hasTarget) {
    char idLine[10];
    snprintf(idLine, sizeof(idLine), "#%u", data.targetBeaconId);
    _tft.setCursor(LEFT_ZONE_X + 10, LOWER_ZONE_Y + 8);
    _tft.print(idLine);
    _tft.setCursor(LEFT_ZONE_X + 10, LOWER_ZONE_Y + 20);
    _tft.print(teamLabel(data.targetTeamId));

    const uint16_t statusColor = (data.targetStatus == BeaconStatus::Eliminated)
      ? COLOR_ELIMINATED
      : (data.targetSignalStale ? COLOR_STALE : ILI9341_GREEN);
    _tft.setTextColor(statusColor, COLOR_ZONE_SIDE);
    _tft.setCursor(LEFT_ZONE_X + 4, LOWER_ZONE_Y + 34);
    _tft.print(data.targetSignalStale ? "FAIBLE" : statusLabel(data.targetStatus));
  }

  // Zone droite : distance et niveau de signal.
  _tft.setTextColor(COLOR_TEXT, COLOR_ZONE_SIDE);
  if (data.hasTarget) {
    char distLine[12];
    if (data.targetDistanceMeters >= 1000.0) {
      snprintf(distLine, sizeof(distLine), "%.2fkm", data.targetDistanceMeters / 1000.0);
    } else {
      snprintf(distLine, sizeof(distLine), "%.0fm", data.targetDistanceMeters);
    }
    _tft.setCursor(RIGHT_ZONE_X + 6, LOWER_ZONE_Y + 8);
    _tft.print(distLine);

    char rssiLine[12];
    snprintf(rssiLine, sizeof(rssiLine), "%ddBm", data.targetRssi);
    _tft.setCursor(RIGHT_ZONE_X + 6, LOWER_ZONE_Y + 20);
    _tft.print(rssiLine);
  }

  // Zone centrale : relèvement numérique.
  _tft.fillRoundRect(VALUE_BOX_X, VALUE_BOX_Y, VALUE_BOX_W, VALUE_BOX_H, 4, COLOR_DISPLAY);
  _tft.drawRoundRect(VALUE_BOX_X, VALUE_BOX_Y, VALUE_BOX_W, VALUE_BOX_H, 4, ILI9341_LIGHTGREY);

  char valueText[8] = "---";
  int relative360 = 0;
  if (data.hasTarget) {
    relative360 = normalizeAngle360(lroundf(data.relativeBearingDeg));
    snprintf(valueText, sizeof(valueText), "%03d", relative360);
  }

  _tft.setTextSize(3);
  _tft.setTextColor(data.hasTarget ? COLOR_VALUE : ILI9341_DARKGREY, COLOR_DISPLAY);

  int16_t boundsX, boundsY;
  uint16_t textWidth, textHeight;
  _tft.getTextBounds(valueText, 0, 0, &boundsX, &boundsY, &textWidth, &textHeight);

  const int textX = VALUE_BOX_X + (VALUE_BOX_W - textWidth) / 2 - 3;
  const int textY = VALUE_BOX_Y + (VALUE_BOX_H - textHeight) / 2;
  _tft.setCursor(textX, textY);
  _tft.print(valueText);

  _tft.setTextSize(1);
  _tft.setCursor(VALUE_BOX_X + VALUE_BOX_W - 12, VALUE_BOX_Y + 6);
  _tft.print("o");

  // Petits triangles rouges indiquant si la cible est à gauche, à droite,
  // en face ou dans le dos, selon le relèvement relatif.
  if (data.hasTarget) {
    if (relative360 >= 180) {
      drawTriangleLeft(_tft, LEFT_TRIANGLE_X, TRIANGLE_Y);
      drawTriangleRight(_tft, RIGHT_TRIANGLE_X, TRIANGLE_Y);
    } else if (relative360 >= FRONT_MIN_ANGLE && relative360 <= FRONT_MAX_ANGLE) {
      drawTriangleRight(_tft, LEFT_TRIANGLE_X, TRIANGLE_Y);
      drawTriangleLeft(_tft, RIGHT_TRIANGLE_X, TRIANGLE_Y);
    } else if (relative360 < FRONT_MIN_ANGLE) {
      drawTriangleLeft(_tft, LEFT_TRIANGLE_X, TRIANGLE_Y);
      drawTriangleLeft(_tft, RIGHT_TRIANGLE_X, TRIANGLE_Y);
    } else {
      drawTriangleRight(_tft, LEFT_TRIANGLE_X, TRIANGLE_Y);
      drawTriangleRight(_tft, RIGHT_TRIANGLE_X, TRIANGLE_Y);
    }
  }
}

void Display::showBeaconList(const TrackedBeacon *beacons[], int count, int highlightIndex,
                              bool gpsHasFix, uint8_t satellites, uint8_t batteryPercent)
{
  drawTopBar(gpsHasFix, satellites, batteryPercent, count, "LISTE");

  constexpr int LIST_Y = TOP_H;
  constexpr int LIST_H = SCREEN_H - TOP_H;
  constexpr int ROW_H = 30;
  constexpr int VISIBLE_ROWS = LIST_H / ROW_H;

  _tft.fillRect(0, LIST_Y, SCREEN_W, LIST_H, COLOR_BACKGROUND);

  if (count == 0) {
    _tft.setTextSize(2);
    _tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
    _tft.setCursor(50, LIST_Y + 90);
    _tft.print("Aucune balise recue");
    return;
  }

  int firstVisible = 0;
  if (highlightIndex >= VISIBLE_ROWS) {
    firstVisible = highlightIndex - VISIBLE_ROWS + 1;
  }

  for (int row = 0; row < VISIBLE_ROWS; row++) {
    const int index = firstVisible + row;
    if (index >= count) {
      break;
    }

    const TrackedBeacon *beacon = beacons[index];
    const int rowY = LIST_Y + row * ROW_H;
    const bool selected = (index == highlightIndex);

    _tft.fillRect(0, rowY, SCREEN_W, ROW_H, selected ? COLOR_ROW_SELECT : COLOR_BACKGROUND);
    _tft.drawFastHLine(0, rowY + ROW_H - 1, SCREEN_W, COLOR_BORDER);

    const BeaconType type = static_cast<BeaconType>(beacon->lastPacket.type);
    const BeaconStatus status = static_cast<BeaconStatus>(beacon->lastPacket.status);

    char line1[24];
    snprintf(line1, sizeof(line1), "#%-3u %s %s",
             beacon->lastPacket.beaconId,
             teamLabel(beacon->lastPacket.teamId),
             (type == BeaconType::StaticBeacon) ? "BPS" : "BEM");

    char line2[24];
    if (beacon->distanceMeters >= 1000.0) {
      snprintf(line2, sizeof(line2), "%.2fkm  %ddBm", beacon->distanceMeters / 1000.0, beacon->lastRssi);
    } else {
      snprintf(line2, sizeof(line2), "%.0fm  %ddBm", beacon->distanceMeters, beacon->lastRssi);
    }

    _tft.setTextSize(1);
    _tft.setTextColor(COLOR_TEXT, selected ? COLOR_ROW_SELECT : COLOR_BACKGROUND);
    _tft.setCursor(8, rowY + 6);
    _tft.print(line1);
    _tft.setCursor(8, rowY + 18);
    _tft.print(line2);

    const uint16_t statusColor = (status == BeaconStatus::Eliminated) ? COLOR_ELIMINATED
      : (status == BeaconStatus::LowBattery ? COLOR_STALE : ILI9341_GREEN);
    _tft.setTextColor(statusColor, selected ? COLOR_ROW_SELECT : COLOR_BACKGROUND);
    _tft.setCursor(190, rowY + 12);
    _tft.print(statusLabel(status));
  }
}

// ============================================================
// 10) OBJETS GLOBAUX ET ORCHESTRATION (setup / loop)
// ============================================================

SPIClass sharedSpi(FSPI);
HardwareSerial gpsSerial(1);

Display display(sharedSpi);
LoRaRadio loraRadio(sharedSpi, PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
GpsReceiver gps;
CompassImu compass(Wire);
BeaconTracker tracker;
EncoderInput encoder;
Buzzer buzzer(PIN_BUZZER);

DisplayScreen currentScreen = DisplayScreen::CompassTarget;
int listHighlightIndex = 0;

bool loraReady = false;
bool compassReady = false;

float ownHeadingDeg = 0.0f;
bool ownHeadingValid = false;

bool calibratingCompass = false;
uint32_t calibrationEndMillis = 0;

uint8_t batteryPercent = 0;

uint32_t lastImuReadMillis = 0;
uint32_t lastGeometryUpdateMillis = 0;
uint32_t lastExpiryCheckMillis = 0;
uint32_t lastBatteryReadMillis = 0;
uint32_t lastDisplayRefreshMillis = 0;

constexpr uint32_t IMU_READ_INTERVAL_MS = 50;
constexpr uint32_t GEOMETRY_UPDATE_INTERVAL_MS = 500;
constexpr uint32_t EXPIRY_CHECK_INTERVAL_MS = 1000;
constexpr uint32_t BATTERY_READ_INTERVAL_MS = 5000;

void updateBatteryPercent()
{
  const int raw = analogRead(PIN_BATTERY_ADC);
  const float measuredVoltage = (raw / 4095.0f) * 3.3f * BATTERY_DIVIDER_RATIO;

  float percent = (measuredVoltage - BATTERY_EMPTY_V) /
                   (BATTERY_FULL_V - BATTERY_EMPTY_V) * 100.0f;
  percent = constrain(percent, 0.0f, 100.0f);

  batteryPercent = static_cast<uint8_t>(lroundf(percent));
}

void handleIncomingLoRaPackets()
{
  LoRaReceivedPacket received;
  if (!loraRadio.poll(received)) {
    return;
  }

  if (received.length != sizeof(BeaconPacket)) {
    return; // taille inattendue : version de protocole différente ou bruit
  }

  BeaconPacket packet;
  memcpy(&packet, received.data, sizeof(BeaconPacket));

  if (packet.protocolVersion != PROTOCOL_VERSION) {
    return;
  }

  const bool isNewBeacon = tracker.handlePacket(packet, received.rssi, received.snr);
  if (isNewBeacon) {
    buzzer.play(BuzzerPattern::SingleBeep);
  }
}

void updateCompassHeading()
{
  if (!compassReady) {
    return;
  }

  const ImuSample sample = compass.read();

  if (calibratingCompass) {
    compass.feedCalibrationSample(sample);
    if (millis() >= calibrationEndMillis) {
      compass.finishCalibration();
      calibratingCompass = false;
      buzzer.play(BuzzerPattern::TripleBeep);
    }
    return;
  }

  const float heading = compass.headingDegrees(sample);
  ownHeadingValid = !isnan(heading);
  if (ownHeadingValid) {
    ownHeadingDeg = heading;
  }
}

void updateBeaconGeometry()
{
  const GpsFix &fix = gps.fix();
  if (!fix.hasFix) {
    return;
  }

  const GeoPoint ownPosition{ fix.latitudeDeg, fix.longitudeDeg };
  tracker.updateGeometry(ownPosition);
}

void handleEncoderInput()
{
  if (encoder.consumeVeryLongPress()) {
    // Appui très long (>= CALIBRATION_HOLD_MS) : calibration du
    // magnétomètre (rotation complète à effectuer pendant
    // CALIBRATION_DURATION_MS).
    if (compassReady && !calibratingCompass) {
      calibratingCompass = true;
      calibrationEndMillis = millis() + CALIBRATION_DURATION_MS;
      compass.startCalibration();
    }
    return;
  }

  if (encoder.consumeLongPress()) {
    currentScreen = (currentScreen == DisplayScreen::CompassTarget)
      ? DisplayScreen::BeaconList
      : DisplayScreen::CompassTarget;
    listHighlightIndex = 0;
    return;
  }

  const int steps = encoder.consumeRotationSteps();

  if (currentScreen == DisplayScreen::BeaconList) {
    if (steps != 0) {
      const int count = tracker.activeCount();
      if (count > 0) {
        listHighlightIndex = constrain(listHighlightIndex + steps, 0, count - 1);
      }
    }

    if (encoder.consumeShortPress()) {
      const TrackedBeacon *sorted[MAX_TRACKED_BEACONS];
      const int count = tracker.listSortedByDistance(sorted, MAX_TRACKED_BEACONS);
      if (listHighlightIndex < count) {
        tracker.selectBeacon(sorted[listHighlightIndex]);
        currentScreen = DisplayScreen::CompassTarget;
      }
    }
  } else {
    if (steps != 0) {
      tracker.selectNext();
    }
    encoder.consumeShortPress(); // pas d'action dédiée sur cet écran
  }
}

void renderDisplay()
{
  if (currentScreen == DisplayScreen::CompassTarget) {
    CompassScreenData data;
    data.gpsHasFix = gps.fix().hasFix;
    data.satellites = gps.fix().satellites;
    data.headingValid = ownHeadingValid;
    data.ownHeadingDeg = ownHeadingDeg;
    data.batteryPercent = batteryPercent;
    data.activeBeaconCount = tracker.activeCount();

    const TrackedBeacon *target = tracker.selectedBeacon();
    if (target != nullptr && target->geometryValid && ownHeadingValid) {
      data.hasTarget = true;
      data.relativeBearingDeg = fmodf(
        static_cast<float>(target->bearingDegrees) - ownHeadingDeg + 360.0f, 360.0f);
      data.targetDistanceMeters = target->distanceMeters;
      data.targetBeaconId = target->lastPacket.beaconId;
      data.targetTeamId = target->lastPacket.teamId;
      data.targetStatus = static_cast<BeaconStatus>(target->lastPacket.status);
      data.targetRssi = target->lastRssi;
      data.targetSignalStale = (millis() - target->lastSeenMillis) > BEACON_STALE_MS;
    }

    display.showCompassScreen(data);
  } else {
    const TrackedBeacon *sorted[MAX_TRACKED_BEACONS];
    const int count = tracker.listSortedByDistance(sorted, MAX_TRACKED_BEACONS);
    if (count > 0) {
      listHighlightIndex = constrain(listHighlightIndex, 0, count - 1);
    }
    display.showBeaconList(sorted, count, listHighlightIndex,
                            gps.fix().hasFix, gps.fix().satellites, batteryPercent);
  }
}

void setup()
{
  Serial.begin(115200);

  encoder.begin(PIN_ENC_A, PIN_ENC_B, PIN_ENC_SW);
  buzzer.begin();

  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  gps.begin(gpsSerial);

  analogReadResolution(12);

  compassReady = compass.begin();
  if (!compassReady) {
    Serial.println("MPU9265/9250 introuvable sur le bus I2C.");
  }

  loraReady = loraRadio.begin(LORA_FREQUENCY_HZ, LORA_SPREADING_FACTOR,
                               LORA_BANDWIDTH_INDEX, LORA_CODING_RATE, LORA_SYNC_WORD);
  if (!loraReady) {
    Serial.println("Module SX1278 introuvable (verifier le cablage SPI).");
  }

  display.begin();

  Serial.println("Detecteur initialise.");
}

void loop()
{
  encoder.update();
  gps.update();
  buzzer.update();

  handleEncoderInput();

  if (loraReady) {
    handleIncomingLoRaPackets();
  }

  const uint32_t now = millis();

  if (compassReady && (now - lastImuReadMillis) >= IMU_READ_INTERVAL_MS) {
    lastImuReadMillis = now;
    updateCompassHeading();
  }

  if ((now - lastGeometryUpdateMillis) >= GEOMETRY_UPDATE_INTERVAL_MS) {
    lastGeometryUpdateMillis = now;
    updateBeaconGeometry();
  }

  if ((now - lastExpiryCheckMillis) >= EXPIRY_CHECK_INTERVAL_MS) {
    lastExpiryCheckMillis = now;
    if (tracker.expireStaleBeacons()) {
      buzzer.play(BuzzerPattern::DoubleBeep);
    }
  }

  if ((now - lastBatteryReadMillis) >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadMillis = now;
    updateBatteryPercent();
  }

  if ((now - lastDisplayRefreshMillis) >= DISPLAY_REFRESH_MS) {
    lastDisplayRefreshMillis = now;
    renderDisplay();
  }
}
