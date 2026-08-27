#pragma once

#include <Arduino.h>

// ============================================================
// Parseur NMEA minimal pour le GPS u-blox NEO-6M, sans
// dépendance externe (TinyGPSPlus n'étant pas garanti présent
// dans le registre de composants ESP-IDF). Traite les trames
// $--GGA (fix, satellites, altitude) et $--RMC (position,
// vitesse, cap), avec vérification du checksum.
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
  void begin(HardwareSerial &serial);

  // À appeler à chaque tour de boucle : lit tous les octets
  // disponibles sur l'UART et met à jour l'état interne.
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
