#include "gps_nmea.h"

#include <cstdlib>
#include <cstring>

void GpsReceiver::begin(HardwareSerial &serial)
{
  _serial = &serial;
  _lineLen = 0;
}

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
      // Ligne anormalement longue (bruit série) : on l'abandonne.
      _lineLen = 0;
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

  // Les préfixes talker peuvent être GP (GPS seul) ou GN (multi-GNSS)
  // selon la configuration du module u-blox.
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
