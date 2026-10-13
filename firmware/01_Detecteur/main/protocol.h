#pragma once

#include <cstdint>
#include <cmath>

// ============================================================
// Format de paquet applicatif échangé sur la radio LoRa entre
// les balises (BEM/BPS) et le Détecteur.
//
// Ce format est le format canonique du projet : une copie de
// référence (documentation, non compilée) se trouve dans
// firmware/shared/protocols/tracker_protocol.h. Toute évolution
// de ce struct doit être répercutée aux deux endroits, ainsi
// que dans les firmwares BEM et BPS lorsqu'ils seront écrits.
//
// L'intégrité du paquet radio est vérifiée par le CRC matériel
// du SX1278 (voir sx1278.cpp) : aucun CRC applicatif redondant
// n'est ajouté ici.
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

// ============================================================
// Géométrie : distance et relèvement entre deux coordonnées
// WGS84 (formules de navigation sphérique, précision suffisante
// à l'échelle d'un terrain de jeu).
// ============================================================

struct GeoPoint {
  double latitudeDeg;
  double longitudeDeg;
};

inline double degToRad(double deg) { return deg * M_PI / 180.0; }
inline double radToDeg(double rad) { return rad * 180.0 / M_PI; }

// Distance en mètres (formule haversine).
inline double haversineDistanceMeters(const GeoPoint &from, const GeoPoint &to)
{
  constexpr double EARTH_RADIUS_M = 6371000.0;

  const double lat1 = degToRad(from.latitudeDeg);
  const double lat2 = degToRad(to.latitudeDeg);
  const double dLat = degToRad(to.latitudeDeg - from.latitudeDeg);
  const double dLon = degToRad(to.longitudeDeg - from.longitudeDeg);

  const double a = sin(dLat / 2) * sin(dLat / 2) +
                    cos(lat1) * cos(lat2) *
                    sin(dLon / 2) * sin(dLon / 2);
  const double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS_M * c;
}

// Relèvement initial en degrés [0, 360) de "from" vers "to".
inline double initialBearingDegrees(const GeoPoint &from, const GeoPoint &to)
{
  const double lat1 = degToRad(from.latitudeDeg);
  const double lat2 = degToRad(to.latitudeDeg);
  const double dLon = degToRad(to.longitudeDeg - from.longitudeDeg);

  const double y = sin(dLon) * cos(lat2);
  const double x = cos(lat1) * sin(lat2) -
                    sin(lat1) * cos(lat2) * cos(dLon);

  double bearing = radToDeg(atan2(y, x));
  bearing = fmod(bearing + 360.0, 360.0);
  return bearing;
}
