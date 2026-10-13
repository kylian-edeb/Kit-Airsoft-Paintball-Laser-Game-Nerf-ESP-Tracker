// ============================================================
// Copie de référence (documentation) du format de paquet LoRa
// applicatif échangé entre les balises (BEM/BPS) et le Détecteur.
//
// Ce fichier n'est PAS compilé : c'est une référence partagée pour
// que les firmwares BEM et BPS (à écrire) implémentent exactement
// le même format que celui utilisé par le Détecteur. La version
// réellement compilée vit dans :
//   firmware/Détecteur/main/protocol.h
//
// Toute évolution du format doit être répercutée aux deux endroits.
// ============================================================

#pragma once

#include <cstdint>

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

// L'intégrité du paquet radio est vérifiée par le CRC matériel du
// SX1278 (RegModemConfig2 / CrcOn + IRQ PayloadCrcError), pas par un
// CRC applicatif redondant.
