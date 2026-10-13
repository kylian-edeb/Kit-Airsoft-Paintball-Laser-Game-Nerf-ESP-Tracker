#pragma once

#include <Arduino.h> // millis()

#include "protocol.h"
#include "config.h"

// ============================================================
// Table des balises suivies par le Détecteur : une entrée par
// balise (BEM ou BPS) vue récemment sur la radio LoRa, avec sa
// distance et son relèvement calculés depuis la position GPS
// courante du Détecteur.
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
  // Traite un paquet reçu ; retourne true si la balise n'était pas déjà
  // suivie (pour déclencher un bip de "nouvelle balise détectée").
  bool handlePacket(const BeaconPacket &packet, int16_t rssi, float snr);

  // Recalcule distance/relèvement de toutes les balises actives depuis
  // la position GPS courante du Détecteur.
  void updateGeometry(const GeoPoint &ownPosition);

  // Retire les balises silencieuses depuis plus de BEACON_TIMEOUT_MS.
  // Retourne true si c'est la cible sélectionnée qui vient d'expirer.
  bool expireStaleBeacons();

  int activeCount() const;

  // Remplit `out` avec les balises actives triées par distance croissante
  // (les plus proches en premier) ; retourne le nombre d'entrées écrites.
  int listSortedByDistance(const TrackedBeacon *out[], int maxCount) const;

  const TrackedBeacon *selectedBeacon() const;
  void selectBeacon(const TrackedBeacon *beacon);
  void selectNext(); // cible suivante (cycle) parmi les balises actives

private:
  TrackedBeacon _beacons[MAX_TRACKED_BEACONS];
  int8_t _selectedSlot = -1;

  int8_t findSlotById(uint8_t beaconId) const;
  int8_t findFreeOrOldestSlot() const;
};
