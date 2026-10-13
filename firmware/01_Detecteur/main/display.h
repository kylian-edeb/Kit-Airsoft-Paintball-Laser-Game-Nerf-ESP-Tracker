#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include "protocol.h"
#include "tracker.h"

// ============================================================
// Interface graphique 320x240 (paysage) du Détecteur, sur deux
// écrans basculables par l'encodeur :
//
//  - CompassTarget : cadran de relèvement vers la cible
//    sélectionnée (repris du gabarit "cadran rotatif" proposé
//    initialement, réutilisé ici comme indicateur gauche/
//    centre/droite plutôt que comme boussole 360°).
//  - BeaconList : liste des balises actives triées par distance.
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
  bool    targetSignalStale = false; // pas de paquet récent (> BEACON_STALE_MS)
};

class Display {
public:
  explicit Display(SPIClass &spi);

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
