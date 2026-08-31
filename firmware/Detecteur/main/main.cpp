/*
  Détecteur — unité principale du Kit de Détection Tactique.

  Reçoit en LoRa les paquets télémétrie des balises (BEM/BPS),
  calcule leur relèvement et leur distance depuis la position
  GPS courante, et affiche un cadran de visée sur l'écran TFT.

  Compilé en ESP-IDF avec le composant arduino-esp32 (voir
  main/idf_component.yml et sdkconfig.defaults) : setup()/loop()
  sont fournis par ce composant, qui génère app_main() lui-même.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <cmath>
#include <cstring>

#include "config.h"
#include "protocol.h"
#include "sx1278.h"
#include "gps_nmea.h"
#include "compass_mpu.h"
#include "buzzer.h"
#include "encoder_input.h"
#include "tracker.h"
#include "display.h"

namespace {

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

#if DETECTOR_DEBUG_SIMULATION
// Position fixe arbitraire servant de "position du Détecteur" tant que le
// GPS réel n'a pas de fix : uniquement pour pouvoir calculer un relèvement
// et une distance vers les balises simulées.
constexpr GeoPoint DEBUG_OWN_POSITION{ 48.8566, 2.3522 };
constexpr uint32_t DEBUG_BEACON_UPDATE_MS = 500;
uint32_t lastDebugInjectMillis = 0;

void injectFakeBeacon(uint8_t id, uint8_t team, BeaconStatus status,
                       float bearingDeg, float distanceM)
{
  // Déplace la balise fictive à `distanceM` mètres de DEBUG_OWN_POSITION,
  // dans la direction `bearingDeg` (approximation plane, largement
  // suffisante à l'échelle d'un terrain de jeu).
  const double bearingRad = degToRad(bearingDeg);
  const double dLat = (distanceM * cos(bearingRad)) / 111320.0;
  const double dLon = (distanceM * sin(bearingRad)) /
    (111320.0 * cos(degToRad(DEBUG_OWN_POSITION.latitudeDeg)));

  BeaconPacket packet{};
  packet.protocolVersion = PROTOCOL_VERSION;
  packet.beaconId = id;
  packet.teamId = team;
  packet.type = static_cast<uint8_t>(BeaconType::MobileBeacon);
  packet.status = static_cast<uint8_t>(status);
  packet.latitudeE7 = static_cast<int32_t>((DEBUG_OWN_POSITION.latitudeDeg + dLat) * 1e7);
  packet.longitudeE7 = static_cast<int32_t>((DEBUG_OWN_POSITION.longitudeDeg + dLon) * 1e7);
  packet.altitudeM = 100;
  packet.satellites = 8;
  packet.batteryPercent = 70;
  packet.uptimeSeconds = millis() / 1000;
  packet.sequenceNumber = static_cast<uint16_t>(millis() / DEBUG_BEACON_UPDATE_MS);

  tracker.handlePacket(packet, -60, 8.0f);
}

void injectDebugBeacons()
{
  const float t = millis() / 1000.0f;

  // Trois balises fictives dont le relèvement tourne lentement et la
  // distance varie, pour faire bouger l'aiguille du cadran et exercer
  // toute la plage gauche/centre/droite/dos sans matériel radio réel.
  injectFakeBeacon(101, 1, BeaconStatus::Active,
                    fmodf(t * 20.0f, 360.0f), 40.0f + 20.0f * sinf(t * 0.3f));
  injectFakeBeacon(102, 2, BeaconStatus::Eliminated,
                    fmodf(t * 20.0f + 130.0f, 360.0f), 90.0f);
  injectFakeBeacon(103, 1, BeaconStatus::LowBattery,
                    fmodf(t * 20.0f + 250.0f, 360.0f), 220.0f);
}
#endif // DETECTOR_DEBUG_SIMULATION

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
    // Paquet d'une autre taille : ignoré (version de protocole différente
    // ou bruit radio ayant passé le CRC matériel par coïncidence).
    return;
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
  if (compassReady) {
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
    return;
  }

#if DETECTOR_DEBUG_SIMULATION
  // Pas de boussole réelle détectée : cap simulé qui balaie lentement les
  // 360°, uniquement pour pouvoir tester le rendu du cadran sur banc.
  ownHeadingValid = true;
  ownHeadingDeg = fmodf(millis() / 60.0f, 360.0f);
#endif
}

void updateBeaconGeometry()
{
  const GpsFix &fix = gps.fix();
  if (fix.hasFix) {
    const GeoPoint ownPosition{ fix.latitudeDeg, fix.longitudeDeg };
    tracker.updateGeometry(ownPosition);
    return;
  }

#if DETECTOR_DEBUG_SIMULATION
  // Pas de fix GPS réel : position simulée pour pouvoir quand même calculer
  // un relèvement/une distance vers les balises (réelles ou simulées).
  tracker.updateGeometry(DEBUG_OWN_POSITION);
#endif
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

} // namespace

void setup()
{
  Serial.begin(115200);

#if DETECTOR_DEBUG_SIMULATION
  Serial.println("################################################");
  Serial.println("## MODE DEBUG ACTIF : balises simulees en cours");
  Serial.println("## Repasser DETECTOR_DEBUG_SIMULATION a 0 (config.h)");
  Serial.println("## avant toute utilisation en jeu reel.");
  Serial.println("################################################");
#endif

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
    Serial.println("Module SX1278 introuvable (vérifier le câblage SPI).");
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

  if ((compassReady || DETECTOR_DEBUG_SIMULATION) &&
      (now - lastImuReadMillis) >= IMU_READ_INTERVAL_MS) {
    lastImuReadMillis = now;
    updateCompassHeading();
  }

  if ((now - lastGeometryUpdateMillis) >= GEOMETRY_UPDATE_INTERVAL_MS) {
    lastGeometryUpdateMillis = now;
    updateBeaconGeometry();
  }

#if DETECTOR_DEBUG_SIMULATION
  if ((now - lastDebugInjectMillis) >= DEBUG_BEACON_UPDATE_MS) {
    lastDebugInjectMillis = now;
    injectDebugBeacons();
  }
#endif

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

  // loop() ne bloque jamais ailleurs (pas de delay() dans les fonctions
  // ci-dessus) : sans ce yield, la tâche loopTask (priorité 1) monopolise
  // le cœur 1 en continu et la tâche idle de ce cœur (priorité 0) n'est
  // jamais programmée pour réarmer son chien de garde — d'où le
  // "task_wdt ... IDLE1" observé après quelques secondes. 1 ms suffit
  // largement compte tenu des intervalles (le plus court est 50 ms).
  delay(1);
}
