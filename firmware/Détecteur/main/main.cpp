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

} // namespace

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
