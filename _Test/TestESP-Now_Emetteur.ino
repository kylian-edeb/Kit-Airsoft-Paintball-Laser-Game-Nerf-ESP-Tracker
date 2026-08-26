#include <WiFi.h>
#include <esp_now.h>
#include <TinyGPS++.h>

// =====================================================
// GPS
// =====================================================

constexpr int GPS_RX_PIN = 16;  // NEO-6M TX -> ESP32 GPIO16
constexpr int GPS_TX_PIN = 17;  // NEO-6M RX -> ESP32 GPIO17

constexpr uint32_t GPS_DEFAULT_BAUD = 9600;
constexpr uint32_t GPS_TARGET_BAUD  = 38400;

HardwareSerial GPS(2);
TinyGPSPlus gps;

// Champ UTC de la phrase GGA
TinyGPSCustom ggaTime(gps, "GPGGA", 1);

// Diagnostic GGA
uint32_t lastGgaUs = 0;
uint32_t ggaCount = 0;
uint32_t windowStartMs = 0;

// =====================================================
// PAQUET ESP-NOW
// =====================================================

struct __attribute__((packed)) PositionPacket {
  uint32_t nodeId;

  int32_t latitude;     // degrés x 10 000 000
  int32_t longitude;    // degrés x 10 000 000

  int16_t altitude;     // mètres
  int16_t heading;      // degrés

  uint8_t team;
  uint8_t etat;

  uint32_t timestamp;   // millis() de l’émetteur
};

static_assert(sizeof(PositionPacket) == 22,
              "Taille PositionPacket incorrecte");

PositionPacket packet = {};

// =====================================================
// ESP-NOW
// =====================================================

uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

constexpr uint32_t SEND_PERIOD_MS = 450;
uint32_t previousSend = 0;

volatile bool sendResultAvailable = false;
volatile bool sendSucceeded = false;

void onDataSent(const wifi_tx_info_t *txInfo,
                esp_now_send_status_t status)
{
  // Ne pas faire de Serial.print() dans le callback
  sendSucceeded =
      (status == ESP_NOW_SEND_SUCCESS);

  sendResultAvailable = true;
}

// =====================================================
// ENVOI D’UNE COMMANDE UBX
// =====================================================

void sendUBX(uint8_t messageClass,
             uint8_t messageId,
             const uint8_t *payload,
             uint16_t *ayloadLength)
{
  uint8_t checksumA = 0;
  uint8_t checksumB = 0;

  auto sendCheckedByte = uint8_t value {
    GPS.write(value);
    checksumA += value;
    checksumB += checksumA;
  };

  GPS.write(0xB5);
  GPS.write(0x62);

  sendCheckedByte(messageClass);
  sendCheckedByte(messageId);
  sendCheckedByte(payloadLength & 0xFF);
  sendCheckedByte((payloadLength >> 8) & 0xFF);

  for (uint16_t i = 0; i < payloadLength; i++) {
    sendCheckedByte(payload[i]);
  }

  GPS.write(checksumA);
  GPS.write(checksumB);
  GPS.flush();
}

// =====================================================
// CONFIGURATION DU DÉBIT UART GPS
// =====================================================

void configureGPSBaud38400()
{
  // UBX-CFG-PRT pour UART1 :
  // 8 bits, pas de parité, 1 stop bit
  // Entrée/sortie UBX + NMEA
  const uint8_t payload[] = {
    0x01,                   // portID : UART1
    0x00,                   // réservé

    0x00, 0x00,             // txReady

    0xD0, 0x08, 0x00, 0x00, // mode : 8N1

    0x00, 0x96, 0x00, 0x00, // 38400 bauds

    0x03, 0x00,             // entrée : UBX + NMEA
    0x03, 0x00,             // sortie : UBX + NMEA

    0x00, 0x00,             // flags
    0x00, 0x00              // réservé
  };

  sendUBX(0x06, 0x00, payload, sizeof(payload));
}

// =====================================================
// CONFIGURATION 4 HZ
// =====================================================

void configureGPS4Hz()
{
  const uint8_t payload[] = {
    0xFA, 0x00, // 250 ms
    0x01, 0x00, // une navigation par mesure
    0x01, 0x00  // référence temporelle GPS
  };

  sendUBX(0x06, 0x08, payload, sizeof(payload));

  Serial.println("GPS configure a 4 Hz");
}

// =====================================================
// CONFIGURATION DES PHRASES NMEA
// =====================================================

void configureNmeaMessage(uint8_t messageId,
                          uint8_t rate)
{
  // Classe NMEA standard : 0xF0
  const uint8_t payload[] = {
    0xF0,
    messageId,
    rate
  };

  sendUBX(0x06, 0x01, payload, sizeof(payload));
}

void configureNmeaOutput()
{
  // GGA et RMC conservés
  configureNmeaMessage(0x00, 1); // GGA
  configureNmeaMessage(0x04, 1); // RMC

  // Messages inutiles désactivés
  configureNmeaMessage(0x01, 0); // GLL
  configureNmeaMessage(0x02, 0); // GSA
  configureNmeaMessage(0x03, 0); // GSV
  configureNmeaMessage(0x05, 0); // VTG

  Serial.println("NMEA limite a GGA et RMC");
}

// =====================================================
// CONFIGURATION GLOBALE DU GPS
// =====================================================

void configureGPS()
{
  // Le GPS démarre normalement à 9600 bauds
  GPS.begin(
    GPS_DEFAULT_BAUD,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );

  delay(1200);

  // La commande est envoyée à 9600
  configureGPSBaud38400();

  delay(150);

  // L’ESP32 doit ensuite suivre le nouveau débit
  GPS.end();

  GPS.begin(
    GPS_TARGET_BAUD,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );

  delay(200);

  configureNmeaOutput();
  configureGPS4Hz();
}

// =====================================================
// INITIALISATION ESP-NOW
// =====================================================

bool configureEspNow()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("MAC ESP32 : ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erreur initialisation ESP-NOW");
    return false;
  }

  if (esp_now_register_send_cb(onDataSent) != ESP_OK) {
    Serial.println("Erreur callback ESP-NOW");
    return false;
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    broadcastAddress,
    sizeof(broadcastAddress)
  );

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peerInfo);

  if (result != ESP_OK &&
      result != ESP_ERR_ESPNOW_EXIST) {
    Serial.printf(
      "Erreur ajout broadcast : %d\n",
      result
    );

    return false;
  }

  return true;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Balise ESP-NOW + NEO-6M ===");

  packet.nodeId = 1;
  packet.team = 1;
  packet.etat = 1;

  configureGPS();

  if (!configureEspNow()) {
    Serial.println("Initialisation interrompue");
    return;
  }

  previousSend = millis();

  Serial.println("Initialisation terminee");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  // Lecture continue de l’UART GPS
  while (GPS.available() > 0) {
    char receivedCharacter = GPS.read();
    gps.encode(receivedCharacter);

    // Une seule mesure par phrase GGA
    if (ggaTime.isUpdated()) {
      const uint32_t nowUs = micros();

      if (windowStartMs == 0) {
        windowStartMs = millis();
      }

      if (lastGgaUs != 0) {
        Serial.printf(
          "GGA UTC=%s | periode=%.1f ms\n",
          ggaTime.value(),
          (nowUs - lastGgaUs) / 1000.0f
        );
      }

      lastGgaUs = nowUs;
      ggaCount++;
    }
  }

  // Actualisation des données du paquet
  if (gps.location.isUpdated() &&
      gps.location.isValid()) {
    packet.latitude =
        static_cast<int32_t>(
          gps.location.lat() * 10000000.0
        );

    packet.longitude =
        static_cast<int32_t>(
          gps.location.lng() * 10000000.0
        );
  }

  if (gps.altitude.isUpdated() &&
      gps.altitude.isValid()) {
    packet.altitude =
        static_cast<int16_t>(
          gps.altitude.meters()
        );
  }

  if (gps.course.isUpdated() &&
      gps.course.isValid()) {
    packet.heading =
        static_cast<int16_t>(
          gps.course.deg()
        );
  }

  const uint32_t nowMs = millis();

  // Mesure de cadence sur 10 secondes
  if (windowStartMs != 0 &&
      nowMs - windowStartMs >= 10000) {
    float frequency =
        ggaCount * 1000.0f /
        (nowMs - windowStartMs);

    Serial.printf(
      "Cadence GGA reelle : %.2f Hz | sats=%u\n",
      frequency,
      gps.satellites.isValid()
        ? gps.satellites.value()
        : 0
    );

    ggaCount = 0;
    windowStartMs = nowMs;
  }

  // Affichage différé du résultat ESP-NOW
  if (sendResultAvailable) {
    sendResultAvailable = false;

    Serial.println(
      sendSucceeded
        ? "Send Status: OK"
        : "Send Status: FAIL"
    );
  }

  // Émission non bloquante environ toutes les 450 ms
  if (nowMs - previousSend >= SEND_PERIOD_MS) {
    // Conserve une cadence stable même si loop() dérive
    previousSend += SEND_PERIOD_MS;

    packet.timestamp = nowMs;

    esp_err_t result = esp_now_send(
      broadcastAddress,
      reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet)
    );

    if (result != ESP_OK) {
      Serial.printf(
        "Erreur esp_now_send : %d\n",
        result
      );
    }
  }
}