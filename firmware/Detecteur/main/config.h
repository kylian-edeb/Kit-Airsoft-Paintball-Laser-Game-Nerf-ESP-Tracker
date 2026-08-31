#pragma once

#include <cstdint>

// ============================================================
// Détecteur — brochage ESP32-S3 N16R8 (PSRAM Octal : GPIO33-37
// réservés au bus PSRAM/flash interne, ne jamais les utiliser)
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

// ============================================================
// Paramètres applicatifs
// ============================================================

constexpr uint8_t  MAX_TRACKED_BEACONS   = 16;
constexpr uint32_t BEACON_TIMEOUT_MS     = 8000;   // balise considérée perdue
constexpr uint32_t BEACON_STALE_MS       = 3000;   // affichage "signal faible"
constexpr uint32_t DISPLAY_REFRESH_MS    = 150;

constexpr uint32_t ENCODER_LONGPRESS_MS  = 600;
constexpr uint32_t CALIBRATION_HOLD_MS   = 3000;
constexpr uint32_t CALIBRATION_DURATION_MS = 15000;

// ============================================================
// Mode debug / banc d'essai
// ============================================================
// À 1 : injecte 3 balises fictives en mouvement dans le tracker,
// pour tester l'écran, l'encodeur et le buzzer du Détecteur seul,
// sans aucune balise BEM réelle à proximité. Le cap et la position
// simulés ne remplacent le GPS/la boussole réels que s'ils sont
// absents ou sans fix : du vrai matériel connecté reprend toujours
// la main automatiquement. Un bandeau orange s'affiche en
// permanence à l'écran tant que ce mode est actif.
//
// NE JAMAIS laisser à 1 pour une utilisation en jeu réel : les
// balises affichées seraient fictives. Remettre à 0 avant de
// reflasher pour le terrain.
#define DETECTOR_DEBUG_SIMULATION 0
