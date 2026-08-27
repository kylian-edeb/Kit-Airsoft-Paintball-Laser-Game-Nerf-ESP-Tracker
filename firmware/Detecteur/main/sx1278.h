#pragma once

#include <Arduino.h>
#include <SPI.h>

// ============================================================
// Pilote minimal pour le module radio LoRa-02 (SX1278), en
// registre direct (aucune dépendance à la librairie Arduino
// "LoRa" de Sandeep Mistry, qui n'existe pas dans le registre
// de composants ESP-IDF). Fonctionne en réception continue :
// c'est le mode dont le Détecteur a besoin pour écouter les
// balises BEM/BPS.
// ============================================================

struct LoRaReceivedPacket {
  uint8_t data[256];
  uint8_t length;
  int16_t rssi;   // dBm
  float   snr;    // dB
};

class LoRaRadio {
public:
  LoRaRadio(SPIClass &spi, int csPin, int resetPin, int dio0Pin);

  // Initialise le module et bascule en réception continue.
  // Retourne false si la puce ne répond pas (version registre invalide).
  bool begin(float frequencyHz, uint8_t spreadingFactor,
             uint8_t bandwidthIndex, uint8_t codingRate, uint8_t syncWord);

  // À appeler à chaque tour de boucle : renvoie true si un paquet
  // valide (CRC matériel correct) vient d'être reçu dans `out`.
  bool poll(LoRaReceivedPacket &out);

private:
  SPIClass &_spi;
  int _csPin;
  int _resetPin;
  int _dio0Pin;
  bool _lastDio0State = false;

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  void readFifo(uint8_t *buffer, uint8_t length);

  void setFrequency(float frequencyHz);
  void setLoRaMode();
  void setRxContinuousMode();
};
