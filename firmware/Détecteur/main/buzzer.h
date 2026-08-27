#pragma once

#include <Arduino.h>

// ============================================================
// Alerte sonore non bloquante sur buzzer passif, pilotée via
// tone()/noTone() (disponible nativement depuis arduino-esp32
// 3.x, basé sur le pilote LEDC de l'ESP-IDF).
// ============================================================

enum class BuzzerPattern {
  None,
  SingleBeep,   // nouvelle balise détectée
  DoubleBeep,   // cible sélectionnée perdue
  TripleBeep,   // batterie faible / erreur capteur
};

class Buzzer {
public:
  explicit Buzzer(int pin);

  void begin();
  void play(BuzzerPattern pattern);

  // À appeler à chaque tour de boucle pour dérouler les séquences
  // de bips sans bloquer le reste du firmware.
  void update();

private:
  int _pin;

  BuzzerPattern _activePattern = BuzzerPattern::None;
  uint8_t _stepIndex = 0;
  uint32_t _nextStepMillis = 0;

  void startStep(uint16_t frequencyHz, uint16_t durationMs, uint16_t silenceMs);
};
