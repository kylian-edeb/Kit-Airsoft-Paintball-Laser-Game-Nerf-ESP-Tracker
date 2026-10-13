#pragma once

#include <Arduino.h>

// ============================================================
// Encodeur rotatif + bouton poussoir : navigation dans l'IHM du
// Détecteur (changement d'écran, sélection de cible, calibration).
// Reprend la logique de quadrature du code proposé initialement.
// ============================================================

class EncoderInput {
public:
  void begin(int pinA, int pinB, int pinButton);

  // À appeler à chaque tour de boucle.
  void update();

  // Nombre de crans franchis (signé) depuis le dernier appel ;
  // remet le compteur à zéro.
  int consumeRotationSteps();

  // La classification (court / long / très long) se fait au relâchement,
  // selon la durée totale de l'appui : cela évite qu'un appui long ne
  // déclenche d'abord l'action "court" avant d'être requalifié.
  // Chaque consume* ne renvoie true qu'une seule fois par appui.
  bool consumeShortPress();
  bool consumeLongPress();     // ENCODER_LONGPRESS_MS <= durée < CALIBRATION_HOLD_MS
  bool consumeVeryLongPress(); // durée >= CALIBRATION_HOLD_MS

  bool isButtonHeld() const { return _buttonHeld; }

private:
  int _pinA = -1;
  int _pinB = -1;
  int _pinButton = -1;

  int32_t _lastProcessedTransitions = 0;
  int32_t _accumulator = 0;
  int _pendingSteps = 0;

  bool _buttonHeld = false;
  bool _shortPressPending = false;
  bool _longPressPending = false;
  bool _veryLongPressPending = false;
  uint32_t _pressStartMillis = 0;
  uint32_t _lastButtonChangeMillis = 0;
  bool _previousButtonState = HIGH;

  void handleInterrupt();
  static void IRAM_ATTR isrTrampoline();
};
