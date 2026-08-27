#include "encoder_input.h"
#include "config.h"

namespace {

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t encoderTransitions = 0;
volatile uint8_t previousEncoderState = 0;

const int8_t kQuadratureTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

constexpr int32_t TRANSITIONS_PER_DETENT = 4; // 4 pour la plupart des encodeurs mécaniques

} // namespace

void IRAM_ATTR EncoderInput::isrTrampoline()
{
  uint8_t currentState =
    (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

  uint8_t tableIndex = (previousEncoderState << 2) | currentState;
  int8_t movement = kQuadratureTable[tableIndex];

  if (movement != 0) {
    portENTER_CRITICAL_ISR(&encoderMux);
    encoderTransitions += movement;
    portEXIT_CRITICAL_ISR(&encoderMux);
  }

  previousEncoderState = currentState;
}

void EncoderInput::begin(int pinA, int pinB, int pinButton)
{
  _pinA = pinA;
  _pinB = pinB;
  _pinButton = pinButton;

  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  pinMode(_pinButton, INPUT_PULLUP);

  previousEncoderState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);

  attachInterrupt(digitalPinToInterrupt(_pinA), isrTrampoline, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinB), isrTrampoline, CHANGE);

  _previousButtonState = digitalRead(_pinButton);
}

void EncoderInput::update()
{
  int32_t transitionsCopy;
  portENTER_CRITICAL(&encoderMux);
  transitionsCopy = encoderTransitions;
  portEXIT_CRITICAL(&encoderMux);

  const int32_t difference = transitionsCopy - _lastProcessedTransitions;
  if (difference != 0) {
    _lastProcessedTransitions = transitionsCopy;
    _accumulator += difference;

    while (_accumulator >= TRANSITIONS_PER_DETENT) {
      _accumulator -= TRANSITIONS_PER_DETENT;
      _pendingSteps += 1;
    }
    while (_accumulator <= -TRANSITIONS_PER_DETENT) {
      _accumulator += TRANSITIONS_PER_DETENT;
      _pendingSteps -= 1;
    }
  }

  const bool currentButtonState = digitalRead(_pinButton);
  const uint32_t now = millis();

  if (currentButtonState != _previousButtonState && (now - _lastButtonChangeMillis) > 30) {
    _lastButtonChangeMillis = now;
    _previousButtonState = currentButtonState;

    if (currentButtonState == LOW) {
      _buttonHeld = true;
      _pressStartMillis = now;
    } else {
      _buttonHeld = false;

      // Classification au relâchement, selon la durée totale de l'appui :
      // un appui qui finit par durer 3 s ne doit jamais avoir déjà
      // déclenché l'action "appui court" ou "appui long" en cours de route.
      const uint32_t heldDuration = now - _pressStartMillis;

      if (heldDuration >= CALIBRATION_HOLD_MS) {
        _veryLongPressPending = true;
      } else if (heldDuration >= ENCODER_LONGPRESS_MS) {
        _longPressPending = true;
      } else {
        _shortPressPending = true;
      }
    }
  }
}

int EncoderInput::consumeRotationSteps()
{
  const int steps = _pendingSteps;
  _pendingSteps = 0;
  return steps;
}

bool EncoderInput::consumeShortPress()
{
  if (_shortPressPending) {
    _shortPressPending = false;
    return true;
  }
  return false;
}

bool EncoderInput::consumeLongPress()
{
  if (_longPressPending) {
    _longPressPending = false;
    return true;
  }
  return false;
}

bool EncoderInput::consumeVeryLongPress()
{
  if (_veryLongPressPending) {
    _veryLongPressPending = false;
    return true;
  }
  return false;
}

uint32_t EncoderInput::buttonHeldDurationMs() const
{
  if (!_buttonHeld) {
    return 0;
  }
  return millis() - _pressStartMillis;
}
