#include "buzzer.h"

namespace {

struct PatternSpec {
  uint8_t beepCount;
  uint16_t frequencyHz;
  uint16_t onMs;
  uint16_t offMs;
};

PatternSpec specFor(BuzzerPattern pattern)
{
  switch (pattern) {
    case BuzzerPattern::SingleBeep:
      return { 1, 2400, 80, 80 };
    case BuzzerPattern::DoubleBeep:
      return { 2, 2000, 100, 100 };
    case BuzzerPattern::TripleBeep:
      return { 3, 1200, 150, 120 };
    default:
      return { 0, 0, 0, 0 };
  }
}

} // namespace

Buzzer::Buzzer(int pin) : _pin(pin) {}

void Buzzer::begin()
{
  pinMode(_pin, OUTPUT);
  noTone(_pin);
}

void Buzzer::play(BuzzerPattern pattern)
{
  if (pattern == BuzzerPattern::None) {
    return;
  }

  // Une alerte plus prioritaire (déjà en cours) n'est pas interrompue par
  // une simple détection : on ne remplace que si le buzzer est libre.
  if (_activePattern != BuzzerPattern::None) {
    return;
  }

  _activePattern = pattern;
  _stepIndex = 0;

  const PatternSpec spec = specFor(pattern);
  startStep(spec.frequencyHz, spec.onMs, spec.offMs);
}

void Buzzer::startStep(uint16_t frequencyHz, uint16_t durationMs, uint16_t /*silenceMs*/)
{
  tone(_pin, frequencyHz, durationMs);
  _nextStepMillis = millis() + durationMs;
}

void Buzzer::update()
{
  if (_activePattern == BuzzerPattern::None) {
    return;
  }

  if (millis() < _nextStepMillis) {
    return;
  }

  const PatternSpec spec = specFor(_activePattern);
  const bool completedPhaseWasBeep = (_stepIndex % 2) == 0;

  if (completedPhaseWasBeep) {
    const uint8_t beepsCompleted = (_stepIndex / 2) + 1;

    if (beepsCompleted >= spec.beepCount) {
      _activePattern = BuzzerPattern::None;
      return;
    }

    // Bip terminé, pas le dernier : on attend en silence avant le suivant.
    _stepIndex++;
    _nextStepMillis = millis() + spec.offMs;
  } else {
    // Silence terminé : déclenche le bip suivant.
    _stepIndex++;
    tone(_pin, spec.frequencyHz, spec.onMs);
    _nextStepMillis = millis() + spec.onMs;
  }
}
