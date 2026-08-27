#include "sx1278.h"

namespace {

constexpr uint8_t REG_FIFO                 = 0x00;
constexpr uint8_t REG_OP_MODE              = 0x01;
constexpr uint8_t REG_FRF_MSB              = 0x06;
constexpr uint8_t REG_FRF_MID              = 0x07;
constexpr uint8_t REG_FRF_LSB              = 0x08;
constexpr uint8_t REG_PA_CONFIG            = 0x09;
constexpr uint8_t REG_LNA                  = 0x0C;
constexpr uint8_t REG_FIFO_ADDR_PTR        = 0x0D;
constexpr uint8_t REG_FIFO_TX_BASE_ADDR    = 0x0E;
constexpr uint8_t REG_FIFO_RX_BASE_ADDR    = 0x0F;
constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
constexpr uint8_t REG_IRQ_FLAGS            = 0x12;
constexpr uint8_t REG_RX_NB_BYTES          = 0x13;
constexpr uint8_t REG_PKT_SNR_VALUE        = 0x19;
constexpr uint8_t REG_PKT_RSSI_VALUE       = 0x1A;
constexpr uint8_t REG_MODEM_CONFIG_1       = 0x1D;
constexpr uint8_t REG_MODEM_CONFIG_2       = 0x1E;
constexpr uint8_t REG_PREAMBLE_MSB         = 0x20;
constexpr uint8_t REG_PREAMBLE_LSB         = 0x21;
constexpr uint8_t REG_PAYLOAD_LENGTH       = 0x22;
constexpr uint8_t REG_MODEM_CONFIG_3       = 0x26;
constexpr uint8_t REG_SYNC_WORD            = 0x39;
constexpr uint8_t REG_DIO_MAPPING_1        = 0x40;
constexpr uint8_t REG_VERSION              = 0x42;

constexpr uint8_t MODE_LONG_RANGE_MODE = 0x80;
constexpr uint8_t MODE_SLEEP           = 0x00;
constexpr uint8_t MODE_STDBY           = 0x01;
constexpr uint8_t MODE_RX_CONTINUOUS   = 0x05;

constexpr uint8_t IRQ_RX_DONE_MASK          = 0x40;
constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20;

constexpr uint32_t SX1278_XOSC_HZ = 32000000UL;
constexpr uint32_t FREQ_STEP_DENOMINATOR = 1UL << 19;

const SPISettings kSpiSettings(8000000, MSBFIRST, SPI_MODE0);

} // namespace

LoRaRadio::LoRaRadio(SPIClass &spi, int csPin, int resetPin, int dio0Pin)
  : _spi(spi), _csPin(csPin), _resetPin(resetPin), _dio0Pin(dio0Pin)
{
}

uint8_t LoRaRadio::readRegister(uint8_t address)
{
  _spi.beginTransaction(kSpiSettings);
  digitalWrite(_csPin, LOW);

  _spi.transfer(address & 0x7F);
  uint8_t value = _spi.transfer(0x00);

  digitalWrite(_csPin, HIGH);
  _spi.endTransaction();

  return value;
}

void LoRaRadio::writeRegister(uint8_t address, uint8_t value)
{
  _spi.beginTransaction(kSpiSettings);
  digitalWrite(_csPin, LOW);

  _spi.transfer(address | 0x80);
  _spi.transfer(value);

  digitalWrite(_csPin, HIGH);
  _spi.endTransaction();
}

void LoRaRadio::readFifo(uint8_t *buffer, uint8_t length)
{
  _spi.beginTransaction(kSpiSettings);
  digitalWrite(_csPin, LOW);

  _spi.transfer(REG_FIFO & 0x7F);
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = _spi.transfer(0x00);
  }

  digitalWrite(_csPin, HIGH);
  _spi.endTransaction();
}

void LoRaRadio::setFrequency(float frequencyHz)
{
  const uint64_t frf =
    (static_cast<uint64_t>(frequencyHz) * FREQ_STEP_DENOMINATOR) /
    SX1278_XOSC_HZ;

  writeRegister(REG_FRF_MSB, static_cast<uint8_t>(frf >> 16));
  writeRegister(REG_FRF_MID, static_cast<uint8_t>(frf >> 8));
  writeRegister(REG_FRF_LSB, static_cast<uint8_t>(frf >> 0));
}

void LoRaRadio::setLoRaMode()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
}

void LoRaRadio::setRxContinuousMode()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

bool LoRaRadio::begin(float frequencyHz, uint8_t spreadingFactor,
                       uint8_t bandwidthIndex, uint8_t codingRate,
                       uint8_t syncWord)
{
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  pinMode(_resetPin, OUTPUT);
  pinMode(_dio0Pin, INPUT);

  // Reset matériel du module (voir datasheet SX1276/78, section reset).
  digitalWrite(_resetPin, LOW);
  delay(10);
  digitalWrite(_resetPin, HIGH);
  delay(10);

  const uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    // 0x12 = SX1276/77/78/79. Autre valeur => câblage ou module invalide.
    return false;
  }

  setLoRaMode();
  setFrequency(frequencyHz);

  // Base FIFO pour RX et TX à l'adresse 0 (paquet unique à la fois).
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

  // LNA gain maximum + boost, AGC gère l'atténuation automatiquement.
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  const uint8_t bw = bandwidthIndex & 0x0F;
  const uint8_t cr = (codingRate - 4) & 0x07; // 4/5..4/8 -> 1..4

  writeRegister(REG_MODEM_CONFIG_1, (bw << 4) | (cr << 1));

  const uint8_t sf = spreadingFactor & 0x0F;
  constexpr uint8_t CRC_ON = 0x04;
  writeRegister(REG_MODEM_CONFIG_2, (sf << 4) | CRC_ON);

  // AgcAutoOn=1 ; LowDataRateOptimize activé si le symbole dépasse 16ms
  // (recommandation Semtech pour SF11/SF12 en 125 kHz).
  const bool lowDataRateOptimize = (spreadingFactor >= 11) && (bandwidthIndex <= 7);
  writeRegister(REG_MODEM_CONFIG_3, 0x04 | (lowDataRateOptimize ? 0x08 : 0x00));

  writeRegister(REG_PREAMBLE_MSB, 0x00);
  writeRegister(REG_PREAMBLE_LSB, 0x08);

  writeRegister(REG_SYNC_WORD, syncWord);

  // DIO0 -> RxDone (bits 7:6 = 00 en mode réception).
  writeRegister(REG_DIO_MAPPING_1, 0x00);

  writeRegister(REG_IRQ_FLAGS, 0xFF); // purge des drapeaux résiduels

  setRxContinuousMode();

  return true;
}

bool LoRaRadio::poll(LoRaReceivedPacket &out)
{
  const bool dio0State = digitalRead(_dio0Pin) == HIGH;
  const bool risingEdge = dio0State && !_lastDio0State;
  _lastDio0State = dio0State;

  if (!risingEdge) {
    return false;
  }

  const uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
  writeRegister(REG_IRQ_FLAGS, 0xFF); // acquitte tous les drapeaux

  if (!(irqFlags & IRQ_RX_DONE_MASK)) {
    return false;
  }

  if (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
    // Paquet corrompu : le module reste en RX continu, on ignore.
    return false;
  }

  const uint8_t length = readRegister(REG_RX_NB_BYTES);
  const uint8_t currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);

  writeRegister(REG_FIFO_ADDR_PTR, currentAddr);
  readFifo(out.data, length);
  out.length = length;

  const uint8_t rawRssi = readRegister(REG_PKT_RSSI_VALUE);
  // Offset -164 dBm valable en bande basse (<525 MHz), cf. datasheet §5.5.5.
  out.rssi = static_cast<int16_t>(rawRssi) - 164;

  const int8_t rawSnr = static_cast<int8_t>(readRegister(REG_PKT_SNR_VALUE));
  out.snr = rawSnr / 4.0f;

  return true;
}
