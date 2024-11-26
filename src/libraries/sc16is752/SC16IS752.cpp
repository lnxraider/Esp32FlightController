// SC16IS752.cpp

#include "SC16IS752.h"

#if defined(ESP32) || defined(ESP8266)
SC16IS752::SC16IS752(uint8_t i2cAddr, TwoWire& wire, const I2CPins& pins)
  : _wire(&wire),
    _spi(nullptr),
    _i2cAddr(i2cAddr),
    _pins(pins),
    _interface(I2C_MODE),
    _initialized(false),
    _lastError(Error::NONE) {}
#else
SC16IS752::SC16IS752(uint8_t i2cAddr, TwoWire& wire)
  : _wire(&wire),
    _spi(nullptr),
    _i2cAddr(i2cAddr),
    _interface(I2C_MODE),
    _initialized(false),
    _lastError(Error::NONE) {}
#endif

SC16IS752::SC16IS752(SPIClass& spi, uint8_t csPin, const SPIConfig& config)
  : _wire(nullptr),
    _spi(&spi),
    _csPin(csPin),
    _spiConfig(config),
    _interface(SPI_MODE),
    _initialized(false),
    _lastError(Error::NONE) {}

bool SC16IS752::begin(const CrystalConfig& config) {
  _crystalConfig = config;
  _lastError = Error::NONE;

  if (_interface == I2C_MODE) {
#if defined(ESP32) || defined(ESP8266)
    if (!initI2C()) {
      _lastError = Error::INITIALIZATION_ERROR;
      return false;
    }
#else
    _wire->begin();
#endif
  } else {
    if (!beginSPI()) {
      _lastError = Error::INITIALIZATION_ERROR;
      return false;
    }
  }

  // Software reset
  writeRegister(CHANNEL_A, REG_IOCTRL, 0x08);
  delay(100);

  if (!ping()) {
    _lastError = Error::COMMUNICATION_ERROR;
    return false;
  }

  // Disable FIFOs initially
  writeRegister(CHANNEL_A, REG_FCR, 0x00);
  writeRegister(CHANNEL_B, REG_FCR, 0x00);
  delay(10);

  _initialized = true;
  return true;
}

void SC16IS752::end() {
  if (_initialized) {
    if (_interface == I2C_MODE) {
#if defined(ESP32) || defined(ESP8266)
      _wire->end();
#endif
    } else {
      _spi->end();
    }
    _initialized = false;
  }
}

bool SC16IS752::beginSPI() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  _spi->begin();
  initSPI();
  return true;
}

void SC16IS752::initSPI() {
  _spi->beginTransaction(SPISettings(
    _spiConfig.clockSpeed,
    _spiConfig.bitOrder,
    _spiConfig.dataMode));
  _spi->endTransaction();
}

bool SC16IS752::setSPIConfig(const SPIConfig& config) {
  _spiConfig = config;
  if (_initialized && _interface == SPI_MODE) {
    initSPI();
  }
  return true;
}

void SC16IS752::writeRegister(uint8_t channel, uint8_t reg, uint8_t value) {
  if (!isValidChannel(channel)) {
    _lastError = Error::INVALID_CHANNEL;
    return;
  }

  if (_interface == I2C_MODE) {
    writeRegisterI2C(channel, reg, value);
  } else {
    writeRegisterSPI(channel, reg, value);
  }
}

uint8_t SC16IS752::readRegister(uint8_t channel, uint8_t reg) {
  if (!isValidChannel(channel)) {
    _lastError = Error::INVALID_CHANNEL;
    return 0;
  }

  return (_interface == I2C_MODE) ? readRegisterI2C(channel, reg) : readRegisterSPI(channel, reg);
}

void SC16IS752::writeRegisterI2C(uint8_t channel, uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(regAddr(reg, channel));
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    _lastError = Error::COMMUNICATION_ERROR;
  }
}

uint8_t SC16IS752::readRegisterI2C(uint8_t channel, uint8_t reg) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(regAddr(reg, channel));
  if (_wire->endTransmission(false) != 0) {
    _lastError = Error::COMMUNICATION_ERROR;
    return 0;
  }

  if (_wire->requestFrom(_i2cAddr, (uint8_t)1) != 1) {
    _lastError = Error::COMMUNICATION_ERROR;
    return 0;
  }

  return _wire->read();
}

void SC16IS752::writeRegisterSPI(uint8_t channel, uint8_t reg, uint8_t value) {
  beginSPITransaction();
  _spi->transfer(regAddr(reg, channel));
  _spi->transfer(value);
  endSPITransaction();
}

uint8_t SC16IS752::readRegisterSPI(uint8_t channel, uint8_t reg) {
  beginSPITransaction();
  _spi->transfer(regAddr(reg, channel) | 0x80);
  uint8_t value = _spi->transfer(0);
  endSPITransaction();
  return value;
}

void SC16IS752::beginSPITransaction() {
  _spi->beginTransaction(SPISettings(
    _spiConfig.clockSpeed,
    _spiConfig.bitOrder,
    _spiConfig.dataMode));
  digitalWrite(_csPin, LOW);
}

void SC16IS752::endSPITransaction() {
  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();
}

uint8_t SC16IS752::regAddr(uint8_t reg, uint8_t channel) const {
  return (reg << 3) | (channel ? 0x02 : 0x00);
}

bool SC16IS752::ping() {
  writeRegister(CHANNEL_A, REG_SPR, 0x55);
  return (readRegister(CHANNEL_A, REG_SPR) == 0x55);
}

#if defined(ESP32) || defined(ESP8266)
bool SC16IS752::initI2C() {
  if (_pins.sda >= 0 && _pins.scl >= 0) {
    return _wire->begin(_pins.sda, _pins.scl);
  }
  return _wire->begin();
}
#endif

// SC16IS752.cpp additions
bool SC16IS752::verifyClockConfig() {
  // Write test pattern to SPR
  writeRegister(CHANNEL_A, REG_SPR, 0x55);
  delayMicroseconds(_optimizedDelay);

  // Verify with timing measurement
  uint32_t start = micros();
  uint8_t value = readRegister(CHANNEL_A, REG_SPR);
  uint32_t elapsed = micros() - start;

  if (value != 0x55 || elapsed > 1000) {
    return false;
  }

  return performClockTest();
}

bool SC16IS752::performClockTest() {
  // Test clock stability using LSR readings
  uint8_t lsr1 = readRegister(CHANNEL_A, REG_LSR);
  delayMicroseconds(100);
  uint8_t lsr2 = readRegister(CHANNEL_A, REG_LSR);

  return (lsr1 & 0x60) == 0x60 && (lsr2 & 0x60) == 0x60;
}

bool SC16IS752::verifyDeviceID() {
  // Write unique pattern
  writeRegister(CHANNEL_A, REG_SPR, 0xA5);
  writeRegister(CHANNEL_B, REG_SPR, 0x5A);

  return (readRegister(CHANNEL_A, REG_SPR) == 0xA5) && (readRegister(CHANNEL_B, REG_SPR) == 0x5A);
}

bool SC16IS752::setPowerMode(PowerMode mode) {
  switch (mode) {
    case POWER_NORMAL:
      writeRegister(CHANNEL_A, REG_IER, readRegister(CHANNEL_A, REG_IER) & ~0x10);
      break;

    case POWER_SLEEP:
      writeRegister(CHANNEL_A, REG_IER, readRegister(CHANNEL_A, REG_IER) | 0x10);
      break;

    case POWER_SHUTDOWN:
      // Set both channels to sleep
      writeRegister(CHANNEL_A, REG_IER, readRegister(CHANNEL_A, REG_IER) | 0x10);
      writeRegister(CHANNEL_B, REG_IER, readRegister(CHANNEL_B, REG_IER) | 0x10);
      break;
  }

  _powerMode = mode;
  return true;
}

bool SC16IS752::optimizeBusTiming() {
  uint32_t minDelay = 10;   // Start with 10us
  uint32_t maxDelay = 200;  // Max 200us
  uint32_t optimalDelay = maxDelay;

  // Binary search for optimal timing
  while (minDelay <= maxDelay) {
    uint32_t testDelay = (minDelay + maxDelay) / 2;
    _optimizedDelay = testDelay;

    if (verifyClockConfig()) {
      optimalDelay = testDelay;
      maxDelay = testDelay - 1;
    } else {
      minDelay = testDelay + 1;
    }
  }

  _optimizedDelay = optimalDelay;
  return true;
}
