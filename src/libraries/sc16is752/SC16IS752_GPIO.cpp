// SC16IS752_GPIO.cpp
#include "SC16IS752_GPIO.h"

#if defined(ESP32) || defined(ESP8266)
SC16IS752_GPIO::SC16IS752_GPIO(uint8_t i2cAddr, TwoWire& wire, const I2CPins& pins)
  : SC16IS752(i2cAddr, wire, pins),
    _lastError(GPIOError::NONE),
    _gpioInitialized(false) {
  initializeGPIOArrays();
}
#else
SC16IS752_GPIO::SC16IS752_GPIO(uint8_t i2cAddr, TwoWire& wire)
  : SC16IS752(i2cAddr, wire),
    _lastError(GPIOError::NONE),
    _gpioInitialized(false) {
  initializeGPIOArrays();
}
#endif

SC16IS752_GPIO::SC16IS752_GPIO(SPIClass& spi, uint8_t csPin, const SPIConfig& config)
  : SC16IS752(spi, csPin, config),
    _lastError(GPIOError::NONE),
    _gpioInitialized(false) {
  initializeGPIOArrays();
}

void SC16IS752_GPIO::initializeGPIOArrays() {
  for (uint8_t i = 0; i < MAX_GPIO_PINS; i++) {
    _gpioStats[i] = {};
    _pinStates[i] = {};
  }
}

bool SC16IS752_GPIO::beginGPIO() {
  if (!SC16IS752::begin()) {
    handleError(GPIOError::INITIALIZATION_ERROR);
    return false;
  }

  // Disable all interrupts initially
  writeRegister(CHANNEL_A, REG_IOCTRL, 0x00);

  // Set all pins as inputs initially
  writeRegister(CHANNEL_A, REG_IODIR, 0x00);

  // Clear any pending interrupts
  readRegister(CHANNEL_A, REG_IOSTATE);

  // Configure GPIO functionality
  uint8_t ioControl = readRegister(CHANNEL_A, REG_IOCTRL);
  ioControl &= ~0x0C;  // Clear GPIO/modem pins selection bits
  writeRegister(CHANNEL_A, REG_IOCTRL, ioControl);

  _gpioInitialized = true;
  return true;
}

void SC16IS752_GPIO::endGPIO() {
  if (_gpioInitialized) {
    writeRegister(CHANNEL_A, REG_IOCTRL, 0x00);  // Disable interrupts
    setPortDirection(0x00);                      // Set all pins as inputs
    _gpioInitialized = false;
  }
}

bool SC16IS752_GPIO::configurePin(uint8_t pin, const PinConfig& config) {
  if (!validatePin(pin) || !validatePinMode(pin, config.mode)) {
    return false;
  }

  _pinStates[pin].config = config;
  _pinStates[pin].lastState = config.initialState;
  _pinStates[pin].lastDebounceTime = 0;
  _pinStates[pin].interruptPending = false;

  if (!setPinMode(pin, config.mode)) {
    return false;
  }

  if (config.interruptMode != INTERRUPT_DISABLED) {
    enableInterrupt(pin, config.interruptMode);
  }

  if (config.mode == GPIO_OUTPUT) {
    digitalWrite(pin, config.initialState);
  }

  return true;
}

bool SC16IS752_GPIO::setPinMode(uint8_t pin, GPIOMode mode) {
  if (!validatePin(pin) || !validatePinMode(pin, mode)) {
    return false;
  }

  uint8_t dir = readRegister(CHANNEL_A, REG_IODIR);
  uint8_t ctrl = readRegister(CHANNEL_A, REG_IOCTRL);

  switch (mode) {
    case GPIO_OUTPUT:
      dir |= (1 << pin);
      ctrl &= ~(1 << pin);
      break;
    case GPIO_INPUT:
      dir &= ~(1 << pin);
      ctrl &= ~(1 << pin);
      break;
    case GPIO_INPUT_PULLUP:
      dir &= ~(1 << pin);
      ctrl |= (1 << pin);
      break;
  }

  writeRegister(CHANNEL_A, REG_IODIR, dir);
  writeRegister(CHANNEL_A, REG_IOCTRL, ctrl);

  return true;
}

bool SC16IS752_GPIO::digitalWrite(uint8_t pin, uint8_t value) {
  if (!validatePin(pin)) {
    handleError(GPIOError::INVALID_PIN);
    return false;
  }

  if (_pinStates[pin].config.mode != GPIO_OUTPUT) {
    handleError(GPIOError::INVALID_OPERATION);
    return false;
  }

  uint8_t state = readRegister(CHANNEL_A, REG_IOSTATE);
  if (value) {
    state |= (1 << pin);
  } else {
    state &= ~(1 << pin);
  }

  writeRegister(CHANNEL_A, REG_IOSTATE, state);

  _gpioStats[pin].currentState = value;
  _gpioStats[pin].lastChangeTime = millis();
  updateGPIOStats(pin, false, value);

  return true;
}

int SC16IS752_GPIO::digitalRead(uint8_t pin) {
  if (!validatePin(pin)) {
    handleError(GPIOError::INVALID_PIN);
    return -1;
  }

  uint8_t state = readRegister(CHANNEL_A, REG_IOSTATE);
  uint8_t currentState = (state & (1 << pin)) ? HIGH : LOW;

  if (_pinStates[pin].config.mode != GPIO_OUTPUT) {
    uint32_t currentTime = millis();

    if (currentState != _pinStates[pin].lastState) {
      _pinStates[pin].lastDebounceTime = currentTime;
    }

    if ((currentTime - _pinStates[pin].lastDebounceTime) > _pinStates[pin].config.debounceTime) {
      if (currentState != _gpioStats[pin].currentState) {
        _gpioStats[pin].currentState = currentState;
        updateGPIOStats(pin, false, currentState);
      }
    } else {
      _gpioStats[pin].bounceEvents++;
      return _gpioStats[pin].currentState;
    }
  }

  _pinStates[pin].lastState = currentState;
  return currentState;
}

bool SC16IS752_GPIO::togglePin(uint8_t pin) {
  if (!validatePin(pin)) {
    handleError(GPIOError::INVALID_PIN);
    return false;
  }

  if (_pinStates[pin].config.mode != GPIO_OUTPUT) {
    handleError(GPIOError::INVALID_OPERATION);
    return false;
  }

  uint8_t state = readRegister(CHANNEL_A, REG_IOSTATE);
  state ^= (1 << pin);
  writeRegister(CHANNEL_A, REG_IOSTATE, state);

  _gpioStats[pin].toggleCount++;
  _gpioStats[pin].currentState = !_gpioStats[pin].currentState;
  _gpioStats[pin].lastChangeTime = millis();

  updateGPIOStats(pin, false, _gpioStats[pin].currentState);
  return true;
}

bool SC16IS752_GPIO::writePort(uint8_t value) {
  if (!_gpioInitialized) {
    handleError(GPIOError::INITIALIZATION_ERROR);
    return false;
  }

  uint8_t direction = readRegister(CHANNEL_A, REG_IODIR);
  uint8_t currentState = readRegister(CHANNEL_A, REG_IOSTATE);
  uint8_t newState = (currentState & ~direction) | (value & direction);

  writeRegister(CHANNEL_A, REG_IOSTATE, newState);

  uint8_t changes = currentState ^ newState;
  for (uint8_t pin = 0; pin < MAX_GPIO_PINS; pin++) {
    if (changes & (1 << pin)) {
      _gpioStats[pin].currentState = (newState & (1 << pin)) ? HIGH : LOW;
      _gpioStats[pin].lastChangeTime = millis();
      updateGPIOStats(pin, false, _gpioStats[pin].currentState);
    }
  }

  return true;
}

uint8_t SC16IS752_GPIO::readPort() {
  return readRegister(CHANNEL_A, REG_IOSTATE);
}

bool SC16IS752_GPIO::setPortDirection(uint8_t direction) {
  writeRegister(CHANNEL_A, REG_IODIR, direction);
  return true;
}

uint8_t SC16IS752_GPIO::getPortDirection() {
  return readRegister(CHANNEL_A, REG_IODIR);
}

bool SC16IS752_GPIO::enableInterrupt(uint8_t pin, InterruptMode mode) {
  if (!validatePin(pin)) {
    handleError(GPIOError::INVALID_PIN);
    return false;
  }

  if (_pinStates[pin].config.mode == GPIO_OUTPUT) {
    handleError(GPIOError::INVALID_OPERATION);
    return false;
  }

  configureInterrupt(pin, mode);
  _pinStates[pin].config.interruptMode = mode;
  return true;
}

void SC16IS752_GPIO::configureInterrupt(uint8_t pin, InterruptMode mode) {
  uint8_t ctrl = readRegister(CHANNEL_A, REG_IOCTRL);
  uint8_t mask = 0x03 << (pin * 2);
  ctrl &= ~mask;

  switch (mode) {
    case INTERRUPT_RISING:
      ctrl |= (0x01 << (pin * 2));
      break;
    case INTERRUPT_FALLING:
      ctrl |= (0x02 << (pin * 2));
      break;
    case INTERRUPT_BOTH:
      ctrl |= (0x03 << (pin * 2));
      break;
  }

  writeRegister(CHANNEL_A, REG_IOCTRL, ctrl);
}

bool SC16IS752_GPIO::disableInterrupt(uint8_t pin) {
  if (!validatePin(pin)) {
    return false;
  }

  configureInterrupt(pin, INTERRUPT_DISABLED);
  _pinStates[pin].config.interruptMode = INTERRUPT_DISABLED;
  _pinStates[pin].interruptPending = false;
  return true;
}

uint8_t SC16IS752_GPIO::getInterruptStatus() {
  uint8_t status = 0;
  for (uint8_t pin = 0; pin < MAX_GPIO_PINS; pin++) {
    if (_pinStates[pin].interruptPending) {
      status |= (1 << pin);
    }
  }
  return status;
}

void SC16IS752_GPIO::clearInterrupt(uint8_t pin) {
  if (validatePin(pin)) {
    _pinStates[pin].interruptPending = false;
    readRegister(CHANNEL_A, REG_IOSTATE);
  }
}

bool SC16IS752_GPIO::setDebounceTime(uint8_t pin, uint32_t debounceTime) {
  if (!validatePin(pin)) {
    return false;
  }
  _pinStates[pin].config.debounceTime = debounceTime;
  return true;
}

SC16IS752_GPIO::GPIOStats SC16IS752_GPIO::getGPIOStats(uint8_t pin) {
  if (!validatePin(pin)) {
    return GPIOStats{};
  }
  return _gpioStats[pin];
}

bool SC16IS752_GPIO::isPinStable(uint8_t pin) {
  if (!validatePin(pin)) {
    return false;
  }

  return (millis() - _gpioStats[pin].lastChangeTime) > STABILITY_THRESHOLD;
}

uint32_t SC16IS752_GPIO::getLastChangeTime(uint8_t pin) {
  if (!validatePin(pin)) {
    return 0;
  }
  return _gpioStats[pin].lastChangeTime;
}

void SC16IS752_GPIO::updateGPIOStats(uint8_t pin, bool isInterrupt, bool risingEdge) {
  if (!validatePin(pin)) {
    return;
  }

  uint32_t currentTime = millis();

  if (isInterrupt) {
    _gpioStats[pin].interruptCount++;
    _gpioStats[pin].lastInterruptTime = currentTime;
  }

  _gpioStats[pin].lastChangeTime = currentTime;
  checkPinStability(pin);
}

void SC16IS752_GPIO::checkPinStability(uint8_t pin) {
  if (!validatePin(pin)) {
    return;
  }

  uint32_t timeSinceLastChange = millis() - _gpioStats[pin].lastChangeTime;
  _gpioStats[pin].isStable = (timeSinceLastChange > STABILITY_THRESHOLD);
}

bool SC16IS752_GPIO::validatePinMode(uint8_t pin, GPIOMode mode) {
  if (!validatePin(pin)) {
    return false;
  }

  switch (mode) {
    case GPIO_INPUT:
    case GPIO_OUTPUT:
    case GPIO_INPUT_PULLUP:
      return true;
    default:
      handleError(GPIOError::INVALID_MODE);
      return false;
  }
}

// SC16IS752_GPIO.cpp additions
bool SC16IS752_GPIO::getNextInterrupt(InterruptEvent& event) {
  if (_queueHead == _queueTail) {
    return false;
  }

  event = _interruptQueue[_queueHead];
  _queueHead = (_queueHead + 1) % MAX_QUEUE_SIZE;
  return true;
}

void SC16IS752_GPIO::queueInterrupt(uint8_t pin, bool rising) {
  uint8_t nextTail = (_queueTail + 1) % MAX_QUEUE_SIZE;

  if (nextTail != _queueHead) {
    _interruptQueue[_queueTail] = {
      .pin = pin,
      .rising = rising,
      .timestamp = millis()
    };
    _queueTail = nextTail;
  }
}

bool SC16IS752_GPIO::saveState() {
  uint8_t state = readRegister(CHANNEL_A, REG_IOSTATE);
  for (uint8_t i = 0; i < MAX_GPIO_PINS; i++) {
    _savedState[i] = (state & (1 << i)) ? 1 : 0;
  }
  return true;
}

bool SC16IS752_GPIO::restoreState() {
  uint8_t state = 0;
  for (uint8_t i = 0; i < MAX_GPIO_PINS; i++) {
    if (_savedState[i]) {
      state |= (1 << i);
    }
  }
  writeRegister(CHANNEL_A, REG_IOSTATE, state);
  return true;
}

void SC16IS752_GPIO::clearInterruptQueue() {
  _queueHead = _queueTail = 0;
}
