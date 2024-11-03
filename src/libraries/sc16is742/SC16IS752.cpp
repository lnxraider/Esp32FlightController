// SC16IS752.cpp
#include "SC16IS752.h"

//=====================================================================
// Constructor & Initialization
//=====================================================================

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
SC16IS752::SC16IS752(TwoWire& wire, const I2CPins& pins) :
    _wire(wire),
    _i2cAddr(0),
    _initialized(false),
    _currentBaudRate(0),
    _gpioDirection(0),
    _gpioState(0),
    _pwmStates(),
    _stats(),
    _timings(),
    _pins(pins),
    _i2cFreq(DEFAULT_I2C_FREQ) {}
#else
SC16IS752::SC16IS752(TwoWire& wire) :
    _wire(wire),
    _i2cAddr(0),
    _initialized(false),
    _currentBaudRate(0),
    _gpioDirection(0),
    _gpioState(0),
    _pwmStates(),
    _stats(),
    _timings() {}
#endif

bool SC16IS752::begin(uint8_t i2cAddr, uint32_t i2cFreq) {
    _i2cAddr = i2cAddr;
    _stats.reset();

    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    _i2cFreq = i2cFreq;
    if (!initI2C()) {
        if (DEBUG_ENABLED) {
            Serial.println(F("I2C initialization failed"));
        }
        return false;
    }
    #else
    _wire.begin();
    _wire.setClock(i2cFreq);
    #endif

    // Test device presence with scratch register
    uint8_t testValue = 0x55;
    if (!verifyRegisterWrite(regAddr(REG_SPR, CHANNEL_A), testValue)) {
        if (DEBUG_ENABLED) {
            Serial.println(F("Device verification failed"));
        }
        return false;
    }

    _initialized = true;
    return true;
}

void SC16IS752::end() {
    if (_initialized) {
        // Disable all PWM channels
        for (uint8_t i = 0; i < 8; i++) {
            if (_pwmStates[i].enabled) {
                pwmEnd(i);
            }
        }

        // Reset GPIO states
        _gpioDirection = 0;
        _gpioState = 0;

        #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
        _wire.end();
        #endif
        
        _initialized = false;
        _stats.reset();
    }
}

bool SC16IS752::detectDevice() {
    _wire.beginTransmission(_i2cAddr);
    return (_wire.endTransmission() == 0);
}

//=====================================================================
// Register Access Methods
//=====================================================================

int SC16IS752::writeReg(uint8_t reg, uint8_t value) {
    if (!_initialized && reg != regAddr(REG_SPR, CHANNEL_A)) {
        return ERR_STATE;
    }

    for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
        _wire.beginTransmission(_i2cAddr);
        if (_wire.write(reg) != 1 || _wire.write(value) != 1) {
            delayMicroseconds(50);
            continue;
        }

        if (_wire.endTransmission() == 0) {
            return OK;
        }
        delayMicroseconds(100);
    }

    return ERR_I2C;
}

int SC16IS752::readReg(uint8_t reg) {
    if (!_initialized && reg != regAddr(REG_SPR, CHANNEL_A)) {
        return ERR_STATE;
    }

    for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
        _wire.beginTransmission(_i2cAddr);
        if (_wire.write(reg) != 1 || _wire.endTransmission(false) != 0) {
            delayMicroseconds(50);
            continue;
        }

        if (_wire.requestFrom(_i2cAddr, (uint8_t)1) != 1) {
            delayMicroseconds(50);
            continue;
        }

        return _wire.read();
    }

    return ERR_I2C;
}

bool SC16IS752::verifyRegisterWrite(uint8_t reg, uint8_t value) {
    if (writeReg(reg, value) != OK) {
        return false;
    }

    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    delayMicroseconds(200);
    #else
    delayMicroseconds(100);
    #endif

    int readValue = readReg(reg);
    return (readValue >= 0) && ((uint8_t)readValue == value);
}

uint8_t SC16IS752::regAddr(uint8_t reg, uint8_t channel) const {
    return (reg << 3) | (channel ? 0x02 : 0x00);
}

//=====================================================================
// Platform-Specific Methods
//=====================================================================

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
bool SC16IS752::initI2C() {
    if (_pins.sda >= 0 && _pins.scl >= 0) {
        if (!_wire.begin(_pins.sda, _pins.scl)) {
            return false;
        }
    } else {
        if (!_wire.begin()) {
            return false;
        }
    }
    _wire.setClock(_i2cFreq);
    return true;
}

bool SC16IS752::setPins(int sda, int scl) {
    if (_initialized) {
        _wire.end();
    }
    _pins.sda = sda;
    _pins.scl = scl;
    return initI2C();
}

bool SC16IS752::setI2CFrequency(uint32_t frequency) {
    _i2cFreq = frequency;
    _wire.setClock(frequency);
    return true;
}
#endif

//=====================================================================
// UART Configuration
//=====================================================================

int SC16IS752::initializeUART(uint8_t channel, uint32_t baudRate, bool testMode, uint32_t xtalFreq) {
    if (!isValidChannel(channel)) return ERR_PARAM;

    if (DEBUG_ENABLED) {
        Serial.print(F("\nInitializing UART channel "));
        Serial.println(channel);
    }

    _currentBaudRate = baudRate;

    // Reset FIFOs first
    int result = writeReg(regAddr(REG_FCR, channel), 0x00);
    if (result != OK) {
        if (DEBUG_ENABLED) Serial.println(F("Failed to disable FIFOs"));
        return result;
    }
    delay(10);

    // Set baud rate
    result = setBaudRate(channel, baudRate, xtalFreq);
    if (result != OK) {
        if (DEBUG_ENABLED) Serial.println(F("Failed to set baud rate"));
        return result;
    }
    delay(10);

    // Set line control for 8N1
    result = writeReg(regAddr(REG_LCR, channel), LCR_WORD_LEN_8);
    if (result != OK) {
        if (DEBUG_ENABLED) Serial.println(F("Failed to set line control"));
        return result;
    }
    delay(10);

    if (testMode) {
        if (DEBUG_ENABLED) Serial.println(F("Configuring for loopback test"));

        // Reset and configure FIFOs
        result = writeReg(regAddr(REG_FCR, channel),
                         FCR_FIFO_ENABLE | FCR_RX_FIFO_RESET | FCR_TX_FIFO_RESET);
        if (result != OK) return result;
        delay(10);

        // Enable loopback mode
        result = writeReg(regAddr(REG_MCR, channel), MCR_LOOPBACK);
        if (result != OK) return result;
        delay(10);

        // Set FIFO triggers
        result = writeReg(regAddr(REG_FCR, channel),
                         FCR_FIFO_ENABLE | FCR_TX_TRIGGER_LVL0 | FCR_RX_TRIGGER_LVL0);
        if (result != OK) return result;
        delay(10);

        // Verify loopback mode
        int mcr = readReg(regAddr(REG_MCR, channel));
        if ((mcr & MCR_LOOPBACK) != MCR_LOOPBACK) {
            if (DEBUG_ENABLED) Serial.println(F("Loopback mode verification failed"));
            return ERR_VERIFY;
        }
    } else {
        // Normal operation mode
        // Configure FIFOs with default triggers
        result = writeReg(regAddr(REG_FCR, channel),
                         FCR_FIFO_ENABLE | FCR_TX_TRIGGER_LVL2 | FCR_RX_TRIGGER_LVL2);
        if (result != OK) return result;
    }

    // Disable all interrupts
    result = writeReg(regAddr(REG_IER, channel), 0x00);
    return result;
}

int SC16IS752::setBaudRate(uint8_t channel, uint32_t baud, uint32_t xtalFreq) {
    if (!isValidChannel(channel) || baud == 0) return ERR_PARAM;

    uint16_t divisor = xtalFreq / (16 * baud);
    if (divisor == 0) return ERR_PARAM;

    // Save current LCR value
    int lcr = readReg(regAddr(REG_LCR, channel));
    if (lcr < 0) return lcr;

    // Enable divisor latch
    int result = writeReg(regAddr(REG_LCR, channel), lcr | LCR_DIVISOR_ENABLE);
    if (result != OK) return result;

    // Set divisor
    result = writeReg(regAddr(REG_DLL, channel), divisor & 0xFF);
    if (result != OK) return result;

    result = writeReg(regAddr(REG_DLH, channel), (divisor >> 8) & 0xFF);
    if (result != OK) return result;

    _currentBaudRate = baud;

    // Restore original LCR value
    return writeReg(regAddr(REG_LCR, channel), lcr);
}

int SC16IS752::setLineControl(uint8_t channel, uint8_t databits, uint8_t stopbits, uint8_t parity) {
    if (!isValidChannel(channel)) return ERR_PARAM;
    return writeReg(regAddr(REG_LCR, channel), databits | stopbits | parity);
}

int SC16IS752::configureFIFO(uint8_t channel, uint8_t txTrigger, uint8_t rxTrigger) {
    if (!isValidChannel(channel)) return ERR_PARAM;

    // First, disable FIFO
    int result = writeReg(regAddr(REG_FCR, channel), 0);
    if (result != OK) return result;
    delay(10);

    // Reset both FIFOs
    result = writeReg(regAddr(REG_FCR, channel),
                     FCR_FIFO_ENABLE | FCR_RX_FIFO_RESET | FCR_TX_FIFO_RESET);
    if (result != OK) return result;
    delay(10);

    // Configure trigger levels
    result = writeReg(regAddr(REG_FCR, channel),
                     FCR_FIFO_ENABLE | txTrigger | rxTrigger);
    delay(10);

    return result;
}

//=====================================================================
// UART Status Methods
//=====================================================================

SC16IS752::UARTStatus SC16IS752::getStatus(uint8_t channel) {
    UARTStatus status;
    status.error = OK;

    if (!isValidChannel(channel)) {
        status.error = ERR_PARAM;
        return status;
    }

    int lsr = readReg(regAddr(REG_LSR, channel));
    int msr = readReg(regAddr(REG_MSR, channel));
    int txlvl = readReg(regAddr(REG_TXLVL, channel));
    int rxlvl = readReg(regAddr(REG_RXLVL, channel));

    if (lsr < 0 || msr < 0 || txlvl < 0 || rxlvl < 0) {
        status.error = ERR_I2C;
        return status;
    }

    status.lsr = static_cast<uint8_t>(lsr);
    status.msr = static_cast<uint8_t>(msr);
    status.txlvl = static_cast<uint8_t>(txlvl);
    status.rxlvl = static_cast<uint8_t>(rxlvl);

    return status;
}

int SC16IS752::getErrorStatus(uint8_t channel) {
    if (!isValidChannel(channel)) return ERR_PARAM;

    int lsr = readReg(regAddr(REG_LSR, channel));
    if (lsr < 0) return ERR_I2C;

    int errorStatus = OK;
    if (lsr & LSR_OVERRUN_ERROR) errorStatus |= ERROR_OVERRUN;
    if (lsr & LSR_PARITY_ERROR) errorStatus |= ERROR_PARITY;
    if (lsr & LSR_FRAMING_ERROR) errorStatus |= ERROR_FRAMING;
    if (lsr & LSR_BREAK_INTERRUPT) errorStatus |= ERROR_BREAK;
    if (lsr & LSR_FIFO_ERROR) errorStatus |= ERROR_FIFO;

    return errorStatus;
}

bool SC16IS752::isTxEmpty(uint8_t channel) {
    if (!isValidChannel(channel)) return false;
    int lsr = readReg(regAddr(REG_LSR, channel));
    return (lsr >= 0) && (lsr & LSR_THR_EMPTY);
}

bool SC16IS752::isRxAvailable(uint8_t channel) {
    if (!isValidChannel(channel)) return false;
    int lsr = readReg(regAddr(REG_LSR, channel));
    return (lsr >= 0) && (lsr & LSR_DATA_READY);
}

//=====================================================================
// Optimized Transfer Operations
//=====================================================================

SC16IS752::TransferResult SC16IS752::writeBufferOptimized(uint8_t channel, const uint8_t* buffer, size_t length) {
    TransferResult result;
    result.bytesTransferred = 0;
    result.error = OK;
    result.complete = false;
    result.timeMs = 0;

    if (!isValidChannel(channel) || !buffer || length > MAX_TRANSFER_SIZE) {
        result.error = ERR_PARAM;
        _stats.recordTransfer(false, 0, 0);
        return result;
    }

    // Adjust timing based on transfer size
    _timings.adjustForPacketSize(length);
    unsigned long startTime = millis();

    // Fast initial state check
    int lsr = readReg(regAddr(REG_LSR, channel));
    if (!(lsr & LSR_THR_EMPTY)) {
        delayMicroseconds(_timings.STATUS_CHECK_DELAY_US);
        lsr = readReg(regAddr(REG_LSR, channel));
        if (!(lsr & LSR_THR_EMPTY)) {
            result.error = ERR_BUSY;
            _stats.recordTransfer(false, 0, 0);
            return result;
        }
    }

    size_t bytesRemaining = length;
    size_t currentIndex = 0;

    while (bytesRemaining > 0) {
        int txlvl = readReg(regAddr(REG_TXLVL, channel));
        if (txlvl <= 0) {
            delayMicroseconds(_timings.STATUS_CHECK_DELAY_US);
            continue;
        }

        uint8_t chunkSize = getOptimalChunkSize(lsr, txlvl, bytesRemaining,
                                              _stats.currentSuccessiveTransfers > 50);

        for (uint8_t i = 0; i < chunkSize; i++) {
            if (writeReg(regAddr(REG_THR, channel), buffer[currentIndex + i]) != OK) {
                result.timeMs = millis() - startTime;
                result.error = ERR_I2C;
                _stats.recordTransfer(false, result.bytesTransferred, result.timeMs);
                return result;
            }
            result.bytesTransferred++;
            bytesRemaining--;
            delayMicroseconds(_timings.INTER_BYTE_DELAY_US);
        }

        currentIndex += chunkSize;
        delayMicroseconds(_timings.INTER_CHUNK_DELAY_US);

        // Check LSR for errors
        lsr = readReg(regAddr(REG_LSR, channel));
        if (lsr & (LSR_FRAMING_ERROR | LSR_PARITY_ERROR | LSR_OVERRUN_ERROR | LSR_BREAK_INTERRUPT)) {
            result.error = ERR_FIFO;
            break;
        }
    }

    result.timeMs = millis() - startTime;
    result.complete = (result.bytesTransferred == length);
    _stats.recordTransfer(result.complete, result.bytesTransferred, result.timeMs);

    adjustDelaysBasedOnPerformance();
    return result;
}

SC16IS752::TransferResult SC16IS752::readBufferOptimized(uint8_t channel, uint8_t* buffer, size_t length) {
    TransferResult result;
    result.bytesTransferred = 0;
    result.error = OK;
    result.complete = false;
    result.timeMs = 0;

    if (!isValidChannel(channel) || !buffer || length > MAX_TRANSFER_SIZE) {
        result.error = ERR_PARAM;
        _stats.recordTransfer(false, 0, 0);
        return result;
    }

    unsigned long startTime = millis();
    size_t bytesRemaining = length;
    size_t currentIndex = 0;

    while (bytesRemaining > 0) {
        // Check for timeout
        if (millis() - startTime > RX_TIMEOUT_MS) {
            result.error = ERR_TIMEOUT;
            break;
        }

        int rxlvl = readReg(regAddr(REG_RXLVL, channel));
        if (rxlvl <= 0) {
            delayMicroseconds(_timings.STATUS_CHECK_DELAY_US);
            continue;
        }

        // Read available data
        uint8_t chunkSize = min((size_t)rxlvl, bytesRemaining);
        for (uint8_t i = 0; i < chunkSize; i++) {
            int value = readReg(regAddr(REG_RHR, channel));
            if (value < 0) {
                result.timeMs = millis() - startTime;
                result.error = ERR_I2C;
                _stats.recordTransfer(false, result.bytesTransferred, result.timeMs);
                return result;
            }

            buffer[currentIndex + i] = value;
            result.bytesTransferred++;
            bytesRemaining--;

            if (_stats.currentSuccessiveTransfers > 50) {
                delayMicroseconds(_timings.INTER_BYTE_DELAY_US / 2);
            } else {
                delayMicroseconds(_timings.INTER_BYTE_DELAY_US);
            }
        }

        currentIndex += chunkSize;
        delayMicroseconds(_timings.INTER_CHUNK_DELAY_US);

        // Check for errors
        int lsr = readReg(regAddr(REG_LSR, channel));
        if (lsr & (LSR_FRAMING_ERROR | LSR_PARITY_ERROR | LSR_OVERRUN_ERROR | LSR_BREAK_INTERRUPT)) {
            result.error = ERR_FIFO;
            break;
        }
    }

    result.timeMs = millis() - startTime;
    result.complete = (result.bytesTransferred == length);
    _stats.recordTransfer(result.complete, result.bytesTransferred, result.timeMs);

    return result;
}

//=====================================================================
// Simple Transfer Operations
//=====================================================================

int SC16IS752::writeByte(uint8_t channel, uint8_t data) {
    if (!isValidChannel(channel)) return ERR_PARAM;

    if (!waitForTxReady(channel, TX_TIMEOUT_MS)) {
        return ERR_TIMEOUT;
    }

    return writeReg(regAddr(REG_THR, channel), data);
}

int SC16IS752::readByte(uint8_t channel) {
    if (!isValidChannel(channel)) return ERR_PARAM;

    if (!waitForRxData(channel, RX_TIMEOUT_MS)) {
        return ERR_TIMEOUT;
    }

    return readReg(regAddr(REG_RHR, channel));
}

size_t SC16IS752::writeBytes(uint8_t channel, const uint8_t* data, size_t length) {
    TransferResult result = writeBufferChunked(channel, data, length);
    return result.bytesTransferred;
}

size_t SC16IS752::readBytes(uint8_t channel, uint8_t* buffer, size_t length) {
    TransferResult result = readBufferChunked(channel, buffer, length);
    return result.bytesTransferred;
}

//=====================================================================
// GPIO Implementation
//=====================================================================

void SC16IS752::pinMode(uint8_t pin, uint8_t mode) {
    if (!isValidGPIOPin(pin)) return;

    // Configure pin for GPIO operation first
    uint8_t ioControl = readReg(REG_IOControl);
    ioControl |= (1 << pin);  // Set pin for GPIO mode
    writeReg(REG_IOControl, ioControl);

    // Set direction using renamed constants
    updateGPIORegister(REG_IODir, pin, mode == GPIO_INPUT ? 1 : 0);
}

void SC16IS752::digitalWrite(uint8_t pin, uint8_t state) {
    if (!isValidGPIOPin(pin)) return;

    // Only write if pin is configured as output
    if (!(readGPIORegister(REG_IODir, pin))) {
        updateGPIORegister(REG_IOState, pin, state ? 1 : 0);
    }
}

uint8_t SC16IS752::digitalRead(uint8_t pin) {
    if (!isValidGPIOPin(pin)) return GPIO_LOW;

    return readGPIORegister(REG_IOState, pin) ? GPIO_HIGH : GPIO_LOW;
}

void SC16IS752::updateGPIORegister(uint8_t reg, uint8_t pin, uint8_t value) {
    uint8_t currentValue;
    if (reg == REG_IODir) {
        currentValue = _gpioDirection;
    } else if (reg == REG_IOState) {
        currentValue = _gpioState;
    } else {
        currentValue = readReg(reg);
    }

    if (value) {
        currentValue |= (1 << pin);
    } else {
        currentValue &= ~(1 << pin);
    }

    writeReg(reg, currentValue);

    // Update cache
    if (reg == REG_IODir) {
        _gpioDirection = currentValue;
    } else if (reg == REG_IOState) {
        _gpioState = currentValue;
    }
}

uint8_t SC16IS752::readGPIORegister(uint8_t reg, uint8_t pin) {
    uint8_t value;
    if (reg == REG_IODir) {
        value = _gpioDirection;
    } else if (reg == REG_IOState) {
        value = _gpioState;
    } else {
        value = readReg(reg);
    }

    return (value & (1 << pin)) ? 1 : 0;
}

//=====================================================================
// PWM Implementation
//=====================================================================

bool SC16IS752::pwmBegin(uint8_t pin, uint32_t frequency, PWMResolution resolution) {
    if (!isValidGPIOPin(pin)) return false;

    // Configure pin for GPIO operation first
    pinMode(pin, GPIO_OUTPUT);

    // Initialize PWM state
    _pwmStates[pin].enabled = true;
    _pwmStates[pin].frequency = frequency;
    _pwmStates[pin].resolution = resolution;
    _pwmStates[pin].dutyCycle = 0;

    // Calculate and set prescaler
    updatePWMPrescaler(pin);

    // Configure PWM settings
    updatePWMSettings(pin);

    // Set initial duty cycle to 0
    pwmWrite(pin, 0);

    return true;
}

void SC16IS752::pwmWrite(uint8_t pin, uint32_t value) {
    if (!isValidGPIOPin(pin) || !_pwmStates[pin].enabled) return;

    // Clamp value to resolution
    uint32_t maxValue = pwmGetMaxValue(pin);
    value = min(value, maxValue);

    _pwmStates[pin].dutyCycle = value;

    // Calculate register values based on resolution
    uint32_t regValue = (value * maxValue) >> static_cast<uint8_t>(_pwmStates[pin].resolution);

    // Write duty cycle registers
    writeReg(REG_PWM_DUTY + (pin * 2), regValue & 0xFF);
    writeReg(REG_PWM_DUTY + (pin * 2) + 1, (regValue >> 8) & 0xFF);
}

void SC16IS752::pwmWriteHR(uint8_t pin, uint32_t value) {
    if (!isValidGPIOPin(pin) || !_pwmStates[pin].enabled) return;

    // Direct write without scaling
    uint32_t maxValue = pwmGetMaxValue(pin);
    value = min(value, maxValue);

    _pwmStates[pin].dutyCycle = value;

    // Write duty cycle registers directly
    writeReg(REG_PWM_DUTY + (pin * 2), value & 0xFF);
    writeReg(REG_PWM_DUTY + (pin * 2) + 1, (value >> 8) & 0xFF);
}

void SC16IS752::updatePWMPrescaler(uint8_t pin) {
    if (!isValidGPIOPin(pin) || !_pwmStates[pin].enabled) return;

    uint32_t prescaler = calculatePrescaler(
        _pwmStates[pin].frequency,
        _pwmStates[pin].resolution
    );

    _pwmStates[pin].prescaler = prescaler;

    // Write prescaler registers
    writeReg(REG_PWM_PRE + (pin * 2), prescaler & 0xFF);
    writeReg(REG_PWM_PRE + (pin * 2) + 1, (prescaler >> 8) & 0xFF);
}

void SC16IS752::pwmEnd(uint8_t pin) {
    if (!isValidGPIOPin(pin)) return;

    // Disable PWM mode
    uint8_t pwmCtrl = readReg(REG_PWM_CTRL);
    pwmCtrl &= ~(1 << pin);  // Disable PWM for this pin
    writeReg(REG_PWM_CTRL, pwmCtrl);

    // Reset state
    _pwmStates[pin].enabled = false;
    _pwmStates[pin].dutyCycle = 0;

    // Return pin to normal GPIO output mode
    pinMode(pin, GPIO_OUTPUT);
    digitalWrite(pin, GPIO_LOW);
}

void SC16IS752::analogWrite(uint8_t pin, uint8_t value) {
    if (!_pwmStates[pin].enabled) {
        pwmBegin(pin);
    }
    pwmWrite(pin, value);
}

//=====================================================================
// Timing and Performance Functions
//=====================================================================

void SC16IS752::adjustDelaysBasedOnPerformance() {
    if (_stats.currentSuccessiveTransfers > 100 &&
        _stats.averageRate > TransferTimings::SUSTAINED_RATE_BPS_SMALL * 0.95) {
        // High performance mode
        _timings.INTER_BYTE_DELAY_US = max(40UL, _timings.INTER_BYTE_DELAY_US - 1);
        _timings.INTER_CHUNK_DELAY_US = max(160UL, _timings.INTER_CHUNK_DELAY_US - 2);
        _timings.STATUS_CHECK_DELAY_US = max(80UL, _timings.STATUS_CHECK_DELAY_US - 1);
    } else if (_stats.currentSuccessiveTransfers > 50) {
        // Normal optimized mode
        _timings.INTER_BYTE_DELAY_US = 45;
        _timings.INTER_CHUNK_DELAY_US = 180;
        _timings.STATUS_CHECK_DELAY_US = 90;
    } else if (_stats.currentSuccessiveTransfers < 10) {
        // Conservative mode
        _timings.INTER_BYTE_DELAY_US = 55;
        _timings.INTER_CHUNK_DELAY_US = 220;
        _timings.STATUS_CHECK_DELAY_US = 100;
    }
}

uint8_t SC16IS752::getOptimalChunkSize(int lsr, int txlvl, size_t remaining, bool burstMode) {
    size_t maxChunk;

    if (burstMode && (lsr & LSR_THR_EMPTY) &&
        _stats.averageRate > TransferTimings::SUSTAINED_RATE_BPS_SMALL * 0.9) {
        // Aggressive chunk size for high-performance scenarios
        maxChunk = 16;
    } else if (lsr & LSR_THR_EMPTY) {
        // Standard chunk size for stable operation
        maxChunk = 8;
    } else {
        // Conservative chunk size when FIFO is partially full
        maxChunk = 4;
    }

    return min(min(maxChunk, remaining), (size_t)txlvl);
}

bool SC16IS752::waitForTxReady(uint8_t channel, unsigned long timeoutMs) {
    unsigned long startTime = millis();
    while (millis() - startTime < timeoutMs) {
        if (isTxEmpty(channel)) {
            return true;
        }
        delayMicroseconds(_timings.STATUS_CHECK_DELAY_US);
    }
    return false;
}

bool SC16IS752::waitForRxData(uint8_t channel, unsigned long timeoutMs) {
    unsigned long startTime = millis();
    while (millis() - startTime < timeoutMs) {
        if (isRxAvailable(channel)) {
            return true;
        }
        delayMicroseconds(_timings.STATUS_CHECK_DELAY_US);
    }
    return false;
}

unsigned long SC16IS752::calculateByteTransmitTime(uint32_t baudRate) {
    if (baudRate == 0) return 1000;  // Default to 1ms if invalid baud rate

    // Calculate time for one byte (10 bits: start + 8 data + stop)
    // Multiply by 2 for safety margin
    return (20000000UL / baudRate);  // Result in microseconds
}

//=====================================================================
// PWM Helper Functions
//=====================================================================

uint32_t SC16IS752::calculatePrescaler(uint32_t targetFreq, PWMResolution resolution) const {
    uint32_t clockFreq = getClockFrequency();
    uint32_t steps = 1UL << static_cast<uint8_t>(resolution);
    
    // Calculate ideal prescaler value
    uint32_t prescaler = (clockFreq / (targetFreq * steps)) - 1;
    
    // Clamp to 16-bit value
    return min(prescaler, 65535UL);
}

uint32_t SC16IS752::getClockFrequency() const {
    // Return appropriate clock frequency based on clock source
    if (_pwmStates[0].clockSource == PWMClockSource::INTERNAL) {
        return 1843200;  // Internal oscillator frequency
    } else {
        return 14745600;  // Default crystal frequency
    }
}

void SC16IS752::updatePWMSettings(uint8_t pin) {
    if (!isValidGPIOPin(pin) || !_pwmStates[pin].enabled) return;
    
    uint8_t ctrlReg = readReg(REG_PWM_CTRL);
    
    // Clear resolution bits for this pin (2 bits per pin)
    uint8_t pinShift = (pin * 2);
    ctrlReg &= ~(0x03 << pinShift);
    
    // Set new resolution
    uint8_t resValue;
    switch (_pwmStates[pin].resolution) {
        case PWMResolution::PWM_8_BIT:  resValue = 0; break;
        case PWMResolution::PWM_10_BIT: resValue = 1; break;
        case PWMResolution::PWM_12_BIT: resValue = 2; break;
        case PWMResolution::PWM_16_BIT: resValue = 3; break;
        default: resValue = 0;
    }
    
    ctrlReg |= (resValue << pinShift);
    writeReg(REG_PWM_CTRL, ctrlReg);
}

//=====================================================================
// Statistics Management
//=====================================================================

void SC16IS752::updateTransferStatistics(bool success, size_t bytes, uint32_t timeMs) {
    _stats.recordTransfer(success, bytes, timeMs);

    if (_stats.currentSuccessiveTransfers > 100 &&
        _stats.averageRate > TransferTimings::SUSTAINED_RATE_BPS_SMALL * 0.95) {
        // Optimize timings for high performance
        adjustDelaysBasedOnPerformance();
    } else if (_stats.currentSuccessiveTransfers < 10) {
        // Reset to conservative timings
        _timings.adjustForPacketSize(_stats.totalBytes);
    }
}

// Fix the getTransferStats method implementation
SC16IS752::TransferStats SC16IS752::getTransferStats() const {
    return _stats;
}

void SC16IS752::resetTransferStats() {
    _stats.reset();
    _timings = TransferTimings();
}

//=====================================================================
// Validation Functions
//=====================================================================

bool SC16IS752::isValidChannel(uint8_t channel) const {
    return channel < MAX_CHANNELS;
}

bool SC16IS752::isValidGPIOPin(uint8_t pin) const {
    return pin <= GPIO_7;
}

//=====================================================================
// Chunked Transfer Operations
//=====================================================================

SC16IS752::TransferResult SC16IS752::writeBufferChunked(uint8_t channel, const uint8_t* buffer, size_t length) {
    TransferResult result;
    result.bytesTransferred = 0;
    result.error = OK;
    result.complete = false;
    result.timeMs = 0;

    if (!isValidChannel(channel) || !buffer || length == 0 || length > MAX_TRANSFER_SIZE) {
        result.error = ERR_PARAM;
        _stats.recordTransfer(false, 0, 0);
        return result;
    }

    _timings.adjustForPacketSize(length);
    unsigned long startTime = millis();
    size_t currentOffset = 0;

    while (currentOffset < length) {
        if (!waitForTxReady(channel, TX_TIMEOUT_MS)) {
            result.error = ERR_TIMEOUT;
            break;
        }

        int txlvl = readReg(regAddr(REG_TXLVL, channel));
        if (txlvl <= 0) continue;

        // Get LSR for optimal chunk size calculation
        int lsr = readReg(regAddr(REG_LSR, channel));
        uint8_t chunkSize = getOptimalChunkSize(
            lsr,
            txlvl,
            length - currentOffset,
            _stats.currentSuccessiveTransfers > 50
        );

        for (uint8_t i = 0; i < chunkSize; i++) {
            if (writeReg(regAddr(REG_THR, channel), buffer[currentOffset + i]) != OK) {
                result.error = ERR_I2C;
                break;
            }
            result.bytesTransferred++;
        }

        if (result.error != OK) break;
        currentOffset += chunkSize;

        // Dynamic delay based on performance
        if (_stats.currentSuccessiveTransfers > 50) {
            delayMicroseconds(_timings.INTER_CHUNK_DELAY_US / 2);
        } else {
            delayMicroseconds(_timings.INTER_CHUNK_DELAY_US);
        }
    }

    result.timeMs = millis() - startTime;
    result.complete = (result.bytesTransferred == length);
    _stats.recordTransfer(result.complete, result.bytesTransferred, result.timeMs);

    return result;
}

SC16IS752::TransferResult SC16IS752::readBufferChunked(uint8_t channel, uint8_t* buffer, size_t length) {
    TransferResult result;
    result.bytesTransferred = 0;
    result.error = OK;
    result.complete = false;
    result.timeMs = 0;

    if (!isValidChannel(channel) || !buffer || length == 0 || length > MAX_TRANSFER_SIZE) {
        result.error = ERR_PARAM;
        _stats.recordTransfer(false, 0, 0);
        return result;
    }

    unsigned long startTime = millis();
    size_t currentOffset = 0;

    while (currentOffset < length) {
        if (!waitForRxData(channel, RX_TIMEOUT_MS)) {
            result.error = ERR_TIMEOUT;
            break;
        }

        int rxlvl = readReg(regAddr(REG_RXLVL, channel));
        if (rxlvl <= 0) continue;

        uint8_t chunkSize = min((uint8_t)rxlvl, (uint8_t)(length - currentOffset));

        // Read the chunk
        for (uint8_t i = 0; i < chunkSize; i++) {
            int value = readReg(regAddr(REG_RHR, channel));
            if (value < 0) {
                result.error = ERR_I2C;
                break;
            }
            buffer[currentOffset + i] = value;
            result.bytesTransferred++;

            // Dynamic delay based on performance
            if (_stats.currentSuccessiveTransfers > 50) {
                delayMicroseconds(_timings.INTER_BYTE_DELAY_US / 2);
            } else {
                delayMicroseconds(_timings.INTER_BYTE_DELAY_US);
            }
        }

        if (result.error != OK) break;
        currentOffset += chunkSize;

        // Dynamic delay between chunks
        if (_stats.currentSuccessiveTransfers > 50) {
            delayMicroseconds(_timings.INTER_CHUNK_DELAY_US / 2);
        } else {
            delayMicroseconds(_timings.INTER_CHUNK_DELAY_US);
        }

        // Check for errors
        int lsr = readReg(regAddr(REG_LSR, channel));
        if (lsr & (LSR_FRAMING_ERROR | LSR_PARITY_ERROR | LSR_OVERRUN_ERROR | LSR_BREAK_INTERRUPT)) {
            result.error = ERR_FIFO;
            break;
        }
    }

    result.timeMs = millis() - startTime;
    result.complete = (result.bytesTransferred == length);
    _stats.recordTransfer(result.complete, result.bytesTransferred, result.timeMs);

    return result;
}

//=====================================================================
// TransferTimings Implementation
//=====================================================================

void SC16IS752::TransferTimings::adjustForPacketSize(size_t packetSize) {
    if (packetSize <= SMALL_PACKET_THRESHOLD) {
        // Optimized timings for small packets
        INTER_BYTE_DELAY_US = 45;
        INTER_CHUNK_DELAY_US = 180;
        STATUS_CHECK_DELAY_US = 90;
    } else {
        // Conservative timings for larger packets
        INTER_BYTE_DELAY_US = 55;
        INTER_CHUNK_DELAY_US = 220;
        STATUS_CHECK_DELAY_US = 110;
    }
}

