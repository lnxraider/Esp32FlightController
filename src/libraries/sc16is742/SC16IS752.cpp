// SC16IS752.cpp
#include "SC16IS752.h"

//=====================================================================
// Constructor & Initialization
//=====================================================================

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
SC16IS752::SC16IS752(TwoWire& wire, const I2CPins& pins) :
    _wire(wire),
    _pins(pins),
    _stats(),
    _timings(),
    _currentBaudRate(0),
    _i2cFreq(DEFAULT_I2C_FREQ),
    _i2cAddr(0),
    _analogReadBits(DEFAULT_ADC_BITS),
    _initialized(false) {
    // Initialize PWM configurations
    for (int i = 0; i < 8; i++) {
        _pwmConfig[i] = PWMConfig();
    }
}
#else
SC16IS752::SC16IS752(TwoWire& wire) :
    _wire(wire),
    _stats(),
    _timings(),
    _currentBaudRate(0),
    _i2cFreq(DEFAULT_I2C_FREQ),
    _i2cAddr(0),
    _analogReadBits(DEFAULT_ADC_BITS),
    _initialized(false) {
    // Initialize PWM configurations
    for (int i = 0; i < 8; i++) {
        _pwmConfig[i] = PWMConfig();
    }
}
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

    if (!detectDevice()) {
        if (DEBUG_ENABLED) {
            Serial.println(F("Device not detected"));
        }
        return false;
    }
    
    // Initialize all GPIO pins as inputs (high impedance state)
    if (writeReg(regAddr(REG_IODIR, CHANNEL_A), 0xFF) != OK) {
        return false;
    }
    
    // Disable all GPIO interrupts initially
    if (writeReg(regAddr(REG_IOINTENA, CHANNEL_A), 0x00) != OK) {
        return false;
    }
    
    // Reset all PWM configurations
    if (writeReg(regAddr(REG_EFCR, CHANNEL_A), 0x00) != OK) {
        return false;
    }

    _initialized = true;
    return true;
}

void SC16IS752::end() {
    if (_initialized) {
        // Reset all GPIO pins to inputs
        writeReg(regAddr(REG_IODIR, CHANNEL_A), 0xFF);
        // Disable all PWM outputs
        writeReg(regAddr(REG_EFCR, CHANNEL_A), 0x00);
        
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
// GPIO Methods
//=====================================================================

bool SC16IS752::isValidGPIO(uint8_t pin) const {
    return pin <= GPIO_7;
}

bool SC16IS752::pinMode(uint8_t pin, uint8_t mode) {
    if (!isValidGPIO(pin)) {
        return false;
    }

    // Read current direction register
    int iodir = readReg(regAddr(REG_IODIR, CHANNEL_A));
    if (iodir < 0) {
        return false;
    }

    // Set direction bit (1 = input, 0 = output)
    if (mode == INPUT) {
        iodir |= (1 << pin);
    } else {
        iodir &= ~(1 << pin);
        
        // If configuring as output, disable PWM for this pin
        int efcr = readReg(regAddr(REG_EFCR, CHANNEL_A));
        if (efcr >= 0) {
            efcr &= ~(1 << (pin + 4)); // Clear PWM enable bit
            writeReg(regAddr(REG_EFCR, CHANNEL_A), efcr);
        }
    }

    // Write back to direction register
    return (writeReg(regAddr(REG_IODIR, CHANNEL_A), iodir) == OK);
}

bool SC16IS752::digitalWrite(uint8_t pin, uint8_t state) {
    if (!isValidGPIO(pin)) {
        return false;
    }

    // Check if pin is configured as output
    int iodir = readReg(regAddr(REG_IODIR, CHANNEL_A));
    if (iodir < 0 || (iodir & (1 << pin))) {
        return false; // Pin is not configured as output
    }

    // Read current state
    int iostate = readReg(regAddr(REG_IOD, CHANNEL_A));
    if (iostate < 0) {
        return false;
    }

    // Set or clear the pin
    if (state == HIGH) {
        iostate |= (1 << pin);
    } else {
        iostate &= ~(1 << pin);
    }

    // Write back to state register
    return (writeReg(regAddr(REG_IOD, CHANNEL_A), iostate) == OK);
}

int SC16IS752::digitalRead(uint8_t pin) {
    if (!isValidGPIO(pin)) {
        return -1;
    }

    // Read current state
    int iostate = readReg(regAddr(REG_IOSTATE, CHANNEL_A));
    if (iostate < 0) {
        return -1;
    }

    // Return pin state (HIGH or LOW)
    return (iostate & (1 << pin)) ? HIGH : LOW;
}

//=====================================================================
// Analog Read Implementation
//=====================================================================

void SC16IS752::analogReadResolution(uint8_t bits) {
    if (bits < MIN_ADC_BITS) bits = MIN_ADC_BITS;
    if (bits > MAX_ADC_BITS) bits = MAX_ADC_BITS;
    _analogReadBits = bits;
}

bool SC16IS752::configureADC(uint8_t pin) {
    if (!isValidGPIO(pin)) {
        return false;
    }

    // Configure GPIO pin for analog input
    if (!pinMode(pin, INPUT)) {
        return false;
    }

    // Select ADC channel
    if (writeReg(regAddr(REG_ADCSEL, CHANNEL_A), pin) != OK) {
        return false;
    }

    // Configure ADC resolution and enable
    uint8_t adcConfig = 0x01;  // Enable ADC

    switch (_analogReadBits) {
        case 8:  adcConfig |= 0x00; break;
        case 9:  adcConfig |= 0x20; break;
        case 10: adcConfig |= 0x40; break;
        case 11: adcConfig |= 0x60; break;
        case 12: adcConfig |= 0x80; break;
        default: adcConfig |= 0x40; break; // Default to 10-bit
    }

    return (writeReg(regAddr(REG_ADCCON, CHANNEL_A), adcConfig) == OK);
}

int SC16IS752::analogRead(uint8_t pin) {
    if (!isValidGPIO(pin)) {
        return -1;
    }

    if (!configureADC(pin)) {
        return -1;
    }

    // Wait for conversion
    delay(2);  // Typical ADC conversion time

    // Read ADC data
    int adcLow = readReg(regAddr(REG_ADCDAT, CHANNEL_A));
    int adcHigh = readReg(regAddr(REG_ADCDAT + 1, CHANNEL_A));

    if (adcLow < 0 || adcHigh < 0) {
        return -1;
    }

    // Combine readings
    uint16_t rawValue = (adcHigh << 8) | adcLow;

    // Scale to requested resolution
    uint16_t maxValue = (1 << _analogReadBits) - 1;
    return map(rawValue, 0, 4095, 0, maxValue);
}

//=====================================================================
// PWM Implementation
//=====================================================================

bool SC16IS752::configureGPIOForPWM(uint8_t pin) {
    if (!isValidGPIO(pin)) {
        return false;
    }

    // Configure pin as output
    if (!pinMode(pin, OUTPUT)) {
        return false;
    }

    // Read current EFCR
    int efcr = readReg(regAddr(REG_EFCR, CHANNEL_A));
    if (efcr < 0) {
        return false;
    }

    // Enable PWM for this pin
    efcr |= (1 << (pin + 4));  // PWM enable bits start at bit 4

    return (writeReg(regAddr(REG_EFCR, CHANNEL_A), efcr) == OK);
}

bool SC16IS752::pwmConfig(uint8_t pin, uint32_t frequency, uint16_t resolution) {
    if (!isValidGPIO(pin)) {
        return false;
    }

    // Validate frequency and resolution
    if (frequency < MIN_PWM_FREQUENCY || frequency > MAX_PWM_FREQUENCY ||
        resolution < MIN_PWM_RESOLUTION || resolution > MAX_PWM_RESOLUTION) {
        return false;
    }

    // Store configuration
    _pwmConfig[pin].frequency = frequency;
    _pwmConfig[pin].resolution = resolution;

    // Calculate and apply PWM settings
    if (!calculatePWMSettings(pin)) {
        return false;
    }

    // Configure GPIO for PWM
    return configureGPIOForPWM(pin);
}

bool SC16IS752::calculatePWMSettings(uint8_t pin) {
    PWMConfig& config = _pwmConfig[pin];

    // Calculate optimal divider for desired frequency and resolution
    config.clockDivider = calculateOptimalDivider(config.frequency, config.resolution);

    if (config.clockDivider == 0) {
        return false;
    }

    // Configure PWM clock prescaler
    return (writeReg(regAddr(REG_PWMCFG, CHANNEL_A),
                    (config.clockDivider - 1) & 0xFF) == OK);
}

uint32_t SC16IS752::calculateOptimalDivider(uint32_t targetFreq, uint16_t resolution) {
    uint32_t maxCount = (1UL << resolution) - 1;
    uint32_t desiredDivider = XTAL_FREQ / (targetFreq * maxCount);

    // Round to nearest available divider (1-256)
    if (desiredDivider < 1) return 1;
    if (desiredDivider > 256) return 256;

    return desiredDivider;
}

bool SC16IS752::analogWrite(uint8_t pin, uint32_t value) {
    if (!isValidGPIO(pin)) {
        return false;
    }

    // Get pin's PWM configuration
    PWMConfig& config = _pwmConfig[pin];
    uint32_t maxValue = (1UL << config.resolution) - 1;

    // Constrain value to resolution
    if (value > maxValue) {
        value = maxValue;
    }

    // Scale value to 8-bit for hardware register
    uint8_t scaledValue;
    if (config.resolution > 8) {
        scaledValue = value >> (config.resolution - 8);
    } else {
        scaledValue = value << (8 - config.resolution);
    }

    return (writeReg(regAddr(REG_PWMDUTY + pin, CHANNEL_A), scaledValue) == OK);
}

bool SC16IS752::analogWriteResolution(uint8_t pin, uint16_t resolution) {
    if (!isValidGPIO(pin) ||
        resolution < MIN_PWM_RESOLUTION ||
        resolution > MAX_PWM_RESOLUTION) {
        return false;
    }

    _pwmConfig[pin].resolution = resolution;
    return calculatePWMSettings(pin);
}

bool SC16IS752::analogWriteFrequency(uint8_t pin, uint32_t frequency) {
    if (!isValidGPIO(pin) ||
        frequency < MIN_PWM_FREQUENCY ||
        frequency > MAX_PWM_FREQUENCY) {
        return false;
    }

    _pwmConfig[pin].frequency = frequency;
    return calculatePWMSettings(pin);
}

SC16IS752::PWMConfig SC16IS752::getPWMConfig(uint8_t pin) {
    if (isValidGPIO(pin)) {
        return _pwmConfig[pin];
    }
    return PWMConfig(); // Return default config if invalid pin
}

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
        // Normal operation mode setup
        // Configure FIFOs with standard triggers
        result = writeReg(regAddr(REG_FCR, channel),
                         FCR_FIFO_ENABLE | FCR_TX_TRIGGER_LVL2 | FCR_RX_TRIGGER_LVL2);
        if (result != OK) return result;
        delay(10);

        // Configure modem control
        result = writeReg(regAddr(REG_MCR, channel), MCR_RTS | MCR_DTR);
        if (result != OK) return result;
        delay(10);
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
// Status and Control Methods
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
// Transfer Operations
//=====================================================================

SC16IS752::TransferResult SC16IS752::writeBufferOptimized(uint8_t channel, const uint8_t* buffer, size_t length) {
    TransferResult result;
    result.bytesTransferred = 0;
    result.error = OK;
    result.complete = false;
    result.timeMs = 0;

    if (!isValidChannel(channel) || !buffer || length != TransferStates::BUFFER_SIZE) {
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

    // Write first chunk (8 bytes)
    for (size_t i = 0; i < TransferStates::CHUNK_SIZE; i++) {
        if (writeReg(regAddr(REG_THR, channel), buffer[i]) != OK) {
            result.timeMs = millis() - startTime;
            result.error = ERR_I2C;
            _stats.recordTransfer(false, result.bytesTransferred, result.timeMs);
            return result;
        }
        result.bytesTransferred++;
        delayMicroseconds(_timings.INTER_BYTE_DELAY_US);
    }

    delayMicroseconds(_timings.INTER_CHUNK_DELAY_US);

    // Write second chunk (8 bytes)
    for (size_t i = TransferStates::CHUNK_SIZE; i < TransferStates::BUFFER_SIZE; i++) {
        if (writeReg(regAddr(REG_THR, channel), buffer[i]) != OK) {
            result.timeMs = millis() - startTime;
            result.error = ERR_I2C;
            _stats.recordTransfer(false, result.bytesTransferred, result.timeMs);
            return result;
        }
        result.bytesTransferred++;
        delayMicroseconds(_timings.INTER_BYTE_DELAY_US);
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

    if (!isValidChannel(channel) || !buffer || length != TransferStates::BUFFER_SIZE) {
        result.error = ERR_PARAM;
        _stats.recordTransfer(false, 0, 0);
        return result;
    }

    unsigned long startTime = millis();

    // Wait for FIFO to fill
    delay(_timings.FIFO_WAIT_MS);

    // Quick FIFO check
    int rxlvl = readReg(regAddr(REG_RXLVL, channel));
    if (rxlvl != TransferStates::FIFO_FULL) {
        delayMicroseconds(_timings.STATUS_CHECK_DELAY_US);
        rxlvl = readReg(regAddr(REG_RXLVL, channel));
        if (rxlvl != TransferStates::FIFO_FULL) {
            result.error = ERR_TIMEOUT;
            _stats.recordTransfer(false, 0, 0);
            return result;
        }
    }

    // Read all bytes with optimized timing
    for (size_t i = 0; i < length; i++) {
        int value = readReg(regAddr(REG_RHR, channel));
        if (value < 0) {
            result.timeMs = millis() - startTime;
            result.error = ERR_I2C;
            _stats.recordTransfer(false, result.bytesTransferred, result.timeMs);
            return result;
        }
        buffer[i] = value;
        result.bytesTransferred++;

        // Dynamic delay based on performance
        if (_stats.currentSuccessiveTransfers > 50) {
            delayMicroseconds(_timings.INTER_BYTE_DELAY_US / 2);
        } else {
            delayMicroseconds(_timings.INTER_BYTE_DELAY_US);
        }
    }

    result.timeMs = millis() - startTime;
    result.complete = (result.bytesTransferred == length);
    _stats.recordTransfer(result.complete, result.bytesTransferred, result.timeMs);

    return result;
}

SC16IS752::TransferResult SC16IS752::writeBufferChunked(uint8_t channel, const uint8_t* buffer, size_t length) {
    TransferResult result;
    result.bytesTransferred = 0;
    result.error = OK;
    result.complete = false;
    result.timeMs = 0;

    if (!isValidChannel(channel) || !buffer || length == 0 || length > MAX_TRANSFER_SIZE) {
        result.error = ERR_PARAM;
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

        uint8_t chunkSize = getOptimalChunkSize(
            readReg(regAddr(REG_LSR, channel)),
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

        for (uint8_t i = 0; i < chunkSize; i++) {
            int value = readReg(regAddr(REG_RHR, channel));
            if (value < 0) {
                result.error = ERR_I2C;
                break;
            }
            buffer[currentOffset + i] = value;
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
// Helper Methods
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
        if (_stats.currentSuccessiveTransfers > 50) {
            delayMicroseconds(_timings.STATUS_CHECK_DELAY_US / 2);
        } else {
            delayMicroseconds(_timings.STATUS_CHECK_DELAY_US);
        }
    }
    return false;
}

bool SC16IS752::waitForRxData(uint8_t channel, unsigned long timeoutMs) {
    unsigned long startTime = millis();
    while (millis() - startTime < timeoutMs) {
        if (isRxAvailable(channel)) {
            return true;
        }
        if (_stats.currentSuccessiveTransfers > 50) {
            delayMicroseconds(_timings.STATUS_CHECK_DELAY_US / 2);
        } else {
            delayMicroseconds(_timings.STATUS_CHECK_DELAY_US);
        }
    }
    return false;
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
// Register Access Methods
//=====================================================================

int SC16IS752::writeReg(uint8_t reg, uint8_t value) {
    if (!_initialized && reg != regAddr(REG_SPR, CHANNEL_A)) return ERR_PARAM;

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
    if (!_initialized && reg != regAddr(REG_SPR, CHANNEL_A)) return ERR_PARAM;

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

//=====================================================================
// Statistics Management
//=====================================================================

void SC16IS752::updateTransferStatistics(bool success, size_t bytes, uint32_t timeMs) {
    _stats.recordTransfer(success, bytes, timeMs);

    // Update timing parameters based on performance
    if (_stats.currentSuccessiveTransfers > 100 &&
        _stats.averageRate > TransferTimings::SUSTAINED_RATE_BPS_SMALL * 0.95) {
        // Performing well, optimize further
        _timings.INTER_BYTE_DELAY_US = max(40UL, _timings.INTER_BYTE_DELAY_US - 1);
        _timings.INTER_CHUNK_DELAY_US = max(160UL, _timings.INTER_CHUNK_DELAY_US - 2);
        _timings.STATUS_CHECK_DELAY_US = max(80UL, _timings.STATUS_CHECK_DELAY_US - 1);
    } else if (_stats.currentSuccessiveTransfers < 10) {
        // Having issues, reset to safe values
        _timings.adjustForPacketSize(bytes);
        _timings.STATUS_CHECK_DELAY_US = 90;
    }
}

//=====================================================================
// TransferTimings Implementation
//=====================================================================

void SC16IS752::TransferTimings::adjustForPacketSize(size_t packetSize) {
    if (packetSize <= SMALL_PACKET_THRESHOLD) {
        INTER_BYTE_DELAY_US = 45;
        INTER_CHUNK_DELAY_US = 180;
    } else {
        INTER_BYTE_DELAY_US = 55;
        INTER_CHUNK_DELAY_US = 220;
    }
}

SC16IS752::TransferStats SC16IS752::getTransferStats() const {
    return _stats;
}

void SC16IS752::resetTransferStats() {
    _stats.reset();
    _timings = TransferTimings();
}

//=====================================================================
// Validation Methods
//=====================================================================

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

bool SC16IS752::isValidChannel(uint8_t channel) const {
    return channel < MAX_CHANNELS;
}

uint8_t SC16IS752::regAddr(uint8_t reg, uint8_t channel) const {
    return (reg << 3) | (channel ? 0x02 : 0x00);
}

//=====================================================================
// TransferStats Implementation
//=====================================================================

SC16IS752::TransferStats::TransferStats() {
    reset();
}

void SC16IS752::TransferStats::reset() {
    totalTransfers = 0;
    successfulTransfers = 0;
    totalBytes = 0;
    totalTimeMs = 0;
    averageRate = 0;
    maxSuccessiveTransfers = 0;
    currentSuccessiveTransfers = 0;
}

void SC16IS752::TransferStats::updateRate() {
    if (totalTimeMs > 0) {
        averageRate = (float)(totalBytes * 1000) / totalTimeMs;
    } else {
        averageRate = 0;
    }
}

void SC16IS752::TransferStats::recordTransfer(bool success, size_t bytes, uint32_t timeMs) {
    totalTransfers++;
    if (success) {
        successfulTransfers++;
        totalBytes += bytes;
        totalTimeMs += timeMs;
        currentSuccessiveTransfers++;
        if (currentSuccessiveTransfers > maxSuccessiveTransfers) {
            maxSuccessiveTransfers = currentSuccessiveTransfers;
        }
    } else {
        currentSuccessiveTransfers = 0;
    }
    updateRate();
}

