// SC16IS752.cpp
#include "SC16IS752.h"

//=====================================================================
// Constructor & Initialization
//=====================================================================

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
SC16IS752::SC16IS752(TwoWire& wire, const I2CPins& pins) : 
    _wire(wire),
    _initialized(false),
    _i2cAddr(0),
    _currentBaudRate(0),
    _pins(pins),
    _i2cFreq(DEFAULT_I2C_FREQ),
    _stats(),
    _timings() {}
#else
SC16IS752::SC16IS752(TwoWire& wire) : 
    _wire(wire),
    _initialized(false),
    _i2cAddr(0),
    _currentBaudRate(0),
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

    if (!detectDevice()) {
        if (DEBUG_ENABLED) {
            Serial.println(F("Device not detected"));
        }
        return false;
    }

    _initialized = true;
    return true;
}

void SC16IS752::end() {
    if (_initialized) {
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
    }

    // Disable all interrupts
    result = writeReg(regAddr(REG_IER, channel), 0x00);
    return result;
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
// Configuration Methods
//=====================================================================

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

SC16IS752::TransferStats SC16IS752::getTransferStats() const {
    return _stats;
}

void SC16IS752::resetTransferStats() {
    _stats.reset();

    // Reset timing parameters to default values
    _timings = TransferTimings();
}

//=====================================================================
// Utility Functions
//=====================================================================

unsigned long SC16IS752::calculateByteTransmitTime(uint32_t baudRate) {
    if (baudRate == 0) return 1000;  // Default to 1ms if invalid baud rate

    // Calculate time for one byte (10 bits: start + 8 data + stop)
    // Multiply by 2 for safety margin
    return (20000000UL / baudRate);  // Result in microseconds
}

