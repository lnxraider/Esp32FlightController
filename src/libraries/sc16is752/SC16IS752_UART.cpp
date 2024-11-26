// SC16IS752_UART.cpp

#include "SC16IS752_UART.h"

// Constructors
#if defined(ESP32) || defined(ESP8266)
SC16IS752_UART::SC16IS752_UART(uint8_t i2cAddr, TwoWire& wire, const I2CPins& pins)
  : SC16IS752(i2cAddr, wire, pins) {
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    _transferState[i] = TransferState();
    _transferStats[i] = TransferStats();
    _flowConfig[i] = FlowControlConfig();
    _channelConfig[i] = ChannelConfig();
    _lastError[i] = UARTError::NONE;
    _sequence[i] = 0;
    _rxHead[i] = 0;
    _rxTail[i] = 0;
  }
}
#else
SC16IS752_UART::SC16IS752_UART(uint8_t i2cAddr, TwoWire& wire)
  : SC16IS752(i2cAddr, wire) {
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    _transferState[i] = TransferState();
    _transferStats[i] = TransferStats();
    _flowConfig[i] = FlowControlConfig();
    _channelConfig[i] = ChannelConfig();
    _lastError[i] = UARTError::NONE;
    _sequence[i] = 0;
    _rxHead[i] = 0;
    _rxTail[i] = 0;
  }
}
#endif

SC16IS752_UART::SC16IS752_UART(SPIClass& spi, uint8_t csPin, const SPIConfig& config)
  : SC16IS752(spi, csPin, config) {
  // Initialize member arrays
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    _transferState[i] = TransferState();
    _transferStats[i] = TransferStats();
    _flowConfig[i] = FlowControlConfig();
    _lastError[i] = UARTError::NONE;
    _sequence[i] = 0;
    _rxHead[i] = 0;
    _rxTail[i] = 0;
  }
}

bool SC16IS752_UART::begin(uint32_t baudRate) {
  if (!SC16IS752::begin()) {
    return false;
  }

  // Initialize both channels
  return beginUART(CHANNEL_A, baudRate) && beginUART(CHANNEL_B, baudRate);
}

bool SC16IS752_UART::beginUART(uint8_t channel, uint32_t baudRate, UARTConfig config) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  // Store configuration
  _channelConfig[channel].baudRate = baudRate;
  _channelConfig[channel].config = config;
  _channelConfig[channel].enabled = false;  // Will be set to true if initialization succeeds

  // Step 1: Disable interrupts
  writeRegister(channel, REG_IER, 0x00);

  // Step 2: Disable FIFOs for initialization
  writeRegister(channel, REG_FCR, 0x00);
  delay(10);

  // Step 3: Set baud rate
  if (!setBaudRate(channel, baudRate)) {
    return false;
  }

  // Step 4: Set line configuration
  writeRegister(channel, REG_LCR, config);
  delay(10);

  // Step 5: Initialize FIFOs
  if (!initializeFIFO(channel)) {
    handleError(channel, UARTError::FIFO_ERROR);
    return false;
  }

  // Step 6: Verify with loopback test
  if (!performLoopbackTest(channel)) {
    handleError(channel, UARTError::INITIALIZATION_ERROR);
    return false;
  }

  // Clear any pending interrupts
  readRegister(channel, REG_IIR);
  readRegister(channel, REG_LSR);
  readRegister(channel, REG_MSR);

  // Initialization successful
  _channelConfig[channel].enabled = true;

  return true;
}

void SC16IS752_UART::end() {
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    // Disable interrupts
    writeRegister(i, REG_IER, 0x00);

    // Disable FIFOs
    writeRegister(i, REG_FCR, 0x00);

    // Reset flow control
    writeRegister(i, REG_MCR, 0x00);

    // Clear states
    _transferState[i] = TransferState();
    _transferStats[i] = TransferStats();
  }

  SC16IS752::end();
}

bool SC16IS752_UART::initializeFIFO(uint8_t channel) {
  // Reset FIFOs first
  writeRegister(channel, REG_FCR, 0x06);
  delay(1);

  // Enable FIFOs and set trigger levels
  uint8_t fcr = 0x01;  // Enable FIFO
  fcr |= (0x00 << 6);  // RX trigger level = 8 bytes
  fcr |= (0x00 << 4);  // TX trigger level = 8 spaces

  writeRegister(channel, REG_FCR, fcr);
  delay(1);

  // Verify FIFO enabled
  return verifyFIFOStatus(channel);
}

bool SC16IS752_UART::performLoopbackTest(uint8_t channel) {
  // Enable loopback mode
  writeRegister(channel, REG_MCR, 0x10);
  delay(1);

  // Send test byte
  const uint8_t testByte = 0x55;
  writeRegister(channel, REG_THR, testByte);

  // Wait for transmission
  if (!waitForTxEmpty(channel, 100)) {
    writeRegister(channel, REG_MCR, 0x00);  // Disable loopback
    return false;
  }

  // Wait for reception
  if (!waitForRxData(channel, 100)) {
    writeRegister(channel, REG_MCR, 0x00);  // Disable loopback
    return false;
  }

  // Read and verify
  uint8_t received = readRegister(channel, REG_RHR);

  // Disable loopback mode
  writeRegister(channel, REG_MCR, 0x00);

  return received == testByte;
}

void SC16IS752_UART::clearFIFOs(uint8_t channel) {
  uint8_t fcr = readRegister(channel, REG_FCR);
  writeRegister(channel, REG_FCR, fcr | 0x06);  // Reset TX and RX FIFOs
  delay(1);
  writeRegister(channel, REG_FCR, fcr);  // Restore FCR
}

bool SC16IS752_UART::verifyFIFOStatus(uint8_t channel) {
  uint8_t iir = readRegister(channel, REG_IIR);
  return (iir & 0xC0) == 0xC0;  // Check if FIFOs are enabled and working
}

// Basic I/O Methods
size_t SC16IS752_UART::write(uint8_t channel, uint8_t data) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return 0;
  }

  if (!waitForTxEmpty(channel, _flowConfig[channel].timeout)) {
    return 0;
  }

  writeRegister(channel, REG_THR, data);
  _transferStats[channel].bytesSent++;
  return 1;
}

size_t SC16IS752_UART::write(uint8_t channel, const uint8_t* buffer, size_t size) {
  if (!isValidChannel(channel) || !buffer || size == 0) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return 0;
  }

  size_t written = 0;
  uint32_t startTime = millis();

  while (written < size) {
    if (millis() - startTime > _flowConfig[channel].timeout) {
      handleError(channel, UARTError::TIMEOUT_ERROR);
      break;
    }

    if (!handleFlowControl(channel)) {
      continue;
    }

    uint8_t txlvl = readRegister(channel, REG_TXLVL);
    if (txlvl == 0) continue;

    size_t toWrite = min((size_t)txlvl, size - written);
    for (size_t i = 0; i < toWrite; i++) {
      writeRegister(channel, REG_THR, buffer[written++]);
    }
  }

  _transferStats[channel].bytesSent += written;
  return written;
}

int SC16IS752_UART::read(uint8_t channel) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return -1;
  }

  if (!available(channel)) {
    return -1;
  }

  uint8_t lsr = readRegister(channel, REG_LSR);
  if (lsr & 0x1E) {  // Check for errors
    handleError(channel, UARTError::PARITY_ERROR);
  }

  uint8_t data = readRegister(channel, REG_RHR);
  _transferStats[channel].bytesReceived++;
  return data;
}

size_t SC16IS752_UART::read(uint8_t channel, uint8_t* buffer, size_t size) {
  if (!buffer || !size) return 0;

  size_t bytesRead = 0;
  while (bytesRead < size) {
    int data = read(channel);
    if (data == -1) break;
    buffer[bytesRead++] = data;
  }
  return bytesRead;
}

int SC16IS752_UART::available(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return 0;
  }
  return readRegister(channel, REG_RXLVL);
}

void SC16IS752_UART::flush(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return;
  }

  // Wait for TX FIFO empty and shift register empty
  uint32_t startTime = millis();
  while (millis() - startTime < _flowConfig[channel].timeout) {
    uint8_t lsr = readRegister(channel, REG_LSR);
    if (lsr & 0x60) break;  // Both THR and TSR are empty
    delay(1);
  }
}

// Packet Handling Methods
bool SC16IS752_UART::sendPacket(uint8_t channel, const uint8_t* data,
                                size_t size, PacketType type) {
  if (!data || size > MAX_PACKET_SIZE - PACKET_HEADER_SIZE - 2) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  uint8_t packetBuffer[MAX_PACKET_SIZE];
  if (!buildPacket(packetBuffer, data, size, type)) {
    return false;
  }

  if (!beginTransfer(channel, size + PACKET_HEADER_SIZE + 2)) {
    return false;
  }

  size_t totalSize = size + PACKET_HEADER_SIZE + 2;  // Include CRC
  size_t written = 0;
  uint32_t startTime = millis();

  while (written < totalSize) {
    if (millis() - startTime > _flowConfig[channel].timeout) {
      endTransfer(channel, false);
      handleError(channel, UARTError::TIMEOUT_ERROR);
      return false;
    }

    if (!handleFlowControl(channel)) {
      delay(_flowConfig[channel].recoveryDelay);
      continue;
    }

    uint8_t txlvl = readRegister(channel, REG_TXLVL);
    if (txlvl == 0) continue;

    size_t toWrite = min((size_t)txlvl, totalSize - written);
    size_t currentWritten = write(channel,
                                  packetBuffer + written,
                                  toWrite);

    if (currentWritten > 0) {
      written += currentWritten;
      _transferState[channel].bytesTransferred += currentWritten;
      _transferState[channel].lastProgressTime = millis();
    }
  }

  endTransfer(channel, true);
  return true;
}

// Flow Control Methods
bool SC16IS752_UART::handleFlowControl(uint8_t channel) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  UARTStatus status = getStatus(channel);
  bool flowActive = false;

  // Check hardware flow control
  if (_flowConfig[channel].mode & FLOW_CTS) {
    flowActive = !(status.msr & 0x10);  // CTS is not active
  }

  // Handle flow control state
  if (flowActive) {
    if (!_transferState[channel].flowControlled) {
      _transferState[channel].flowControlled = true;
      _transferStats[channel].flowControlEvents++;
      _transferState[channel].lastProgressTime = millis();
    }

    // Check for recovery needed
    if (_transferState[channel].retryCount++ > _flowConfig[channel].maxRetries) {
      handleFlowControlRecovery(channel);
      return false;
    }

    delay(_flowConfig[channel].recoveryDelay);
    return false;
  }

  _transferState[channel].flowControlled = false;
  _transferState[channel].retryCount = 0;
  return true;
}

bool SC16IS752_UART::waitForTxReady(uint8_t channel, uint32_t timeout) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout) {
    if (!handleFlowControl(channel)) {
      delay(_flowConfig[channel].recoveryDelay);
      continue;
    }

    if (isTxEmpty(channel)) {
      return true;
    }
    delay(1);
  }

  handleError(channel, UARTError::TIMEOUT_ERROR);
  return false;
}

// Configuration Methods
bool SC16IS752_UART::setFlowControlConfig(uint8_t channel, const FlowControlConfig& config) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  _flowConfig[channel] = config;

  // Configure hardware flow control registers if needed
  uint8_t mcr = readRegister(channel, REG_MCR);
  uint8_t efr = readRegister(channel, REG_EFR);

  if (config.mode & FLOW_RTSCTS) {
    mcr |= 0x02;  // Enable RTS
    efr |= 0xC0;  // Enable RTS/CTS flow control
  } else {
    mcr &= ~0x02;  // Disable RTS
    efr &= ~0xC0;  // Disable RTS/CTS flow control
  }

  writeRegister(channel, REG_MCR, mcr);
  writeRegister(channel, REG_EFR, efr);

  return true;
}

// Helper Methods
void SC16IS752_UART::handleError(uint8_t channel, UARTError error) {
  if (!isValidChannel(channel)) {
    return;
  }

  _lastError[channel] = error;
  _transferStats[channel].errors++;

  if (error == UARTError::BUFFER_OVERFLOW) {
    _transferStats[channel].bufferOverruns++;
  }
}

bool SC16IS752_UART::waitForTxEmpty(uint8_t channel, uint32_t timeout) {
  uint32_t startTime = millis();

  while (millis() - startTime < timeout) {
    uint8_t lsr = readRegister(channel, REG_LSR);
    if (lsr & 0x20) {  // THR is empty
      return true;
    }
    delay(1);
  }

  handleError(channel, UARTError::TIMEOUT_ERROR);
  return false;
}

bool SC16IS752_UART::waitForRxData(uint8_t channel, uint32_t timeout) {
  uint32_t startTime = millis();

  while (millis() - startTime < timeout) {
    if (available(channel)) {
      return true;
    }
    delay(1);
  }

  handleError(channel, UARTError::TIMEOUT_ERROR);
  return false;
}

bool SC16IS752_UART::checkBufferSpace(uint8_t channel, size_t required) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  UARTStatus status = getStatus(channel);
  return (FIFO_SIZE - status.txlvl) >= required;
}

void SC16IS752_UART::monitorFIFOLevels(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return;
  }

  UARTStatus status = getStatus(channel);

  // Check for potential overflow
  if (status.rxlvl > (FIFO_SIZE - 8)) {  // Near full
    _transferStats[channel].bufferOverruns++;
  }

  // Check for underrun
  if (status.txlvl == 0 && _transferState[channel].inProgress) {
    _transferStats[channel].bufferUnderruns++;
  }
}

// Status Methods
bool SC16IS752_UART::isTxEmpty(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return true;
  }
  return (readRegister(channel, REG_LSR) & 0x20) != 0;
}

bool SC16IS752_UART::isRxFull(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return false;
  }
  return readRegister(channel, REG_RXLVL) >= FIFO_SIZE;
}

bool SC16IS752_UART::isTransferInProgress(uint8_t channel) const {
  if (!isValidChannel(channel)) {
    return false;
  }
  return _transferState[channel].inProgress;
}

SC16IS752_UART::UARTError SC16IS752_UART::getLastUARTError(uint8_t channel) const {
  if (!isValidChannel(channel)) {
    return UARTError::INVALID_CONFIG;
  }
  return _lastError[channel];
}

SC16IS752_UART::TransferStats SC16IS752_UART::getTransferStats(uint8_t channel) const {
  if (!isValidChannel(channel)) {
    return TransferStats();
  }
  return _transferStats[channel];
}

bool SC16IS752_UART::resetTransferStats(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return false;
  }

  _transferStats[channel] = TransferStats();
  return true;
}

// Add these to SC16IS752_UART.cpp

bool SC16IS752_UART::setFlowControl(uint8_t channel, bool enabled) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  uint8_t mcr = readRegister(channel, REG_MCR);
  uint8_t efr = readRegister(channel, REG_EFR);

  if (enabled) {
    mcr |= 0x02;  // Enable RTS
    efr |= 0xC0;  // Enable RTS/CTS flow control
  } else {
    mcr &= ~0x02;  // Disable RTS
    efr &= ~0xC0;  // Disable RTS/CTS flow control
  }

  writeRegister(channel, REG_MCR, mcr);
  writeRegister(channel, REG_EFR, efr);
  return true;
}

bool SC16IS752_UART::setFIFOTriggerLevels(uint8_t channel, uint8_t rxLevel, uint8_t txLevel) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  // Calculate FCR value with trigger levels
  uint8_t fcr = 0x01;              // Enable FIFO
  fcr |= ((rxLevel & 0x03) << 6);  // RX trigger level
  fcr |= ((txLevel & 0x03) << 4);  // TX trigger level

  writeRegister(channel, REG_FCR, fcr);
  return true;
}

bool SC16IS752_UART::validatePacket(const PacketHeader* header, size_t size) {
  if (!header) {
    return false;
  }

  if (header->startMarker != PACKET_START_MARKER) {
    return false;
  }

  if (header->size > MAX_PACKET_SIZE || header->size < PACKET_HEADER_SIZE + 2) {
    return false;
  }

  if (size < PACKET_HEADER_SIZE) {
    return false;
  }

  return true;
}

uint16_t SC16IS752_UART::calculateCRC16(const uint8_t* data, size_t length) {
  if (!data || length == 0) {
    return 0;
  }

  uint16_t crc = CRC16_INIT;

  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ CRC16_POLYNOMIAL) : (crc << 1);
    }
  }

  return crc;
}

void SC16IS752_UART::updateTransferStats(uint8_t channel, bool success,
                                         uint32_t bytesTransferred) {
  if (!isValidChannel(channel)) {
    return;
  }

  TransferStats& stats = _transferStats[channel];
  uint32_t transferTime = millis() - _transferState[channel].startTime;

  if (success) {
    stats.successfulTransfers++;
    stats.bytesReceived += bytesTransferred;

    if (stats.averageTransferTime == 0) {
      stats.averageTransferTime = transferTime;
    } else {
      stats.averageTransferTime = (stats.averageTransferTime * 0.9) + (transferTime * 0.1);
    }

    float currentRate = (float)bytesTransferred * 1000 / transferTime;
    if (currentRate > stats.peakTransferRate) {
      stats.peakTransferRate = currentRate;
    }
  } else {
    stats.failedTransfers++;
    stats.errors++;
  }

  stats.lastTransferTime = millis();
}

bool SC16IS752_UART::buildPacket(uint8_t* buffer, const uint8_t* data, size_t size,
                                 PacketType type) {
  if (!buffer || !data) {
    return false;
  }

  PacketHeader* header = (PacketHeader*)buffer;
  header->startMarker = PACKET_START_MARKER;
  header->size = size + 2;  // Add CRC size
  header->type = type;
  header->sequence = _sequence[0]++;  // Increment sequence number

  // Copy data
  memcpy(buffer + PACKET_HEADER_SIZE, data, size);

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(data, size);
  buffer[PACKET_HEADER_SIZE + size] = (crc >> 8) & 0xFF;
  buffer[PACKET_HEADER_SIZE + size + 1] = crc & 0xFF;

  return true;
}

bool SC16IS752_UART::beginTransfer(uint8_t channel, size_t size) {
  if (!isValidChannel(channel) || size == 0) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  if (_transferState[channel].inProgress) {
    handleError(channel, UARTError::INVALID_OPERATION);
    return false;
  }

  _transferState[channel].inProgress = true;
  _transferState[channel].startTime = millis();
  _transferState[channel].bytesTransferred = 0;
  _transferState[channel].currentCRC = CRC16_INIT;
  _transferState[channel].flowControlled = false;
  _transferState[channel].lastProgressTime = millis();
  _transferState[channel].retryCount = 0;

  return true;
}

void SC16IS752_UART::endTransfer(uint8_t channel, bool success) {
  if (!isValidChannel(channel)) {
    return;
  }

  if (_transferState[channel].inProgress) {
    updateTransferStats(channel, success, _transferState[channel].bytesTransferred);
  }

  _transferState[channel].inProgress = false;
}

bool SC16IS752_UART::setCrystalConfig(const SC16IS752::CrystalConfig& config) {
  _crystalConfig = config;

  // Apply new crystal settings
  bool success = true;
  for (uint8_t channel = 0; channel < MAX_CHANNELS; channel++) {
    if (_channelConfig[channel].enabled) {
      success &= setBaudRate(channel, _channelConfig[channel].baudRate);
    }
  }
  return success;
}

bool SC16IS752_UART::setBaudRate(uint8_t channel, uint32_t baudRate) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  // Calculate divisor based on crystal frequency and prescaler
  uint32_t prescaler = _crystalConfig.usePrescaler ? _crystalConfig.prescalerValue : 1;
  uint32_t div = (_crystalConfig.frequency / prescaler) / (baudRate * 16);

  if (div == 0 || div > 0xFFFF) {
    handleError(channel, UARTError::INVALID_BAUD_RATE);
    return false;
  }

  // Enable access to divisor latches
  uint8_t lcr = readRegister(channel, REG_LCR);
  writeRegister(channel, REG_LCR, lcr | 0x80);

  // Write divisor
  writeRegister(channel, REG_DLL, div & 0xFF);
  writeRegister(channel, REG_DLH, (div >> 8) & 0xFF);

  // Restore LCR
  writeRegister(channel, REG_LCR, lcr);

  _channelConfig[channel].baudRate = baudRate;
  return true;
}

// Add to SC16IS752_UART.cpp

SC16IS752_UART::UARTStatus SC16IS752_UART::getStatus(uint8_t channel) {
  UARTStatus status = { 0 };

  if (!isValidChannel(channel)) {
    status.error = UARTError::INVALID_CONFIG;
    return status;
  }

  // Get UART status registers
  status.lsr = readRegister(channel, REG_LSR);      // Line Status Register
  status.msr = readRegister(channel, REG_MSR);      // Modem Status Register
  status.txlvl = readRegister(channel, REG_TXLVL);  // TX FIFO Level
  status.rxlvl = readRegister(channel, REG_RXLVL);  // RX FIFO Level

  // Check for errors in LSR
  if (status.lsr & 0x1E) {  // Bits 1-4 indicate various errors
    if (status.lsr & 0x02) status.error = UARTError::OVERRUN_ERROR;
    else if (status.lsr & 0x04) status.error = UARTError::PARITY_ERROR;
    else if (status.lsr & 0x08) status.error = UARTError::FRAMING_ERROR;
    else if (status.lsr & 0x10) status.error = UARTError::BREAK_CONDITION;
  } else {
    status.error = _lastError[channel];
  }

  return status;
}

// Update receivePacket method in SC16IS752_UART.cpp:
bool SC16IS752_UART::receivePacket(uint8_t channel, uint8_t* buffer, size_t& size) {
  if (!buffer) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  // Clear any stale data
  while (available(channel) && readRegister(channel, REG_RXLVL) > 64) {
    read(channel);
  }

  uint8_t tempBuffer[MAX_PACKET_SIZE];
  size_t bufferPos = 0;
  uint32_t startTime = millis();
  bool headerFound = false;
  size_t expectedSize = 0;

  while (bufferPos < MAX_PACKET_SIZE) {
    if (millis() - startTime > _flowConfig[channel].timeout) {
      handleError(channel, UARTError::TIMEOUT_ERROR);
      return false;
    }

    if (!available(channel)) {
      delay(1);
      continue;
    }

    int data = read(channel);
    if (data == -1) continue;

    tempBuffer[bufferPos++] = data;

    // Look for packet header
    if (!headerFound && bufferPos >= PACKET_HEADER_SIZE) {
      PacketHeader* header = (PacketHeader*)tempBuffer;
      if (header->startMarker == PACKET_START_MARKER) {
        headerFound = true;
        expectedSize = header->size + PACKET_HEADER_SIZE;
        if (expectedSize > MAX_PACKET_SIZE) {
          handleError(channel, UARTError::BUFFER_OVERFLOW);
          return false;
        }
      } else {
        // Reset if header not found
        bufferPos = 0;
        continue;
      }
    }

    // Process complete packet
    if (headerFound && bufferPos >= expectedSize) {
      PacketHeader* header = (PacketHeader*)tempBuffer;
      size_t payloadSize = header->size - 2;  // Subtract CRC bytes

      // Validate CRC
      uint16_t receivedCrc = (tempBuffer[bufferPos - 2] << 8) | tempBuffer[bufferPos - 1];
      uint16_t calculatedCrc = calculateCRC16(tempBuffer + PACKET_HEADER_SIZE, payloadSize);

      if (receivedCrc != calculatedCrc) {
        handleError(channel, UARTError::CRC_ERROR);
        _transferStats[channel].crcErrors++;
        return false;
      }

      // Copy payload to output buffer
      size = payloadSize;
      memcpy(buffer, tempBuffer + PACKET_HEADER_SIZE, payloadSize);
      updateTransferStats(channel, true, payloadSize);

      // Send acknowledgment
      uint8_t ack = 0xAA;
      write(channel, &ack, 1);

      return true;
    }
  }

  handleError(channel, UARTError::BUFFER_OVERFLOW);
  return false;
}

// Add to handleFlowControl method:
void SC16IS752_UART::handleFlowControlRecovery(uint8_t channel) {
  // Clear FIFOs
  clearFIFOs(channel);

  // Reset flow control state
  _transferState[channel].flowControlled = false;
  _transferState[channel].retryCount = 0;

  // Wait for recovery
  delay(_flowConfig[channel].recoveryDelay);

  // Re-initialize FIFO if needed
  if (!verifyFIFOStatus(channel)) {
    initializeFIFO(channel);
  }
}

// Update enableRS485 method:
bool SC16IS752_UART::enableRS485(uint8_t channel, bool enabled, bool rtsInverted) {
  if (!isValidChannel(channel)) {
    handleError(channel, UARTError::INVALID_CONFIG);
    return false;
  }

  // Clear any pending data first
  clearFIFOs(channel);

  uint8_t efcr = readRegister(channel, REG_EFCR);

  if (enabled) {
    efcr |= 0x20;  // Enable RS-485 mode
    if (rtsInverted) {
      efcr |= 0x40;  // Invert RTS
    } else {
      efcr &= ~0x40;
    }
    efcr |= 0x10;  // Enable auto-direction control

    // Configure for RS-485 timing
    uint8_t mcr = readRegister(channel, REG_MCR);
    mcr |= 0x02;  // Enable RTS
    writeRegister(channel, REG_MCR, mcr);

    delay(10);  // Allow mode change to settle
  } else {
    efcr &= ~0x70;  // Disable RS-485 features
  }

  writeRegister(channel, REG_EFCR, efcr);
  return true;
}

// Add to SC16IS752_UART.cpp

bool SC16IS752_UART::configureRS485(uint8_t channel, uint32_t baudRate) {
  if (!isValidChannel(channel)) {
    return false;
  }

  // Configure UART
  if (!beginUART(channel, baudRate, UART_8N1)) {
    return false;
  }

  // Set up for RS-485 operation
  uint8_t efcr = readRegister(channel, REG_EFCR);
  efcr |= 0x20;  // Enable RS-485 mode
  efcr |= 0x10;  // Enable auto-direction control
  writeRegister(channel, REG_EFCR, efcr);

  // Configure RTS timing
  uint8_t mcr = readRegister(channel, REG_MCR);
  mcr |= 0x02;  // Enable RTS
  writeRegister(channel, REG_MCR, mcr);

  // Adjust timing for RS-485
  _flowConfig[channel].recoveryDelay = 2;  // 2ms for direction switching
  _flowConfig[channel].timeout = 100;      // 100ms timeout

  return true;
}

// Add helper method for RS-485 transmit
bool SC16IS752_UART::transmitRS485(uint8_t channel, const uint8_t* data, size_t size) {
  if (!data || size == 0) {
    return false;
  }

  // Wait for line to be clear
  delay(_flowConfig[channel].recoveryDelay);

  // Send data
  size_t written = write(channel, data, size);

  // Wait for transmission to complete
  flush(channel);
  delay(_flowConfig[channel].recoveryDelay);

  return written == size;
}

bool SC16IS752_UART::resetChannel(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return false;
  }

  // Reset channel using public methods
  while (available(channel)) {
    read(channel);
  }
  flush(channel);

  // Reset stats
  resetTransferStats(channel);
  return true;
}

/*
bool SC16IS752_UART::sendPacket(uint8_t channel, const uint8_t* data, size_t size, PacketType type) {
    uint8_t packetBuffer[MAX_PACKET_SIZE];
    PacketHeader* header = (PacketHeader*)packetBuffer;
    header->startMarker = PACKET_START_MARKER;
    header->size = size;
    header->type = type;
    header->sequence = _sequence[channel]++;

    memcpy(packetBuffer + PACKET_HEADER_SIZE, data, size);
    size_t totalSize = PACKET_HEADER_SIZE + size;

    return write(channel, packetBuffer, totalSize) == totalSize;
}
*/

/*
bool SC16IS752_UART::receivePacket(uint8_t channel, uint8_t* buffer, size_t& size) {
    uint8_t header[PACKET_HEADER_SIZE];
    if (read(channel, header, PACKET_HEADER_SIZE) != PACKET_HEADER_SIZE) {
        return false;
    }
    
    if (header[0] != PACKET_START_MARKER) {
        return false;
    }
    
    size = header[1];
    return read(channel, buffer, size) == size;
}
*/

/*
bool SC16IS752_UART::buildPacket(uint8_t* buffer, const uint8_t* data, size_t size,
                                PacketType type) {
    // Remove CRC calculation
    PacketHeader* header = (PacketHeader*)buffer;
    header->startMarker = PACKET_START_MARKER;
    header->size = size;  // No CRC to add
    header->type = type;
    header->sequence = _sequence[0]++;

    // Copy data only
    memcpy(buffer + PACKET_HEADER_SIZE, data, size);

    return true;
}
*/
