// SC16IS752_UART.h
#pragma once
#include "SC16IS752.h"

class SC16IS752_UART : public SC16IS752 {
public:
  // Hardware Constants
  static const uint16_t FIFO_SIZE = 64;
  static const uint8_t MAX_CHANNELS = 2;
  static const uint8_t PACKET_START_MARKER = 0xAA;
  static const uint8_t PACKET_HEADER_SIZE = 4;
  static const size_t MAX_PACKET_SIZE = 128;
  static const uint32_t DEFAULT_TIMEOUT = 1000;
  static const uint32_t DEFAULT_RECOVERY_DELAY = 5;
  static const uint8_t DEFAULT_MAX_RETRIES = 3;
  static const uint16_t CRC16_POLYNOMIAL = 0x1021;
  static const uint16_t CRC16_INIT = 0xFFFF;

  // UART Configuration Enums
  enum UARTConfig : uint8_t {
    UART_5N1 = 0x00,
    UART_6N1 = 0x01,
    UART_7N1 = 0x02,
    UART_8N1 = 0x03,
    UART_5N2 = 0x04,
    UART_6N2 = 0x05,
    UART_7N2 = 0x06,
    UART_8N2 = 0x07,
    UART_5E1 = 0x08,
    UART_6E1 = 0x09,
    UART_7E1 = 0x0A,
    UART_8E1 = 0x0B,
    UART_5O1 = 0x0C,
    UART_6O1 = 0x0D,
    UART_7O1 = 0x0E,
    UART_8O1 = 0x0F
  };

  enum UARTError {
    NONE = 0,
    INVALID_CONFIG,
    INVALID_BAUD_RATE,
    BUFFER_OVERFLOW,
    FIFO_ERROR,
    PARITY_ERROR,
    FRAMING_ERROR,
    BREAK_CONDITION,
    INVALID_OPERATION,
    INITIALIZATION_ERROR,
    CRC_ERROR,
    TIMEOUT_ERROR,
    FLOW_CONTROL_ERROR,
    OVERRUN_ERROR
  };

  enum PacketType {
    PACKET_SMALL = 0x01,
    PACKET_LARGE = 0x02,
    PACKET_CONTROL = 0x03
  };

  enum FlowControl : uint8_t {
    FLOW_NONE = 0x00,
    FLOW_RTS = 0x01,
    FLOW_CTS = 0x02,
    FLOW_RTSCTS = 0x03,
    FLOW_XON_XOFF = 0x04
  };

  // Configuration Structures
  struct PacketHeader {
    uint8_t startMarker;
    uint8_t size;
    uint8_t type;
    uint8_t sequence;
  };

  struct FlowControlConfig {
    uint32_t recoveryDelay;
    uint8_t maxRetries;
    uint32_t timeout;
    FlowControl mode;
    bool rtsInverted;

    FlowControlConfig()
      : recoveryDelay(DEFAULT_RECOVERY_DELAY),
        maxRetries(DEFAULT_MAX_RETRIES),
        timeout(DEFAULT_TIMEOUT),
        mode(FLOW_NONE),
        rtsInverted(false) {}
  };

  struct UARTStatus {
    uint8_t lsr;      // Line Status Register
    uint8_t msr;      // Modem Status Register
    uint8_t txlvl;    // TX FIFO Level
    uint8_t rxlvl;    // RX FIFO Level
    UARTError error;  // Last error
  };

  struct TransferStats {
    uint32_t bytesSent;
    uint32_t bytesReceived;
    uint32_t errors;
    uint32_t crcErrors;
    uint32_t flowControlEvents;
    uint32_t successfulTransfers;
    uint32_t failedTransfers;
    uint32_t bufferOverruns;
    uint32_t bufferUnderruns;
    float averageTransferTime;
    float peakTransferRate;
    uint32_t lastTransferTime;

    TransferStats()
      : bytesSent(0), bytesReceived(0), errors(0),
        crcErrors(0), flowControlEvents(0),
        successfulTransfers(0), failedTransfers(0),
        bufferOverruns(0), bufferUnderruns(0),
        averageTransferTime(0), peakTransferRate(0),
        lastTransferTime(0) {}
  };

// Constructors
#if defined(ESP32) || defined(ESP8266)
  SC16IS752_UART(uint8_t i2cAddr, TwoWire& wire = Wire,
                 const I2CPins& pins = I2CPins());
#else
  SC16IS752_UART(uint8_t i2cAddr, TwoWire& wire = Wire);
#endif

  SC16IS752_UART(SPIClass& spi, uint8_t csPin,
                 const SPIConfig& config = SPIConfig());

  // Initialization Methods
  bool begin(uint32_t baudRate);
  bool beginUART(uint8_t channel, uint32_t baudRate,
                 UARTConfig config = UART_8N1);
  void end();

  // Basic I/O Methods
  size_t write(uint8_t channel, uint8_t data);
  size_t write(uint8_t channel, const uint8_t* buffer, size_t size);
  int read(uint8_t channel);
  size_t read(uint8_t channel, uint8_t* buffer, size_t size);
  int available(uint8_t channel);
  void flush(uint8_t channel);

  // Configuration Methods
  bool setBaudRate(uint8_t channel, uint32_t baudRate);
  bool setConfig(uint8_t channel, UARTConfig config);
  bool setFlowControl(uint8_t channel, bool enabled);
  bool setFlowControlConfig(uint8_t channel, const FlowControlConfig& config);
  bool setFIFOTriggerLevels(uint8_t channel, uint8_t rxLevel, uint8_t txLevel);
  bool setCrystalConfig(const CrystalConfig& config);
  bool configureRS485(uint8_t channel, uint32_t baudRate);

  // Packet Handling Methods
  bool sendPacket(uint8_t channel, const uint8_t* data, size_t size,
                  PacketType type);
  bool receivePacket(uint8_t channel, uint8_t* buffer, size_t& size);
  bool transmitRS485(uint8_t channel, const uint8_t* data, size_t size);

  // Status and Control Methods
  UARTStatus getStatus(uint8_t channel);
  TransferStats getTransferStats(uint8_t channel) const;
  UARTError getLastUARTError(uint8_t channel) const;
  bool isTransferInProgress(uint8_t channel) const;
  bool isTxEmpty(uint8_t channel);
  bool isRxFull(uint8_t channel);
  bool resetTransferStats(uint8_t channel);
  bool resetChannel(uint8_t channel);

  // Flow Control Methods
  bool handleFlowControl(uint8_t channel);
  bool waitForTxReady(uint8_t channel, uint32_t timeout = DEFAULT_TIMEOUT);
  bool enableRS485(uint8_t channel, bool enabled, bool rtsInverted = false);

protected:
  // Protected Structures
  struct TransferState {
    bool inProgress;
    uint32_t startTime;
    size_t bytesTransferred;
    uint16_t currentCRC;
    bool flowControlled;
    uint32_t lastProgressTime;
    uint8_t retryCount;
    uint8_t lastSequence;

    TransferState()
      : inProgress(false), startTime(0), bytesTransferred(0),
        currentCRC(CRC16_INIT), flowControlled(false),
        lastProgressTime(0), retryCount(0), lastSequence(0) {}
  };

  // Protected Helper Methods
  bool initializeFIFO(uint8_t channel);
  bool performLoopbackTest(uint8_t channel);
  void handleError(uint8_t channel, UARTError error);
  bool waitForTxEmpty(uint8_t channel, uint32_t timeout = DEFAULT_TIMEOUT);
  bool waitForRxData(uint8_t channel, uint32_t timeout = DEFAULT_TIMEOUT);
  void handleFlowControlRecovery(uint8_t channel);
  bool checkBufferSpace(uint8_t channel, size_t required);
  void monitorFIFOLevels(uint8_t channel);
  uint16_t calculateCRC16(const uint8_t* data, size_t length);
  bool validatePacket(const PacketHeader* header, size_t size);
  bool buildPacket(uint8_t* buffer, const uint8_t* data, size_t size,
                   PacketType type);
  bool beginTransfer(uint8_t channel, size_t size);
  void endTransfer(uint8_t channel, bool success);
  void updateTransferStats(uint8_t channel, bool success,
                           uint32_t bytesTransferred);
  void clearFIFOs(uint8_t channel);
  bool verifyFIFOStatus(uint8_t channel);

private:
  // Member Variables
  struct ChannelConfig {
    uint32_t baudRate;
    bool enabled;
    UARTConfig config;
    uint8_t rxTriggerLevel;
    uint8_t txTriggerLevel;

    ChannelConfig()
      : baudRate(0),
        enabled(false),
        config(UART_8N1),
        rxTriggerLevel(0),
        txTriggerLevel(0) {}
  };

  ChannelConfig _channelConfig[MAX_CHANNELS];
  TransferState _transferState[MAX_CHANNELS];
  TransferStats _transferStats[MAX_CHANNELS];
  FlowControlConfig _flowConfig[MAX_CHANNELS];
  UARTError _lastError[MAX_CHANNELS];
  uint8_t _sequence[MAX_CHANNELS];

  // Ring buffer for packet reception
  static const size_t RX_BUFFER_SIZE = 256;
  uint8_t _rxBuffer[MAX_CHANNELS][RX_BUFFER_SIZE];
  size_t _rxHead[MAX_CHANNELS];
  size_t _rxTail[MAX_CHANNELS];
};
