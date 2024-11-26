// SC16IS752.h
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

class SC16IS752 {
public:
  // Constants
  static const uint8_t CHANNEL_A = 0;
  static const uint8_t CHANNEL_B = 1;
  static const uint8_t MAX_CHANNELS = 2;

  // Configuration Structures
  struct I2CPins {
    int sda;
    int scl;
    I2CPins(int sda_pin = -1, int scl_pin = -1)
      : sda(sda_pin), scl(scl_pin) {}
  };

  struct SPIConfig {
    uint32_t clockSpeed;
    uint8_t bitOrder;
    uint8_t dataMode;
    SPIConfig(uint32_t speed = 4000000, uint8_t order = MSBFIRST,
              uint8_t mode = SPI_MODE0)
      : clockSpeed(speed), bitOrder(order), dataMode(mode) {}
  };

  struct CrystalConfig {
    bool useExternalClock;
    uint32_t frequency;
    bool usePrescaler;
    uint8_t prescalerValue;
    CrystalConfig(uint32_t freq = 14745600)
      : useExternalClock(false),
        frequency(freq),
        usePrescaler(false),
        prescalerValue(1) {}
  };

  enum Interface {
    I2C_MODE,
    SPI_MODE
  };

  enum Error {
    NONE = 0,
    COMMUNICATION_ERROR,
    INVALID_CHANNEL,
    INITIALIZATION_ERROR,
    TIMEOUT_ERROR
  };

  // Add to existing enums
  enum PowerMode {
    POWER_NORMAL,
    POWER_SLEEP,
    POWER_SHUTDOWN
  };

// Constructors
#if defined(ESP32) || defined(ESP8266)
  SC16IS752(uint8_t i2cAddr, TwoWire& wire = Wire, const I2CPins& pins = I2CPins());
#else
  SC16IS752(uint8_t i2cAddr, TwoWire& wire = Wire);
#endif
  SC16IS752(SPIClass& spi, uint8_t csPin, const SPIConfig& config = SPIConfig());

  // Initialization
  virtual bool begin(const CrystalConfig& config = CrystalConfig());
  virtual void end();
  bool beginSPI();
  bool setSPIConfig(const SPIConfig& config);
  Error getLastError() const {
    return _lastError;
  }

  // Add to public methods
  bool verifyClockConfig();
  bool verifyDeviceID();
  bool setPowerMode(PowerMode mode);
  bool optimizeBusTiming();

protected:
  // Register definitions
  static const uint8_t REG_RHR = 0x00;
  static const uint8_t REG_THR = 0x00;
  static const uint8_t REG_IER = 0x01;
  static const uint8_t REG_FCR = 0x02;
  static const uint8_t REG_IIR = 0x02;
  static const uint8_t REG_LCR = 0x03;
  static const uint8_t REG_MCR = 0x04;
  static const uint8_t REG_LSR = 0x05;
  static const uint8_t REG_MSR = 0x06;
  static const uint8_t REG_SPR = 0x07;
  static const uint8_t REG_TXLVL = 0x08;
  static const uint8_t REG_RXLVL = 0x09;
  static const uint8_t REG_IODIR = 0x0A;
  static const uint8_t REG_IOSTATE = 0x0B;
  static const uint8_t REG_IOCTRL = 0x0E;
  static const uint8_t REG_EFCR = 0x0F;
  static const uint8_t REG_DLL = 0x00;
  static const uint8_t REG_DLH = 0x01;
  static const uint8_t REG_EFR = 0x02;

  // Timeouts
  static const uint32_t TIMEOUT_MS = 1000;

  // Register access methods
  void writeRegister(uint8_t channel, uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t channel, uint8_t reg);
  uint8_t regAddr(uint8_t reg, uint8_t channel) const;

  // Communication methods
  void writeRegisterI2C(uint8_t channel, uint8_t reg, uint8_t value);
  uint8_t readRegisterI2C(uint8_t channel, uint8_t reg);
  void writeRegisterSPI(uint8_t channel, uint8_t reg, uint8_t value);
  uint8_t readRegisterSPI(uint8_t channel, uint8_t reg);

  // Helper methods
  bool ping();
  bool isValidChannel(uint8_t channel) const {
    return channel < MAX_CHANNELS;
  }
  void setError(Error error) {
    _lastError = error;
  }

  // Protected members
  Interface _interface;
  TwoWire* _wire;
  SPIClass* _spi;
  uint8_t _i2cAddr;
  uint8_t _csPin;
  bool _initialized;
  Error _lastError;
  CrystalConfig _crystalConfig;
  SPIConfig _spiConfig;

#if defined(ESP32) || defined(ESP8266)
  I2CPins _pins;
  bool initI2C();
#endif

  // Add to protected members
  PowerMode _powerMode;
  uint32_t _optimizedDelay;

  // Add helper methods
  bool performClockTest();
  void updateBusTiming();

private:
  void beginSPITransaction();
  void endSPITransaction();
  void initSPI();
};
