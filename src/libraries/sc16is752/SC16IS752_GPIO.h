// SC16IS752_GPIO.h
#pragma once
#include "SC16IS752.h"

class SC16IS752_GPIO : public SC16IS752 {
public:
  enum GPIOMode {
    GPIO_INPUT = 0,
    GPIO_OUTPUT = 1,
    GPIO_INPUT_PULLUP = 2
  };

  enum InterruptMode {
    INTERRUPT_DISABLED = 0,
    INTERRUPT_RISING = 1,
    INTERRUPT_FALLING = 2,
    INTERRUPT_BOTH = 3
  };

  enum GPIOError {
    NONE = 0,
    INVALID_PIN,
    INVALID_MODE,
    INVALID_OPERATION,
    INITIALIZATION_ERROR
  };

  struct PinConfig {
    GPIOMode mode;
    InterruptMode interruptMode;
    bool initialState;
    uint32_t debounceTime;
  };

  struct GPIOStats {
    uint32_t interruptCount;
    uint32_t toggleCount;
    uint32_t lastInterruptTime;
    uint32_t lastChangeTime;
    uint32_t bounceEvents;
    uint8_t currentState;
    bool isStable;
  };

  struct InterruptEvent {
    uint8_t pin;
    bool rising;
    uint32_t timestamp;
  };

  static const uint8_t MAX_QUEUE_SIZE = 16;

// Constructors
#if defined(ESP32) || defined(ESP8266)
  SC16IS752_GPIO(uint8_t i2cAddr, TwoWire& wire = Wire, const I2CPins& pins = I2CPins());
#else
  SC16IS752_GPIO(uint8_t i2cAddr, TwoWire& wire = Wire);
#endif
  SC16IS752_GPIO(SPIClass& spi, uint8_t csPin, const SPIConfig& config = SPIConfig());

  // GPIO Operations
  bool beginGPIO();
  void endGPIO();
  bool configurePin(uint8_t pin, const PinConfig& config);
  bool setPinMode(uint8_t pin, GPIOMode mode);
  bool digitalWrite(uint8_t pin, uint8_t value);
  int digitalRead(uint8_t pin);
  bool togglePin(uint8_t pin);

  // Port Operations
  bool writePort(uint8_t value);
  uint8_t readPort();
  bool setPortDirection(uint8_t direction);
  uint8_t getPortDirection();

  // Interrupt Control
  bool enableInterrupt(uint8_t pin, InterruptMode mode);
  bool disableInterrupt(uint8_t pin);
  uint8_t getInterruptStatus();
  void clearInterrupt(uint8_t pin);
  bool setDebounceTime(uint8_t pin, uint32_t debounceTime);

  // Status and Diagnostics
  GPIOStats getGPIOStats(uint8_t pin);
  GPIOError getLastGPIOError() const {
    return _lastError;
  }
  bool isPinStable(uint8_t pin);
  uint32_t getLastChangeTime(uint8_t pin);

  // Add to public methods
  bool getNextInterrupt(InterruptEvent& event);
  bool saveState();
  bool restoreState();
  void clearInterruptQueue();

protected:
  static const uint8_t MAX_GPIO_PINS = 8;
  static const uint32_t DEBOUNCE_DEFAULT_MS = 50;
  static const uint32_t STABILITY_THRESHOLD = 100;

  struct PinState {
    PinConfig config;
    uint8_t lastState;
    uint32_t lastDebounceTime;
    bool interruptPending;
  };

  void updateGPIOStats(uint8_t pin, bool isInterrupt, bool risingEdge);
  bool validatePin(uint8_t pin) {
    return pin < MAX_GPIO_PINS;
  }
  bool validatePinMode(uint8_t pin, GPIOMode mode);
  void handleError(GPIOError error) {
    _lastError = error;
  }

private:
  GPIOStats _gpioStats[MAX_GPIO_PINS];
  PinState _pinStates[MAX_GPIO_PINS];
  GPIOError _lastError;
  bool _gpioInitialized;

  void initializeGPIOArrays();
  void configureInterrupt(uint8_t pin, InterruptMode mode);
  bool handleInterrupt(uint8_t pin);
  void checkPinStability(uint8_t pin);

  InterruptEvent _interruptQueue[MAX_QUEUE_SIZE];
  uint8_t _queueHead;
  uint8_t _queueTail;
  uint8_t _savedState[MAX_GPIO_PINS];

  void queueInterrupt(uint8_t pin, bool rising);
};
