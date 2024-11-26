# SC16IS752 Class Documentation

## Class Hierarchy and Design

```
SC16IS752 (Base)
├── SC16IS752_GPIO  
└── SC16IS752_UART
```

## Base Class (SC16IS752)

### Purpose
Provides fundamental I2C/SPI communication and register access for the SC16IS752 chip.

### Public Interface

#### Constructors
```cpp
// I2C Constructor
SC16IS752(uint8_t i2cAddr, TwoWire& wire, const I2CPins& pins);

// SPI Constructor
SC16IS752(SPIClass& spi, uint8_t csPin, const SPIConfig& config);
```

#### Configuration Structures
```cpp
struct I2CPins {
    int sda;
    int scl;
};

struct SPIConfig {
    uint32_t clockSpeed;    // Default: 4MHz
    uint8_t bitOrder;       // Default: MSBFIRST
    uint8_t dataMode;       // Default: SPI_MODE0
};

struct CrystalConfig {
    bool useExternalClock;
    uint32_t frequency;     // Default: 14.7456 MHz
    bool usePrescaler;
    uint8_t prescalerValue;
};
```

#### Initialization Methods
```cpp
bool begin(const CrystalConfig& config = CrystalConfig());
virtual void end();
bool setSPIConfig(const SPIConfig& config);
Error getLastError() const;
```

## UART Class (SC16IS752_UART)

### Purpose
Implements UART functionality with packet handling, flow control, and diagnostics.

### Public Interface

#### UART Configuration
```cpp
bool begin(uint32_t baudRate);
bool beginUART(uint8_t channel, uint32_t baudRate, UARTConfig config = UART_8N1);
bool setFlowControl(uint8_t channel, bool enabled);
bool setFlowControlConfig(uint8_t channel, const FlowControlConfig& config);
```

#### Data Transfer
```cpp
// Basic I/O
size_t write(uint8_t channel, uint8_t data);
size_t write(uint8_t channel, const uint8_t* buffer, size_t size);
int read(uint8_t channel);
size_t read(uint8_t channel, uint8_t* buffer, size_t size);
int available(uint8_t channel);
void flush(uint8_t channel);

// Packet Transfer
bool sendPacket(uint8_t channel, const uint8_t* data, size_t size, PacketType type);
bool receivePacket(uint8_t channel, uint8_t* buffer, size_t& size);
```

#### Status and Diagnostics
```cpp
UARTStatus getStatus(uint8_t channel);
TransferStats getTransferStats(uint8_t channel) const;
UARTError getLastUARTError(uint8_t channel) const;
bool resetChannel(uint8_t channel);
bool resetTransferStats(uint8_t channel);
```

#### Key Structures
```cpp
struct FlowControlConfig {
    uint32_t recoveryDelay;   // Default: 5ms
    uint8_t maxRetries;       // Default: 3
    uint32_t timeout;         // Default: 1000ms
    FlowControl mode;         // NONE, RTS, CTS, RTSCTS
    bool rtsInverted;         // Default: false
};

struct TransferStats {
    uint32_t bytesSent;
    uint32_t bytesReceived;
    uint32_t errors;
    uint32_t crcErrors;
    uint32_t flowControlEvents;
    float averageTransferTime;
    float peakTransferRate;
};
```

## GPIO Class (SC16IS752_GPIO)

### Purpose
Implements GPIO functionality with interrupt support and pin configuration.

### Public Interface

#### Initialization
```cpp
bool beginGPIO();
void endGPIO();
```

#### Pin Control
```cpp
bool configurePin(uint8_t pin, const PinConfig& config);
bool setPinMode(uint8_t pin, GPIOMode mode);
bool digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
bool togglePin(uint8_t pin);
```

#### Port Operations
```cpp
bool writePort(uint8_t value);
uint8_t readPort();
bool setPortDirection(uint8_t direction);
uint8_t getPortDirection();
```

#### Interrupt Handling
```cpp
bool enableInterrupt(uint8_t pin, InterruptMode mode);
bool disableInterrupt(uint8_t pin);
uint8_t getInterruptStatus();
void clearInterrupt(uint8_t pin);
```

#### Key Structures
```cpp
struct PinConfig {
    GPIOMode mode;           // INPUT, OUTPUT, INPUT_PULLUP
    InterruptMode intMode;   // DISABLED, RISING, FALLING, BOTH
    bool initialState;
    uint32_t debounceTime;   // Default: 50ms
};

struct GPIOStats {
    uint32_t interruptCount;
    uint32_t toggleCount;
    uint32_t bounceEvents;
    bool isStable;
};
```

## Usage Examples

### Basic UART Communication
```cpp
SC16IS752_UART uart(I2C_ADDRESS);
uart.begin(115200);

// Write data
uart.write(CHANNEL_A, "Hello", 5);

// Read data
while (uart.available(CHANNEL_A)) {
    int data = uart.read(CHANNEL_A);
}
```

### Packet Transfer with CRC
```cpp
const uint8_t data[] = {0x1A, 0x2B, 0x3C};
uart.sendPacket(CHANNEL_A, data, sizeof(data), PACKET_SMALL);

uint8_t rxBuffer[128];
size_t rxSize;
if (uart.receivePacket(CHANNEL_B, rxBuffer, rxSize)) {
    // Process received data
}
```

### GPIO Configuration
```cpp
SC16IS752_GPIO gpio(I2C_ADDRESS);
gpio.beginGPIO();

PinConfig config = {
    .mode = GPIO_OUTPUT,
    .interruptMode = INTERRUPT_DISABLED,
    .initialState = false,
    .debounceTime = 50
};

gpio.configurePin(PIN_NUM, config);
gpio.digitalWrite(PIN_NUM, HIGH);
```

Would you like me to expand on any particular section or add more examples?