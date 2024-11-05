# SC16IS752 Driver Documentation and Test Plan

## API Documentation

### Class: SC16IS752

#### Initialization Methods

```cpp
SC16IS752(TwoWire& wire, const I2CPins& pins = I2CPins(DEFAULT_SDA_PIN, DEFAULT_SCL_PIN))
```
Constructor for ESP32/ESP8266 platforms.
- **Parameters:**
  - `wire`: Reference to TwoWire I2C interface
  - `pins`: I2C pin configuration (optional)
- **Usage Example:**
```cpp
TwoWire wire = Wire;
SC16IS752 uart(wire, SC16IS752::I2CPins(21, 22));  // Custom SDA/SCL pins
// or
SC16IS752 uart(wire);  // Default pins
```

```cpp
bool begin(uint8_t i2cAddr = I2C_DEFAULT_ADDR, uint32_t i2cFreq = DEFAULT_I2C_FREQ)
```
Initializes the device with specified I2C address and frequency.
- **Parameters:**
  - `i2cAddr`: I2C address (default: 0x4D)
  - `i2cFreq`: I2C clock frequency (default: 100kHz)
- **Returns:** true if initialization successful
- **Usage Example:**
```cpp
if (!uart.begin(0x4D, 400000)) {  // 400kHz I2C
    Serial.println("Initialization failed");
    return;
}
```

#### UART Configuration

```cpp
int initializeUART(uint8_t channel, uint32_t baudRate, bool testMode = false, uint32_t xtalFreq = 1843200)
```
Configures UART channel with specified parameters.
- **Parameters:**
  - `channel`: CHANNEL_A (0) or CHANNEL_B (1)
  - `baudRate`: Desired baud rate
  - `testMode`: Enable loopback testing if true
  - `xtalFreq`: Crystal frequency (default: 1.8432MHz)
- **Returns:** OK (0) on success, error code on failure
- **Example:**
```cpp
int result = uart.initializeUART(SC16IS752::CHANNEL_A, 115200);
if (result != SC16IS752::OK) {
    Serial.printf("UART init failed: %d\n", result);
    return;
}
```

#### Data Transfer Methods

```cpp
TransferResult writeBufferOptimized(uint8_t channel, const uint8_t* buffer, size_t length)
```
Optimized write for fixed-size transfers (16 bytes).
- **Parameters:**
  - `channel`: UART channel
  - `buffer`: Data buffer
  - `length`: Must be exactly 16 bytes
- **Returns:** TransferResult structure with status
- **Example:**
```cpp
uint8_t data[16] = {0x01, 0x02, ...};
auto result = uart.writeBufferOptimized(SC16IS752::CHANNEL_A, data, 16);
if (result.error != SC16IS752::OK) {
    Serial.printf("Transfer failed: %d\n", result.error);
}
```

### Error Handling

```cpp
UARTStatus getStatus(uint8_t channel)
```
Returns comprehensive UART status.
- **Returns:** UARTStatus structure containing:
  - `lsr`: Line Status Register
  - `msr`: Modem Status Register
  - `txlvl`: TX FIFO level
  - `rxlvl`: RX FIFO level
  - `error`: Error code
- **Example:**
```cpp
UARTStatus status = uart.getStatus(SC16IS752::CHANNEL_A);
if (status.error == SC16IS752::OK) {
    Serial.printf("TX FIFO: %d, RX FIFO: %d\n", status.txlvl, status.rxlvl);
}
```

## Test Coverage Plan

### 1. Initialization Tests

```cpp
void testInitialization() {
    SC16IS752 uart(Wire);
    
    // Test 1.1: Basic Initialization
    TEST_ASSERT(uart.begin());
    
    // Test 1.2: Custom I2C Address
    TEST_ASSERT(uart.begin(0x4E));
    
    // Test 1.3: Invalid I2C Address
    TEST_ASSERT(!uart.begin(0x00));
    
    // Test 1.4: Device Detection
    TEST_ASSERT(uart.detectDevice());
}
```

### 2. UART Configuration Tests

```cpp
void testUARTConfiguration() {
    SC16IS752 uart(Wire);
    uart.begin();
    
    // Test 2.1: Basic UART Configuration
    TEST_ASSERT_EQUAL(SC16IS752::OK, 
        uart.initializeUART(SC16IS752::CHANNEL_A, 9600));
    
    // Test 2.2: High Baud Rate
    TEST_ASSERT_EQUAL(SC16IS752::OK, 
        uart.initializeUART(SC16IS752::CHANNEL_A, 115200));
    
    // Test 2.3: Invalid Channel
    TEST_ASSERT_EQUAL(SC16IS752::ERR_PARAM, 
        uart.initializeUART(2, 9600));
    
    // Test 2.4: Loopback Mode
    TEST_ASSERT_EQUAL(SC16IS752::OK, 
        uart.initializeUART(SC16IS752::CHANNEL_A, 9600, true));
}
```

### 3. Data Transfer Tests

```cpp
void testDataTransfers() {
    SC16IS752 uart(Wire);
    uart.begin();
    uart.initializeUART(SC16IS752::CHANNEL_A, 115200, true);  // Loopback mode
    
    // Test 3.1: Optimized Buffer Write/Read
    uint8_t txBuffer[16] = {0x01, 0x02, 0x03, 0x04};
    uint8_t rxBuffer[16] = {0};
    
    auto writeResult = uart.writeBufferOptimized(
        SC16IS752::CHANNEL_A, txBuffer, 16);
    TEST_ASSERT_EQUAL(SC16IS752::OK, writeResult.error);
    TEST_ASSERT_EQUAL(16, writeResult.bytesTransferred);
    
    auto readResult = uart.readBufferOptimized(
        SC16IS752::CHANNEL_A, rxBuffer, 16);
    TEST_ASSERT_EQUAL(SC16IS752::OK, readResult.error);
    TEST_ASSERT_EQUAL(16, readResult.bytesTransferred);
    TEST_ASSERT_EQUAL_MEMORY(txBuffer, rxBuffer, 16);
    
    // Test 3.2: Chunked Transfers
    uint8_t largeTxBuffer[256];
    uint8_t largeRxBuffer[256];
    for(int i = 0; i < 256; i++) largeTxBuffer[i] = i;
    
    auto largeWriteResult = uart.writeBufferChunked(
        SC16IS752::CHANNEL_A, largeTxBuffer, 256);
    TEST_ASSERT_EQUAL(SC16IS752::OK, largeWriteResult.error);
    TEST_ASSERT_EQUAL(256, largeWriteResult.bytesTransferred);
}
```

### 4. Error Handling Tests

```cpp
void testErrorHandling() {
    SC16IS752 uart(Wire);
    uart.begin();
    
    // Test 4.1: Buffer Overflow
    uint8_t buffer[512];  // Exceeds MAX_TRANSFER_SIZE
    auto result = uart.writeBufferChunked(
        SC16IS752::CHANNEL_A, buffer, 512);
    TEST_ASSERT_EQUAL(SC16IS752::ERR_PARAM, result.error);
    
    // Test 4.2: Invalid Channel
    result = uart.writeBufferChunked(2, buffer, 16);
    TEST_ASSERT_EQUAL(SC16IS752::ERR_PARAM, result.error);
    
    // Test 4.3: Error Status Check
    int errorStatus = uart.getErrorStatus(SC16IS752::CHANNEL_A);
    TEST_ASSERT_BITS_LOW(0xFF, errorStatus);
}
```

### 5. Performance Tests

```cpp
void testPerformance() {
    SC16IS752 uart(Wire);
    uart.begin();
    uart.initializeUART(SC16IS752::CHANNEL_A, 115200);
    
    // Test 5.1: Sustained Transfer Rate
    uint8_t buffer[16];
    unsigned long startTime = millis();
    int successCount = 0;
    
    for(int i = 0; i < 100; i++) {
        auto result = uart.writeBufferOptimized(
            SC16IS752::CHANNEL_A, buffer, 16);
        if(result.error == SC16IS752::OK) successCount++;
    }
    
    unsigned long duration = millis() - startTime;
    float rate = (successCount * 16 * 1000.0f) / duration;
    
    TEST_ASSERT_GREATER_THAN(900.0f, rate);  // Should exceed 900 bps
    
    // Test 5.2: Transfer Statistics
    auto stats = uart.getTransferStats();
    TEST_ASSERT_EQUAL(100, stats.totalTransfers);
    TEST_ASSERT_EQUAL(successCount, stats.successfulTransfers);
}
```

### 6. Platform-Specific Tests

```cpp
#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
void testPlatformSpecific() {
    SC16IS752 uart(Wire);
    
    // Test 6.1: Custom Pin Configuration
    TEST_ASSERT(uart.setPins(21, 22));
    
    // Test 6.2: I2C Frequency Change
    TEST_ASSERT(uart.setI2CFrequency(400000));
    
    // Test 6.3: High-Speed I2C
    TEST_ASSERT(uart.begin(I2C_DEFAULT_ADDR, 400000));
}
#endif
```

## Test Execution Framework

```cpp
void runAllTests() {
    UNITY_BEGIN();
    
    RUN_TEST(testInitialization);
    RUN_TEST(testUARTConfiguration);
    RUN_TEST(testDataTransfers);
    RUN_TEST(testErrorHandling);
    RUN_TEST(testPerformance);
    
    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    RUN_TEST(testPlatformSpecific);
    #endif
    
    UNITY_END();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    runAllTests();
}

void loop() {
    // Empty
}
```

## Testing Guidelines

1. **Test Environment Setup**
   - Use actual SC16IS752 hardware
   - Configure for loopback mode when testing transfers
   - Consider different I2C clock speeds
   - Test on all supported platforms

2. **Test Categories**
   - Unit tests for individual functions
   - Integration tests for complete workflows
   - Performance tests under various conditions
   - Error condition tests
   - Platform-specific feature tests

3. **Performance Metrics to Monitor**
   - Transfer success rate
   - Data throughput
   - Error rates
   - Timing consistency
   - Resource usage

4. **Continuous Integration**
   - Automated test execution
   - Performance regression testing
   - Cross-platform verification
   - Code coverage analysis

5. **Test Result Documentation**
   - Record all test outcomes
   - Track performance metrics
   - Document any platform-specific issues
   - Maintain test coverage reports
