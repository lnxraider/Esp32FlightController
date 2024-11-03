// SC16IS752_ComprehensiveTest.ino
// Complete test suite for SC16IS752 dual UART with platform-specific optimizations
// Comprehensive stress test for SC16IS752 dual UART

#include "SC16IS752.h"

//=====================================================================
// Test Configuration
//=====================================================================

// Platform-specific I2C pins
#if defined(PLATFORM_ESP32)
    #define I2C_SDA 23
    #define I2C_SCL 21
    #define I2C_FREQ 400000  // 400kHz for ESP32
    //#define I2C_FREQ 100000  // Reduced from 400000 to 100000
#elif defined(PLATFORM_ESP8266)
    #define I2C_SDA 4
    #define I2C_SCL 5
    #define I2C_FREQ 100000  // 100kHz for ESP8266
#else
    #define I2C_FREQ 100000  // 100kHz for Arduino
#endif

// Test parameters
const size_t SMALL_TEST_SIZE = 16;
const size_t LARGE_TEST_SIZE = 64;
const unsigned long TEST_DURATION = 30000;  // 30 seconds per test
const unsigned long REPORT_INTERVAL = 5000; // Status report every 5 seconds
const uint32_t TEST_BAUD_RATE = 9600;
const uint8_t I2C_ADDRESS = 0x4D;

// Platform-specific transfer intervals
#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
const unsigned long TRANSFER_INTERVAL = 50;  // 50ms between transfers for ESP
#else
const unsigned long TRANSFER_INTERVAL = 20;  // 20ms for other platforms
#endif

//=====================================================================
// Test Statistics Structure
//=====================================================================

struct TestStats {
    uint32_t totalTests;
    uint32_t successfulTests;
    uint32_t failedTests;
    uint32_t totalBytes;
    uint32_t totalTime;
    float avgTransferRate;
    
    TestStats() : totalTests(0), successfulTests(0), failedTests(0),
                 totalBytes(0), totalTime(0), avgTransferRate(0) {}
                 
    void reset() {
        totalTests = 0;
        successfulTests = 0;
        failedTests = 0;
        totalBytes = 0;
        totalTime = 0;
        avgTransferRate = 0;
    }
};

//=====================================================================
// Global Instance
//=====================================================================

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
SC16IS752 uart(Wire, SC16IS752::I2CPins(I2C_SDA, I2C_SCL));
#else
SC16IS752 uart(Wire);
#endif

//=====================================================================
// Print Functions
//=====================================================================

void printStatus(SC16IS752::UARTStatus status) {
    Serial.print(F("LSR: 0x"));
    Serial.print(status.lsr, HEX);
    Serial.print(F(", TX FIFO: "));
    Serial.print(status.txlvl);
    Serial.print(F(", RX FIFO: "));
    Serial.print(status.rxlvl);
    
    if (status.error != SC16IS752::OK) {
        Serial.print(F(", Error: "));
        Serial.print(status.error);
    }

    // Check for specific errors using error status
    int errors = uart.getErrorStatus(uart.CHANNEL_A);
    if (errors != SC16IS752::OK) {
        if (errors & SC16IS752::ERROR_OVERRUN) Serial.print(F(" [OVERRUN]"));
        if (errors & SC16IS752::ERROR_PARITY) Serial.print(F(" [PARITY]"));
        if (errors & SC16IS752::ERROR_FRAMING) Serial.print(F(" [FRAMING]"));
        if (errors & SC16IS752::ERROR_BREAK) Serial.print(F(" [BREAK]"));
        if (errors & SC16IS752::ERROR_FIFO) Serial.print(F(" [FIFO]"));
    }
    
    Serial.println();
}

void printTestStats(const TestStats& stats) {
    Serial.println(F("\n=== Test Statistics ==="));
    Serial.print(F("Total Tests: "));
    Serial.println(stats.totalTests);
    Serial.print(F("Successful: "));
    Serial.println(stats.successfulTests);
    Serial.print(F("Failed: "));
    Serial.println(stats.failedTests);
    Serial.print(F("Success Rate: "));
    Serial.print((float)stats.successfulTests / stats.totalTests * 100);
    Serial.println(F("%"));
    if (stats.totalTime > 0) {
        Serial.print(F("Average Transfer Rate: "));
        Serial.print(stats.avgTransferRate);
        Serial.println(F(" bytes/sec"));
    }
}

//=====================================================================
// Test Functions
//=====================================================================

void runContinuousTest(uint8_t channel, size_t size, unsigned long duration, TestStats& stats) {
    Serial.print(F("\n=== Starting Continuous Test ("));
    Serial.print(size);
    Serial.println(F(" bytes) ==="));

    uint8_t* txBuffer = new uint8_t[size];
    uint8_t* rxBuffer = new uint8_t[size];

    if (!txBuffer || !rxBuffer) {
        Serial.println(F("Memory allocation failed!"));
        delete[] txBuffer;
        delete[] rxBuffer;
        return;
    }

    // Initialize test data
    for (size_t i = 0; i < size; i++) {
        txBuffer[i] = i & 0xFF;
    }

    unsigned long startTime = millis();
    unsigned long lastReport = startTime;
    unsigned long lastTransfer = startTime;

    while (millis() - startTime < duration) {
        // Check if enough time has passed since last transfer
        if (millis() - lastTransfer < TRANSFER_INTERVAL) {
            delay(1);
            continue;
        }
        
        lastTransfer = millis();
        stats.totalTests++;
        
        // Clear receive buffer
        memset(rxBuffer, 0, size);

        // Perform transfer
        SC16IS752::TransferResult txResult = uart.writeBufferChunked(channel, txBuffer, size);
        
        // Platform-specific delay between TX and RX
        #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
        delay(20);
        #else
        delay(10);
        #endif
        
        SC16IS752::TransferResult rxResult = uart.readBufferChunked(channel, rxBuffer, size);

        bool success = false;
        if (txResult.complete && rxResult.complete) {
            success = (memcmp(txBuffer, rxBuffer, size) == 0);
        }

        if (success) {
            stats.successfulTests++;
            stats.totalBytes += size;
            stats.totalTime += txResult.timeMs + rxResult.timeMs;
        } else {
            stats.failedTests++;
            
            // Print detailed error information
            Serial.println(F("\nTransfer failed:"));
            if (!txResult.complete) {
                Serial.print(F("TX Error: "));
                Serial.println(txResult.error);
            }
            if (!rxResult.complete) {
                Serial.print(F("RX Error: "));
                Serial.println(rxResult.error);
            }
            
            // Get current status
            SC16IS752::UARTStatus status = uart.getStatus(channel);
            Serial.print(F("UART Status - "));
            printStatus(status);
        }

        // Progress report
        if (millis() - lastReport >= REPORT_INTERVAL) {
            Serial.print(F("\nProgress - Success: "));
            Serial.print(stats.successfulTests);
            Serial.print(F(", Failed: "));
            Serial.println(stats.failedTests);
            
            if (stats.totalTime > 0) {
                stats.avgTransferRate = (float)(stats.totalBytes * 1000) / stats.totalTime;
                Serial.print(F("Current transfer rate: "));
                Serial.print(stats.avgTransferRate);
                Serial.println(F(" bytes/sec"));
            }
            
            lastReport = millis();
        }
    }

    delete[] txBuffer;
    delete[] rxBuffer;
}

//=====================================================================
// Setup and Main Loop
//=====================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("\nSC16IS752 Comprehensive Test"));

    // Initialize I2C with platform-specific settings
    #if defined(PLATFORM_ESP32)
        Wire.begin(I2C_SDA, I2C_SCL);
        uart.setPins(I2C_SDA, I2C_SCL);
        Serial.println(F("ESP32 I2C initialized"));
        Serial.printf("SDA: %d, SCL: %d\n", I2C_SDA, I2C_SCL);
    #elif defined(PLATFORM_ESP8266)
        Wire.begin(I2C_SDA, I2C_SCL);
        Serial.println(F("ESP8266 I2C initialized"));
        Serial.printf("SDA: %d, SCL: %d\n", I2C_SDA, I2C_SCL);
    #else
        Wire.begin();
        Serial.println(F("Arduino I2C initialized"));
    #endif

    Wire.setClock(I2C_FREQ);
    Serial.print(F("I2C Frequency set to: "));
    Serial.println(I2C_FREQ);

    // Scan for I2C devices
    scanI2C();

    // Try to initialize UART
    if (!uart.begin(I2C_ADDRESS)) {
        Serial.println(F("Failed to initialize UART!"));
        Serial.println(F("Checking I2C connection..."));
        
        Wire.beginTransmission(I2C_ADDRESS);
        byte error = Wire.endTransmission();
        
        Serial.print(F("I2C error code: "));
        Serial.println(error);
        
        while (1) delay(1000);
    }

    // Configure both channels with test mode enabled
    if (uart.initializeUART(SC16IS752::CHANNEL_A, TEST_BAUD_RATE, true) != SC16IS752::OK) {
        Serial.println(F("Failed to initialize Channel A!"));
        while (1) delay(1000);
    }

    if (uart.initializeUART(SC16IS752::CHANNEL_B, TEST_BAUD_RATE, true) != SC16IS752::OK) {
        Serial.println(F("Failed to initialize Channel B!"));
        while (1) delay(1000);
    }

    if (!testLoopbackMode(SC16IS752::CHANNEL_A)) {
        Serial.println(F("Channel A loopback mode verification failed!"));
        while (1) delay(1000);
    }

    if (!testLoopbackMode(SC16IS752::CHANNEL_B)) {
        Serial.println(F("Channel B loopback mode verification failed!"));
        while (1) delay(1000);
    }

    Serial.println(F("Initialization and loopback verification successful"));

}

void loop() {
    TestStats stats;
    uint8_t txBuffer[SMALL_TEST_SIZE];
    uint8_t rxBuffer[SMALL_TEST_SIZE];

    // Initialize test data
    for (size_t i = 0; i < SMALL_TEST_SIZE; i++) {
        txBuffer[i] = i & 0xFF;
    }

    // Single transfer test
    Serial.println(F("\n=== Single Transfer Test ==="));
    bool singleTestResult = testSingleTransfer(SC16IS752::CHANNEL_A, 
                                             txBuffer, rxBuffer, 
                                             SMALL_TEST_SIZE);
    Serial.print(F("Single test "));
    Serial.println(singleTestResult ? F("passed") : F("failed"));

    // Small packet continuous test
    stats.reset();
    runContinuousTest(SC16IS752::CHANNEL_A, SMALL_TEST_SIZE, TEST_DURATION, stats);
    printTestStats(stats);

    // Large packet test
    Serial.println(F("\n=== Large Packet Test ==="));
    stats.reset();
    runContinuousTest(SC16IS752::CHANNEL_A, LARGE_TEST_SIZE, TEST_DURATION, stats);
    printTestStats(stats);

    // Dual channel test
    Serial.println(F("\n=== Dual Channel Test ==="));
    bool channel_a_ok = testSingleTransfer(SC16IS752::CHANNEL_A, 
                                         txBuffer, rxBuffer, 
                                         SMALL_TEST_SIZE);
    bool channel_b_ok = testSingleTransfer(SC16IS752::CHANNEL_B, 
                                         txBuffer, rxBuffer, 
                                         SMALL_TEST_SIZE);
    Serial.print(F("Channel A: "));
    Serial.println(channel_a_ok ? F("OK") : F("Failed"));
    Serial.print(F("Channel B: "));
    Serial.println(channel_b_ok ? F("OK") : F("Failed"));

    Serial.println(F("\nAll tests completed. Restarting in 5 seconds..."));
    delay(5000);
}

bool testLoopbackMode(uint8_t channel) {
    Serial.println(F("\nTesting loopback configuration..."));
    
    // Send a single byte
    const uint8_t testByte = 0x55;
    int result = uart.writeByte(channel, testByte);
    if (result != SC16IS752::OK) {
        Serial.println(F("Loopback write failed"));
        return false;
    }
    
    delay(50);  // Wait for loopback
    
    // Read it back
    int readValue = uart.readByte(channel);
    if (readValue < 0) {
        Serial.println(F("Loopback read failed"));
        return false;
    }
    
    bool success = (readValue == testByte);
    Serial.print(F("Loopback test "));
    Serial.println(success ? F("passed") : F("failed"));
    return success;
}

void scanI2C() {
    Serial.println(F("\nScanning I2C bus..."));
    byte error, address;
    int devices = 0;
 
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
 
        if (error == 0) {
            Serial.print(F("I2C device found at address 0x"));
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println();
            devices++;
        }
    }
    
    if (devices == 0) {
        Serial.println(F("No I2C devices found!"));
    }
}

void printDetailedStatus(SC16IS752::UARTStatus status) {
    Serial.print(F("LSR: 0x"));
    Serial.print(status.lsr, HEX);
    Serial.print(F(" ("));
    if (status.lsr & uart.LSR_DATA_READY) Serial.print(F("DR "));
    if (status.lsr & uart.LSR_OVERRUN_ERROR) Serial.print(F("OE "));
    if (status.lsr & uart.LSR_PARITY_ERROR) Serial.print(F("PE "));
    if (status.lsr & uart.LSR_FRAMING_ERROR) Serial.print(F("FE "));
    if (status.lsr & uart.LSR_BREAK_INTERRUPT) Serial.print(F("BI "));
    if (status.lsr & uart.LSR_THR_EMPTY) Serial.print(F("THRE "));
    if (status.lsr & uart.LSR_TRANSMITTER_EMPTY) Serial.print(F("TEMT "));
    Serial.println(F(")"));
    
    Serial.print(F("TX FIFO: "));
    Serial.println(status.txlvl);
    Serial.print(F("RX FIFO: "));
    Serial.println(status.rxlvl);
    
    if (status.error != uart.OK) {
        Serial.print(F("Error: "));
        Serial.println(status.error);
    }
}

bool testSingleTransfer(uint8_t channel, const uint8_t* txBuffer, uint8_t* rxBuffer, size_t size) {
    Serial.print(F("\nTesting transfer of "));
    Serial.print(size);
    Serial.println(F(" bytes"));

    // Get initial status
    SC16IS752::UARTStatus status = uart.getStatus(channel);
    Serial.println(F("Initial Status:"));
    printDetailedStatus(status);

    // Verify loopback mode
    int mcr = uart.readReg(uart.regAddr(uart.REG_MCR, channel));
    Serial.print(F("MCR: 0x"));
    Serial.println(mcr, HEX);

    // Send data
    Serial.println(F("Starting transmission..."));
    SC16IS752::TransferResult txResult = uart.writeBufferChunked(channel, txBuffer, size);

    if (!txResult.complete) {
        Serial.println(F("Transmission failed:"));
        Serial.print(F("Bytes sent: "));
        Serial.print(txResult.bytesTransferred);
        Serial.print(F("/"));
        Serial.println(size);
        Serial.print(F("Error code: "));
        Serial.println(txResult.error);
        return false;
    }

    // Get status after write
    status = uart.getStatus(channel);
    Serial.println(F("Status after write:"));
    printDetailedStatus(status);

    // Read back data
    Serial.println(F("Reading back data..."));
    SC16IS752::TransferResult rxResult = uart.readBufferChunked(channel, rxBuffer, size);
    
    Serial.println(F("\nRead Results:"));
    Serial.print(F("Bytes received: "));
    Serial.print(rxResult.bytesTransferred);
    Serial.print(F("/"));
    Serial.println(size);
    
    if (rxResult.bytesTransferred > 0) {
        Serial.println(F("Received data:"));
        for (size_t i = 0; i < rxResult.bytesTransferred; i++) {
            Serial.print(F("0x"));
            Serial.print(rxBuffer[i], HEX);
            Serial.print(F(" "));
            if ((i + 1) % 8 == 0) Serial.println();
        }
        Serial.println();
    }

    bool success = (rxResult.complete && memcmp(txBuffer, rxBuffer, size) == 0);
    Serial.print(F("Transfer "));
    Serial.println(success ? F("successful") : F("failed"));

    return success;
}

