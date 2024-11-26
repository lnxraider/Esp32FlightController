// SC16IS752_UART_MultiMode_Examples.ino

#include "SC16IS752.h"
#include "SC16IS752_UART.h"

// Use same connection config as before
const uint8_t I2C_ADDRESS = 0x4D;
const int SDA_PIN = 23;
const int SCL_PIN = 21;

SC16IS752_UART uart(I2C_ADDRESS, Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

// 1. Mixed Mode Example - Different baud rates and configurations per channel
void mixedModeExample() {
    Serial.println("\n=== Mixed Mode Example ===");
    
    // Configure Channel A for high-speed data
    uart.beginUART(SC16IS752::CHANNEL_A, 115200, SC16IS752_UART::UART_8N1);
    uart.setFlowControl(SC16IS752::CHANNEL_A, true);  // Enable flow control
    
    // Configure Channel B for low-speed control messages
    uart.beginUART(SC16IS752::CHANNEL_B, 9600, SC16IS752_UART::UART_8E1);
    uart.setFlowControl(SC16IS752::CHANNEL_B, false); // No flow control needed
    
    // Send data on both channels
    const char* highSpeedData = "High speed data stream";
    const char* lowSpeedCmd = "STATUS?";
    
    uart.write(SC16IS752::CHANNEL_A, (uint8_t*)highSpeedData, strlen(highSpeedData));
    uart.write(SC16IS752::CHANNEL_B, (uint8_t*)lowSpeedCmd, strlen(lowSpeedCmd));
    
    // Monitor both channels
    printChannelStatus(SC16IS752::CHANNEL_A);
    printChannelStatus(SC16IS752::CHANNEL_B);
}

// 2. RS-485 Network Example
void rs485NetworkExample() {
    Serial.println("\n=== RS-485 Network Example ===");
    
    // Configure Channel A as RS-485 master
    uart.beginUART(SC16IS752::CHANNEL_A, 19200, SC16IS752_UART::UART_8N1);
    uart.enableRS485(SC16IS752::CHANNEL_A, true);

    // Example addresses and commands
    const uint8_t DEVICE_ADDRESSES[] = {0x01, 0x02, 0x03};
    const uint8_t COMMAND = 0x10;  // Example command
    const uint8_t DATA = 0x55;     // Example data
    
    // Send command to each device
    for (uint8_t addr : DEVICE_ADDRESSES) {
        uint8_t packet[] = {addr, COMMAND, DATA};
        
        if (uart.sendPacket(SC16IS752::CHANNEL_A, packet, sizeof(packet), 
                          SC16IS752_UART::PACKET_CONTROL)) {
            
            Serial.printf("Sent command to device 0x%02X\n", addr);
            
            // Wait for response
            uint8_t response[32];
            size_t responseSize;
            
            if (uart.waitForPacket(SC16IS752::CHANNEL_A, 100) &&
                uart.receivePacket(SC16IS752::CHANNEL_A, response, responseSize)) {
                
                Serial.printf("Response from device 0x%02X: ", addr);
                for (size_t i = 0; i < responseSize; i++) {
                    Serial.printf("%02X ", response[i]);
                }
                Serial.println();
            }
        }
        delay(50); // Gap between transactions
    }
}

// 3. Reliable Transfer Protocol Example
void reliableTransferExample() {
    Serial.println("\n=== Reliable Transfer Example ===");
    
    const uint8_t MAX_RETRIES = 3;
    const uint32_t TIMEOUT = 1000;
    uint8_t retryCount = 0;
    bool transferComplete = false;
    
    // Data to send
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    while (!transferComplete && retryCount < MAX_RETRIES) {
        // Send packet
        if (uart.sendPacket(SC16IS752::CHANNEL_A, data, sizeof(data), 
                          SC16IS752_UART::PACKET_SMALL)) {
            
            // Wait for acknowledgment
            uint8_t ack[8];
            size_t ackSize;
            
            if (uart.waitForPacket(SC16IS752::CHANNEL_B, TIMEOUT) &&
                uart.receivePacket(SC16IS752::CHANNEL_B, ack, ackSize)) {
                
                if (ackSize > 0 && ack[0] == 0xAA) {  // Valid ACK
                    transferComplete = true;
                    Serial.println("Transfer completed successfully");
                    break;
                }
            }
        }
        
        retryCount++;
        Serial.printf("Retry %d of %d\n", retryCount, MAX_RETRIES);
        delay(100 * retryCount);  // Increasing delay between retries
    }
    
    SC16IS752_UART::TransferStats stats = uart.getTransferStats(SC16IS752::CHANNEL_A);
    printTransferStats(stats);
}

// 4. MODBUS RTU Example
void modbusRTUExample() {
    Serial.println("\n=== MODBUS RTU Example ===");
    
    // Configure for MODBUS RTU
    uart.beginUART(SC16IS752::CHANNEL_A, 9600, SC16IS752_UART::UART_8E1);
    uart.enableRS485(SC16IS752::CHANNEL_A, true);
    
    // Example MODBUS query - Read Holding Registers
    uint8_t query[] = {
        0x01,           // Slave Address
        0x03,           // Function Code (Read Holding Registers)
        0x00, 0x00,     // Starting Address
        0x00, 0x02,     // Quantity of Registers
        0x00, 0x00      // CRC (to be calculated)
    };
    
    if (uart.sendPacket(SC16IS752::CHANNEL_A, query, sizeof(query), 
                       SC16IS752_UART::PACKET_CONTROL)) {
        
        uint8_t response[32];
        size_t responseSize;
        
        if (uart.waitForPacket(SC16IS752::CHANNEL_A, 1000) &&
            uart.receivePacket(SC16IS752::CHANNEL_A, response, responseSize)) {
            
            Serial.printf("MODBUS Response (%d bytes):", responseSize);
            for (size_t i = 0; i < responseSize; i++) {
                Serial.printf(" %02X", response[i]);
            }
            Serial.println();
        }
    }
}

// 5. Loopback Test Example
void loopbackTestExample() {
    Serial.println("\n=== Loopback Test Example ===");
    
    // Generate test patterns
    const size_t PATTERN_SIZE = 64;
    uint8_t testPattern[PATTERN_SIZE];
    
    for (size_t i = 0; i < PATTERN_SIZE; i++) {
        testPattern[i] = ((i * 7) + 13) & 0xFF;  // Simple pattern
    }
    
    // Send patterns and verify
    size_t totalTests = 10;
    size_t passedTests = 0;
    
    for (size_t test = 0; test < totalTests; test++) {
        // Send pattern
        size_t written = uart.write(SC16IS752::CHANNEL_A, testPattern, PATTERN_SIZE);
        
        if (written == PATTERN_SIZE) {
            // Read and verify
            uint8_t rxBuffer[PATTERN_SIZE];
            size_t received = 0;
            uint32_t startTime = millis();
            
            while (received < PATTERN_SIZE && 
                   (millis() - startTime) < 1000) {
                
                if (uart.available(SC16IS752::CHANNEL_B)) {
                    int data = uart.read(SC16IS752::CHANNEL_B);
                    if (data != -1) {
                        rxBuffer[received++] = data;
                    }
                }
            }
            
            // Verify received data
            if (received == PATTERN_SIZE && 
                memcmp(testPattern, rxBuffer, PATTERN_SIZE) == 0) {
                passedTests++;
            }
        }
        
        delay(100);  // Gap between tests
    }
    
    Serial.printf("Loopback Test Results: %d/%d passed\n", 
                 passedTests, totalTests);
                 
    // Print final statistics
    printChannelStatus(SC16IS752::CHANNEL_A);
    printChannelStatus(SC16IS752::CHANNEL_B);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nSC16IS752 Advanced UART Examples");

    if (!uart.begin(115200)) {
        Serial.println("UART initialization failed!");
        while(1) delay(100);
    }

    // Run examples
    mixedModeExample();
    delay(1000);
    
    rs485NetworkExample();
    delay(1000);
    
    reliableTransferExample();
    delay(1000);
    
    modbusRTUExample();
    delay(1000);
    
    loopbackTestExample();
}

void loop() {
    delay(1000);
}

// Helper functions (same as previous example)
void printChannelStatus(uint8_t channel) {
    SC16IS752_UART::UARTStatus status = uart.getStatus(channel);
    
    Serial.printf("Channel %c:\n", channel ? 'B' : 'A');
    Serial.printf("  TX FIFO: %d bytes free\n", status.txlvl);
    Serial.printf("  RX FIFO: %d bytes available\n", status.rxlvl);
    
    if (status.error != SC16IS752_UART::UARTError::NONE) {
        Serial.printf("  Error: %d\n", status.error);
    }
}

void printTransferStats(const SC16IS752_UART::TransferStats& stats) {
    Serial.println("Transfer Statistics:");
    Serial.printf("  Bytes TX/RX: %lu/%lu\n", stats.bytesSent, stats.bytesReceived);
    Serial.printf("  Transfers (OK/Fail): %lu/%lu\n", 
                 stats.successfulTransfers, stats.failedTransfers);
    Serial.printf("  Average Transfer Time: %.2f ms\n", stats.averageTransferTime);
    Serial.printf("  Peak Transfer Rate: %.2f bytes/sec\n", stats.peakTransferRate);
    
    if (stats.errors > 0) {
        Serial.printf("  Errors: %lu (CRC: %lu)\n", stats.errors, stats.crcErrors);
        Serial.printf("  Buffer Overruns/Underruns: %lu/%lu\n", 
                     stats.bufferOverruns, stats.bufferUnderruns);
    }
}

void printBuffer(const uint8_t* buffer, size_t size) {
    for (size_t i = 0; i < size; i++) {
        Serial.printf("%02X ", buffer[i]);
        if ((i + 1) % 16 == 0) Serial.println();
    }
    Serial.println();
}

