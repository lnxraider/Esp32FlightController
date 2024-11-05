/*
 * SC16IS752 Hardware Flow Control Test
 * ESP32-specific version
 * 
 * Hardware Setup:
 * ESP32 SDA (GPIO 23) -> SC16IS752 SDA
 * ESP32 SCL (GPIO 21) -> SC16IS752 SCL
 * 
 * SC16IS752 Loopback Connections:
 * - Channel A TX (TXA) -> Channel B RX (RXB)
 * - Channel A RTS (RTSA) -> Channel B CTS (CTSB)
 * - Channel B TX (TXB) -> Channel A RX (RXA)
 * - Channel B RTS (RTSB) -> Channel A CTS (CTSA)
 */

/*
 * SC16IS752 Optimized Transfer Example
 * 
 * This example demonstrates the most efficient ways to use the SC16IS752
 * transfer methods in real applications, using the optimal 16-byte size.
 */

#include <Wire.h>
#include "SC16IS752.h"

// ESP32 Pin Configuration
const int SDA_PIN = 23;
const int SCL_PIN = 21;
const uint8_t I2C_ADDRESS = 0x4D;
const uint32_t I2C_FREQUENCY = 400000;  // Using 400kHz for better performance

// Initialize UART with specific pins
SC16IS752 uart(Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

// Use the optimal buffer size from the library
const uint16_t BUFFER_SIZE = SC16IS752::TransferStates::BUFFER_SIZE;  // This is 16

// Circular buffer for handling data streams
class CircularBuffer {
private:
    static const size_t CAPACITY = 256;  // Power of 2 for efficient wrapping
    uint8_t buffer[CAPACITY];
    size_t readIndex = 0;
    size_t writeIndex = 0;
    
public:
    size_t available() {
        return (writeIndex - readIndex) & (CAPACITY - 1);
    }
    
    size_t freeSpace() {
        return CAPACITY - available() - 1;
    }
    
    bool write(uint8_t data) {
        if (freeSpace() == 0) return false;
        buffer[writeIndex] = data;
        writeIndex = (writeIndex + 1) & (CAPACITY - 1);
        return true;
    }
    
    int read() {
        if (available() == 0) return -1;
        uint8_t data = buffer[readIndex];
        readIndex = (readIndex + 1) & (CAPACITY - 1);
        return data;
    }
    
    size_t read(uint8_t* dest, size_t maxLen) {
        size_t count = 0;
        while (count < maxLen && available() > 0) {
            dest[count++] = buffer[readIndex];
            readIndex = (readIndex + 1) & (CAPACITY - 1);
        }
        return count;
    }
    
    size_t write(const uint8_t* src, size_t len) {
        size_t count = 0;
        while (count < len && freeSpace() > 0) {
            buffer[writeIndex] = src[count++];
            writeIndex = (writeIndex + 1) & (CAPACITY - 1);
        }
        return count;
    }
};

// Buffers for each channel
CircularBuffer txBufferA;
CircularBuffer rxBufferA;
CircularBuffer txBufferB;
CircularBuffer rxBufferB;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\nSC16IS752 Optimized Transfer Example");
    Serial.println("==================================");
    
    if (!initializeHardware()) {
        Serial.println("Hardware initialization failed!");
        while (1) delay(1000);
    }
}

bool initializeHardware() {
    // Initialize with optimal I2C frequency
    if (!uart.begin(I2C_ADDRESS, I2C_FREQUENCY)) {
        Serial.println("Failed to initialize SC16IS752!");
        return false;
    }
    
    // Initialize both channels with optimal settings
    int result = uart.initializeUART(SC16IS752::CHANNEL_A, 115200);
    if (result != SC16IS752::OK) {
        Serial.printf("Failed to initialize Channel A: %d\n", result);
        return false;
    }
    
    result = uart.initializeUART(SC16IS752::CHANNEL_B, 115200);
    if (result != SC16IS752::OK) {
        Serial.printf("Failed to initialize Channel B: %d\n", result);
        return false;
    }
    
    return true;
}

// Optimized send function using optimal buffer size
bool sendOptimized(uint8_t channel, CircularBuffer& txBuffer) {
    if (txBuffer.available() < BUFFER_SIZE) {
        return false;  // Not enough data to send
    }
    
    uint8_t tempBuffer[BUFFER_SIZE];
    txBuffer.read(tempBuffer, BUFFER_SIZE);
    
    auto result = uart.writeBufferOptimized(channel, tempBuffer, BUFFER_SIZE);
    return (result.error == SC16IS752::OK && result.complete);
}

// Optimized receive function using optimal buffer size
bool receiveOptimized(uint8_t channel, CircularBuffer& rxBuffer) {
    if (rxBuffer.freeSpace() < BUFFER_SIZE) {
        return false;  // Not enough space to receive
    }
    
    uint8_t tempBuffer[BUFFER_SIZE];
    auto result = uart.readBufferOptimized(channel, tempBuffer, BUFFER_SIZE);
    
    if (result.error == SC16IS752::OK && result.complete) {
        return rxBuffer.write(tempBuffer, BUFFER_SIZE) == BUFFER_SIZE;
    }
    return false;
}

// Example of sending a larger data stream efficiently
void sendDataStream(uint8_t channel, const uint8_t* data, size_t length) {
    CircularBuffer& txBuffer = (channel == SC16IS752::CHANNEL_A) ? txBufferA : txBufferB;
    
    // Fill the circular buffer
    size_t written = txBuffer.write(data, length);
    
    // Send in optimal chunks
    while (txBuffer.available() >= BUFFER_SIZE) {
        if (!sendOptimized(channel, txBuffer)) {
            // Handle send failure
            Serial.println("Send failed!");
            break;
        }
        
        // Optional: Add more data if available
        if (written < length) {
            size_t remaining = length - written;
            written += txBuffer.write(data + written, remaining);
        }
    }
}

// Example of receiving a data stream efficiently
size_t receiveDataStream(uint8_t channel, uint8_t* buffer, size_t maxLength) {
    CircularBuffer& rxBuffer = (channel == SC16IS752::CHANNEL_A) ? rxBufferA : rxBufferB;
    size_t totalReceived = 0;
    unsigned long timeout = millis() + 1000;  // 1 second timeout
    
    while (totalReceived < maxLength && millis() < timeout) {
        // Try to receive an optimal chunk
        if (receiveOptimized(channel, rxBuffer)) {
            // Read from circular buffer to user buffer
            size_t count = min(rxBuffer.available(), maxLength - totalReceived);
            totalReceived += rxBuffer.read(buffer + totalReceived, count);
        }
        yield();  // Give ESP32 some processing time
    }
    
    return totalReceived;
}

void loop() {
    // Example usage with test data
    static unsigned long lastSend = 0;
    const unsigned long SEND_INTERVAL = 100;  // Send every 100ms
    
    if (millis() - lastSend >= SEND_INTERVAL) {
        lastSend = millis();
        
        // Create test data
        uint8_t testData[BUFFER_SIZE];
        for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
            testData[i] = i;
        }
        
        // Send test data using optimal transfer
        auto result = uart.writeBufferOptimized(SC16IS752::CHANNEL_A, testData, BUFFER_SIZE);
        
        if (result.error == SC16IS752::OK && result.complete) {
            // Read back the data
            uint8_t rxData[BUFFER_SIZE];
            auto rxResult = uart.readBufferOptimized(SC16IS752::CHANNEL_B, rxData, BUFFER_SIZE);
            
            if (rxResult.error == SC16IS752::OK && rxResult.complete) {
                // Verify data
                bool dataOk = true;
                for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
                    if (rxData[i] != testData[i]) {
                        dataOk = false;
                        break;
                    }
                }
                
                Serial.printf("Transfer %s: %d bytes\n", 
                    dataOk ? "OK" : "ERROR",
                    result.bytesTransferred);
            }
        }
    }
    
    // Process any received data
    if (uart.isRxAvailable(SC16IS752::CHANNEL_A)) {
        uint8_t rxBuffer[BUFFER_SIZE];
        auto result = uart.readBufferOptimized(SC16IS752::CHANNEL_A, rxBuffer, BUFFER_SIZE);
        if (result.error == SC16IS752::OK) {
            // Process received data...
        }
    }
}

