// SC16IS752_RS485_LoopbackTest.ino

#include "SC16IS752.h"
#include "SC16IS752_UART.h"

// Connection Configuration
const uint8_t I2C_ADDRESS = 0x4D;
const int SDA_PIN = 23;
const int SCL_PIN = 21;
const uint32_t UART_BAUD = 19200;  // Standard RS-485 rate

SC16IS752_UART uart(I2C_ADDRESS, Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nSC16IS752 RS-485 Loopback Test");

    if (!uart.begin(UART_BAUD)) {
        Serial.println("UART initialization failed!");
        while(1) delay(100);
    }

    // Configure both channels for RS-485
    configureChannelsForRS485();
    
    // Run tests
    testBasicTransfer();
    testPacketTransfer();
    testFlowControl();
}

void loop() {
    monitorChannels();
    delay(1000);
}

void configureChannelsForRS485() {
    // Enable RS-485 mode on both channels
    uart.enableRS485(SC16IS752::CHANNEL_A, true);
    uart.enableRS485(SC16IS752::CHANNEL_B, true);
    
    // Configure for the same baud rate
    uart.beginUART(SC16IS752::CHANNEL_A, UART_BAUD, SC16IS752_UART::UART_8N1);
    uart.beginUART(SC16IS752::CHANNEL_B, UART_BAUD, SC16IS752_UART::UART_8N1);
    
    Serial.println("Channels configured for RS-485");
}

void testBasicTransfer() {
    Serial.println("\n=== RS-485 Basic Transfer Test ===");
    
    // Send test pattern from A to B
    const uint8_t testPattern[] = {0xAA, 0x55, 0x00, 0xFF};
    
    uart.write(SC16IS752::CHANNEL_A, testPattern, sizeof(testPattern));
    delay(50);  // Allow for transmission
    
    // Read from Channel B
    Serial.print("Received: ");
    while (uart.available(SC16IS752::CHANNEL_B)) {
        int data = uart.read(SC16IS752::CHANNEL_B);
        if (data != -1) {
            Serial.printf("0x%02X ", data);
        }
    }
    Serial.println();
}

void testPacketTransfer() {
    Serial.println("\n=== RS-485 Packet Transfer Test ===");
    
    // Create test packet
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    // Send packet from A to B
    if (uart.sendPacket(SC16IS752::CHANNEL_A, testData, sizeof(testData), 
                       SC16IS752_UART::PACKET_SMALL)) {
        Serial.println("Packet sent from Channel A");
        
        // Wait for packet on Channel B
        uint8_t rxBuffer[32];
        size_t rxSize;
        
        if (uart.waitForPacket(SC16IS752::CHANNEL_B, 100) &&
            uart.receivePacket(SC16IS752::CHANNEL_B, rxBuffer, rxSize)) {
            
            Serial.printf("Received packet (%d bytes): ", rxSize);
            for (size_t i = 0; i < rxSize; i++) {
                Serial.printf("0x%02X ", rxBuffer[i]);
            }
            Serial.println();
        }
    }
}

void testFlowControl() {
    Serial.println("\n=== RS-485 Flow Control Test ===");
    
    // Generate larger test data
    const size_t TEST_SIZE = 64;
    uint8_t testData[TEST_SIZE];
    for (size_t i = 0; i < TEST_SIZE; i++) {
        testData[i] = i & 0xFF;
    }
    
    // Send in chunks to test flow control
    size_t sent = 0;
    while (sent < TEST_SIZE) {
        size_t chunk = min(16u, TEST_SIZE - sent);
        size_t written = uart.write(SC16IS752::CHANNEL_A, 
                                  &testData[sent], chunk);
        
        if (written > 0) {
            sent += written;
            Serial.printf("Sent %d bytes (total: %d/%d)\n", 
                        written, sent, TEST_SIZE);
        }
        
        // Process received data
        while (uart.available(SC16IS752::CHANNEL_B)) {
            uart.read(SC16IS752::CHANNEL_B);
        }
        
        delay(10);  // Allow for processing
    }
}

void monitorChannels() {
    for (uint8_t channel = 0; channel < 2; channel++) {
        SC16IS752_UART::UARTStatus status = uart.getStatus(channel);
        
        Serial.printf("\nChannel %c Status:\n", channel ? 'B' : 'A');
        Serial.printf("  TX FIFO: %d bytes free\n", status.txlvl);
        Serial.printf("  RX FIFO: %d bytes available\n", status.rxlvl);
        
        if (status.error != SC16IS752_UART::UARTError::NONE) {
            Serial.printf("  Error: %d\n", status.error);
        }
    }
}

