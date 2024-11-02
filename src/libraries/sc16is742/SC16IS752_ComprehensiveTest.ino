/*
 * SC16IS752 Comprehensive Examples
 * 
 * This file demonstrates various usage examples of the SC16IS752 driver including:
 * 1. Basic UART Communication
 * 2. GPIO Control
 * 3. PWM Generation
 * 4. Analog Reading
 * 5. Performance Monitoring
 */

#include <Wire.h>
#include "SC16IS752.h"

// ESP32 Pin Configuration
const int SDA_PIN = 23;
const int SCL_PIN = 21;
const uint8_t I2C_ADDRESS = 0x4D;
const uint32_t I2C_FREQUENCY = 400000;  // 400 kHz

// Initialize UART with specific pins
SC16IS752 uart(Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

//=====================================================================
// Example 1: Basic UART Setup and Loopback Test
//=====================================================================

void basicUARTTest() {
    Serial.println("\n=== Basic UART Test ===");
    
    // Configure Channel A for loopback test
    if (uart.initializeUART(SC16IS752::CHANNEL_A, 115200, true) != SC16IS752::OK) {
        Serial.println("Failed to initialize UART Channel A");
        return;
    }

    // Test data
    const uint8_t testData[] = "Hello, SC16IS752!";
    uint8_t rxBuffer[32];
    
    // Send data
    size_t sent = uart.writeBytes(SC16IS752::CHANNEL_A, testData, strlen((char*)testData));
    delay(10); // Wait for transmission
    
    // Read back data
    size_t received = uart.readBytes(SC16IS752::CHANNEL_A, rxBuffer, sent);
    
    // Verify
    rxBuffer[received] = 0; // Null terminate
    Serial.printf("Sent: %d bytes, Received: %d bytes\n", sent, received);
    Serial.printf("Data: %s\n", (char*)rxBuffer);
}

//=====================================================================
// Example 2: GPIO Control
//=====================================================================

void gpioTest() {
    Serial.println("\n=== GPIO Test ===");
    
    // Configure GPIO pins
    uart.pinMode(SC16IS752::GPIO_0, OUTPUT);  // LED output
    uart.pinMode(SC16IS752::GPIO_1, INPUT);   // Button input
    uart.pinMode(SC16IS752::GPIO_2, OUTPUT);  // Blink LED
    
    // Basic output test
    Serial.println("Testing digital output...");
    for (int i = 0; i < 5; i++) {
        uart.digitalWrite(SC16IS752::GPIO_0, HIGH);
        delay(500);
        uart.digitalWrite(SC16IS752::GPIO_0, LOW);
        delay(500);
    }
    
    // Input reading test
    Serial.println("Reading input for 5 seconds...");
    unsigned long endTime = millis() + 5000;
    while (millis() < endTime) {
        int value = uart.digitalRead(SC16IS752::GPIO_1);
        Serial.printf("Input state: %d\n", value);
        delay(500);
    }
}

//=====================================================================
// Example 3: PWM Generation
//=====================================================================

void pwmTest() {
    Serial.println("\n=== PWM Test ===");
    
    // Configure PWM pins with different settings
    // Pin 2: Fast PWM (1 kHz, 8-bit)
    uart.pwmConfig(SC16IS752::GPIO_2, 1000, 8);
    
    // Pin 3: Slow PWM (100 Hz, 10-bit)
    uart.pwmConfig(SC16IS752::GPIO_3, 100, 10);
    
    Serial.println("Testing PWM fade on two pins...");
    
    // Fade test
    for (int i = 0; i < 3; i++) {  // 3 fade cycles
        // Fade up
        for (int value = 0; value < 255; value++) {
            uart.analogWrite(SC16IS752::GPIO_2, value);  // 8-bit
            uart.analogWrite(SC16IS752::GPIO_3, value << 2);  // 10-bit (scaled)
            delay(10);
        }
        
        // Fade down
        for (int value = 255; value >= 0; value--) {
            uart.analogWrite(SC16IS752::GPIO_2, value);
            uart.analogWrite(SC16IS752::GPIO_3, value << 2);
            delay(10);
        }
    }
    
    // Frequency change test
    Serial.println("Testing frequency change...");
    uart.analogWriteFrequency(SC16IS752::GPIO_2, 5000);  // Change to 5 kHz
    uart.analogWrite(SC16IS752::GPIO_2, 127);  // 50% duty cycle
    delay(2000);
}

//=====================================================================
// Example 4: Analog Reading with Different Resolutions
//=====================================================================

void analogTest() {
    Serial.println("\n=== Analog Read Test ===");
    
    // Test different resolutions
    uint8_t resolutions[] = {8, 10, 12};
    
    for (uint8_t res : resolutions) {
        uart.analogReadResolution(res);
        Serial.printf("\nTesting %d-bit resolution:\n", res);
        
        // Read each analog pin
        for (int pin = 0; pin < 4; pin++) {
            int value = uart.analogRead(pin);
            float voltage = (value * 3.3) / ((1 << res) - 1);
            Serial.printf("Pin %d: Raw=%d, Voltage=%.2fV\n", pin, value, voltage);
            delay(100);
        }
    }
}

//=====================================================================
// Example 5: Performance Monitoring
//=====================================================================

void performanceTest() {
    Serial.println("\n=== Performance Test ===");
    
    // Configure UART for normal operation
    uart.initializeUART(SC16IS752::CHANNEL_A, 115200);
    
    // Prepare test data
    uint8_t testBuffer[SC16IS752::TransferStates::BUFFER_SIZE];
    for (size_t i = 0; i < sizeof(testBuffer); i++) {
        testBuffer[i] = i & 0xFF;
    }
    
    // Reset statistics
    uart.resetTransferStats();
    
    // Perform multiple transfers
    Serial.println("Performing 100 transfers...");
    for (int i = 0; i < 100; i++) {
        auto result = uart.writeBufferOptimized(SC16IS752::CHANNEL_A, testBuffer, sizeof(testBuffer));
        if (result.error != SC16IS752::OK) {
            Serial.printf("Transfer %d failed: %d\n", i, result.error);
        }
        delay(10);
    }
    
    // Get and display statistics
    auto stats = uart.getTransferStats();
    Serial.println("\nTransfer Statistics:");
    Serial.printf("Total Transfers: %lu\n", stats.totalTransfers);
    Serial.printf("Successful: %lu\n", stats.successfulTransfers);
    Serial.printf("Total Bytes: %lu\n", stats.totalBytes);
    Serial.printf("Average Rate: %.2f bytes/sec\n", stats.averageRate);
    Serial.printf("Max Successive Transfers: %lu\n", stats.maxSuccessiveTransfers);
}

//=====================================================================
// Example 6: Combined Features Test
//=====================================================================

void combinedTest() {
    Serial.println("\n=== Combined Features Test ===");
    
    // Configure multiple features
    uart.pinMode(SC16IS752::GPIO_0, OUTPUT);  // Status LED
    uart.pinMode(SC16IS752::GPIO_1, INPUT);   // Trigger input
    uart.pwmConfig(SC16IS752::GPIO_2, 1000, 8);  // PWM output
    uart.analogReadResolution(10);  // 10-bit ADC
    
    Serial.println("Monitoring system for 10 seconds...");
    unsigned long endTime = millis() + 10000;
    
    while (millis() < endTime) {
        // Read trigger input
        if (uart.digitalRead(SC16IS752::GPIO_1) == HIGH) {
            // Trigger detected - flash LED and modify PWM
            uart.digitalWrite(SC16IS752::GPIO_0, HIGH);
            
            // Read analog value and use it for PWM
            int analogValue = uart.analogRead(SC16IS752::GPIO_3);
            uart.analogWrite(SC16IS752::GPIO_2, analogValue >> 2);  // Scale to 8-bit
            
            delay(100);
            uart.digitalWrite(SC16IS752::GPIO_0, LOW);
        }
        
        delay(50);  // Small delay between checks
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\nSC16IS752 Example Suite");
    Serial.println("======================");
    
    // Initialize SC16IS752
    if (!uart.begin(I2C_ADDRESS, I2C_FREQUENCY)) {
        Serial.println("Failed to initialize SC16IS752!");
        while (1) delay(1000);
    }
    
    // Run examples
    basicUARTTest();
    delay(1000);

#ifdef TESTGPIO    
    gpioTest();
    delay(1000);
    
    pwmTest();
    delay(1000);
    
    analogTest();
    delay(1000);
#endif
    
    performanceTest();
    delay(1000);

#ifdef TESTGPIO    
    combinedTest();
#endif
    
    Serial.println("\nAll tests completed!");
}

void loop() {
    // Nothing in loop - tests run in setup
    delay(1000);
}

