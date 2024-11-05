/*
 * SC16IS752 Basic LED Blink Example
 * 
 * Hardware Connections:
 * ESP32           SC16IS752
 * -----           ---------
 * 3.3V    ----->  VCC
 * GND     ----->  GND
 * GPIO 21 ----->  SCL
 * GPIO 23 ----->  SDA
 * 
 * SC16IS752      Components
 * ---------      ----------
 * GPIO0    ----->  LED (with 330Ω resistor to GND)
 * 
 * Note: LED positive (longer leg) connects to GPIO0 through 330Ω resistor
 *       LED negative (shorter leg) connects to GND
 */

#include <Wire.h>
#include "SC16IS752.h"

// ESP32 Pin Configuration
const int SDA_PIN = 23;
const int SCL_PIN = 21;
const uint8_t I2C_ADDRESS = 0x4D;
const uint32_t I2C_FREQUENCY = 100000;  // 100 kHz

// Create SC16IS752 instance
SC16IS752 uart(Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

// LED Configuration
const uint8_t LED_PIN = SC16IS752::GPIO_0;  // Using GPIO0
const unsigned long BLINK_INTERVAL = 1000;   // Blink every 1 second

// Variables for non-blocking blink
unsigned long previousMillis = 0;  // Will store last time LED was updated
bool ledState = false;            // LED state

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\nSC16IS752 LED Blink Example");
    Serial.println("=========================");
    
    // Initialize SC16IS752
    if (!uart.begin(I2C_ADDRESS, I2C_FREQUENCY)) {
        Serial.println("Failed to initialize SC16IS752!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("SC16IS752 initialized successfully");
    
    // Configure LED pin as output
    if (!uart.pinMode(LED_PIN, OUTPUT)) {
        Serial.println("Failed to configure GPIO!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("GPIO configured successfully");
    
    // Initial LED state
    uart.digitalWrite(LED_PIN, LOW);
    Serial.println("Starting blink sequence...");
}

void loop() {
    // Non-blocking LED blink
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= BLINK_INTERVAL) {
        // Save the last time we blinked the LED
        previousMillis = currentMillis;
        
        // Toggle LED state
        ledState = !ledState;
        
        // Update LED
        if (uart.digitalWrite(LED_PIN, ledState)) {
            Serial.printf("LED %s\n", ledState ? "ON" : "OFF");
        } else {
            Serial.println("Failed to write to GPIO!");
        }
    }
}

