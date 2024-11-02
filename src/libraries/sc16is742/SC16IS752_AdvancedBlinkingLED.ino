/*
 * SC16IS752 Advanced LED Blink Example
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
 * GPIO0    ----->  LED1 (with 330Ω resistor to GND)
 * GPIO1    ----->  LED2 (with 330Ω resistor to GND)
 * GPIO2    ----->  LED3 (with 330Ω resistor to GND)
 * GPIO3    ----->  LED4 (with 330Ω resistor to GND)
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
const uint8_t NUM_LEDS = 4;
const uint8_t LED_PINS[NUM_LEDS] = {
    SC16IS752::GPIO_0,
    SC16IS752::GPIO_1,
    SC16IS752::GPIO_2,
    SC16IS752::GPIO_3
};

// Pattern timing
const unsigned long PATTERN_INTERVAL = 100;  // 100ms between updates
unsigned long previousMillis = 0;
uint8_t currentPattern = 0;

// LED patterns (0 = OFF, 1 = ON)
const uint8_t PATTERNS[][4] = {
    {1, 0, 0, 0},  // Single LED chase
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1},
    {1, 1, 0, 0},  // Double LED chase
    {0, 1, 1, 0},
    {0, 0, 1, 1},
    {1, 0, 0, 1},
    {1, 1, 1, 1},  // All on
    {0, 0, 0, 0},  // All off
};
const uint8_t NUM_PATTERNS = sizeof(PATTERNS) / sizeof(PATTERNS[0]);

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\nSC16IS752 Advanced LED Blink Example");
    Serial.println("================================");
    
    // Initialize SC16IS752
    if (!uart.begin(I2C_ADDRESS, I2C_FREQUENCY)) {
        Serial.println("Failed to initialize SC16IS752!");
        while (1) delay(1000);
    }
    Serial.println("SC16IS752 initialized successfully");
    
    // Configure all LED pins as outputs
    bool configSuccess = true;
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        if (!uart.pinMode(LED_PINS[i], OUTPUT)) {
            Serial.printf("Failed to configure GPIO%d!\n", i);
            configSuccess = false;
        }
    }
    
    if (!configSuccess) {
        while (1) delay(1000);
    }
    
    Serial.println("GPIOs configured successfully");
    Serial.println("Starting LED patterns...");
}

void loop() {
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= PATTERN_INTERVAL) {
        previousMillis = currentMillis;
        
        // Update LEDs with current pattern
        bool updateSuccess = true;
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            if (!uart.digitalWrite(LED_PINS[i], PATTERNS[currentPattern][i])) {
                Serial.printf("Failed to write to GPIO%d!\n", i);
                updateSuccess = false;
            }
        }
        
        if (updateSuccess) {
            Serial.printf("Pattern %d applied\n", currentPattern);
        }
        
        // Move to next pattern
        currentPattern = (currentPattern + 1) % NUM_PATTERNS;
    }
}

// Optional: Add different patterns
void runChasePattern() {
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        // Turn on current LED, turn off others
        for (uint8_t j = 0; j < NUM_LEDS; j++) {
            uart.digitalWrite(LED_PINS[j], (j == i) ? HIGH : LOW);
        }
        delay(200);
    }
}

void runBlinkPattern() {
    // All LEDs on
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        uart.digitalWrite(LED_PINS[i], HIGH);
    }
    delay(500);
    
    // All LEDs off
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        uart.digitalWrite(LED_PINS[i], LOW);
    }
    delay(500);
}

void runAlternatePattern() {
    // Even LEDs on, odd LEDs off
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        uart.digitalWrite(LED_PINS[i], i % 2 == 0 ? HIGH : LOW);
    }
    delay(500);
    
    // Odd LEDs on, even LEDs off
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        uart.digitalWrite(LED_PINS[i], i % 2 == 0 ? LOW : HIGH);
    }
    delay(500);
}

