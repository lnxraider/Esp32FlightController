/*
 * SC16IS752 LED Example
 * 
 * Hardware Connections:
 * ESP32    -> SC16IS752
 * GPIO 21  -> SCL
 * GPIO 23  -> SDA
 * 3.3V     -> VCC
 * GND      -> GND
 * 
 * SC16IS752 -> LED
 * GPIO0    -> LED + (with appropriate current limiting resistor)
 * GND      -> LED -
 * 
 * Note: Use an appropriate current limiting resistor (e.g., 220Ω) in series with the LED
 */

#include <Wire.h>
#include "SC16IS752.h"

// I2C Configuration
#define I2C_ADDRESS 0x4D  // Default address for SC16IS752
#define SDA_PIN     23    // Default SDA pin for ESP32
#define SCL_PIN     21    // Default SCL pin for ESP32

// LED Configuration
#define LED_PIN     GPIO_0  // Using GPIO0 of SC16IS752
#define FADE_DELAY  15      // Delay for LED fading (milliseconds)
#define BLINK_DELAY 1000    // Delay for LED blinking (milliseconds)

// Create SC16IS752 instance
SC16IS752 gpio(Wire);

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("\nSC16IS752 LED Example");

  // Initialize I2C
  if (!gpio.begin(I2C_ADDRESS)) {
    Serial.println("Failed to initialize SC16IS752!");
    while (1) {
      delay(100);
    }
  }
  
  Serial.println("SC16IS752 initialized successfully");

  // Configure LED pin as output
  gpio.pinMode(LED_PIN, GPIO_OUTPUT);
  
  // Initial state: LED off
  gpio.digitalWrite(LED_PIN, GPIO_LOW);
}

// Demo modes
enum DemoMode {
  BLINK,
  FADE
};

DemoMode currentMode = BLINK;
unsigned long lastModeSwitch = 0;
const unsigned long MODE_SWITCH_INTERVAL = 10000; // Switch modes every 10 seconds

void loop() {
  // Switch between demo modes every MODE_SWITCH_INTERVAL milliseconds
  if (millis() - lastModeSwitch >= MODE_SWITCH_INTERVAL) {
    currentMode = (currentMode == BLINK) ? FADE : BLINK;
    lastModeSwitch = millis();
    
    if (currentMode == BLINK) {
      Serial.println("Switching to Blink mode");
      gpio.pinMode(LED_PIN, GPIO_OUTPUT);      // Ensure pin is in GPIO mode
    } else {
      Serial.println("Switching to Fade mode");
      gpio.pwmBegin(LED_PIN);                 // Configure pin for PWM
    }
  }

  // Execute current demo mode
  if (currentMode == BLINK) {
    blinkDemo();
  } else {
    fadeDemo();
  }
}

void blinkDemo() {
  // Simple on/off blinking
  gpio.digitalWrite(LED_PIN, GPIO_HIGH);
  delay(BLINK_DELAY);
  gpio.digitalWrite(LED_PIN, GPIO_LOW);
  delay(BLINK_DELAY);
}

void fadeDemo() {
  // Fade up
  for (int brightness = 0; brightness <= 255; brightness++) {
    gpio.pwmWrite(LED_PIN, brightness);
    delay(FADE_DELAY);
  }
  
  // Fade down
  for (int brightness = 255; brightness >= 0; brightness--) {
    gpio.pwmWrite(LED_PIN, brightness);
    delay(FADE_DELAY);
  }
}
