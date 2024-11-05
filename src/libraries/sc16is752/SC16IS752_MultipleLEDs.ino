/*
 * SC16IS752 Multiple LED Example
 * 
 * Hardware Connections:
 * ESP32    -> SC16IS752
 * GPIO 21  -> SCL
 * GPIO 23  -> SDA
 * 3.3V     -> VCC
 * GND      -> GND
 * 
 * SC16IS752 -> LEDs
 * GPIO0    -> LED1 + (with 220Ω resistor)
 * GPIO1    -> LED2 + (with 220Ω resistor)
 * GPIO2    -> LED3 + (with 220Ω resistor)
 * GPIO3    -> LED4 + (with 220Ω resistor)
 * GND      -> All LED -
 * 
 * Note: Each LED should have its own current limiting resistor (220Ω recommended)
 */

#include <Wire.h>
#include "SC16IS752.h"

// I2C Configuration
#define I2C_ADDRESS 0x4D
#define SDA_PIN     23
#define SCL_PIN     21

// LED Configuration
#define NUM_LEDS    4
const uint8_t LED_PINS[NUM_LEDS] = {GPIO_0, GPIO_1, GPIO_2, GPIO_3};

// Timing Configuration
#define PATTERN_DURATION  10000  // Duration for each pattern (ms)
#define FADE_STEP_DELAY  15     // Delay between fade steps (ms)
#define CHASE_DELAY      100    // Delay for chase pattern (ms)
#define BLINK_DELAY      500    // Delay for blink pattern (ms)

// Create SC16IS752 instance
SC16IS752 gpio(Wire);

// Pattern tracking
enum Pattern {
    BINARY_COUNT,
    CHASE,
    ALL_FADE,
    ALTERNATE_BLINK,
    WAVE
};

Pattern currentPattern = BINARY_COUNT;
unsigned long patternStartTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("\nSC16IS752 Multiple LED Example");

    // Initialize I2C
    if (!gpio.begin(I2C_ADDRESS)) {
        Serial.println("Failed to initialize SC16IS752!");
        while (1) delay(100);
    }
    
    // Initialize all LED pins as outputs
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio.pinMode(LED_PINS[i], GPIO_OUTPUT);
        gpio.digitalWrite(LED_PINS[i], GPIO_LOW);
    }

    Serial.println("Initialization complete");
    printDeviceInfo();
}

void loop() {
    // Change pattern every PATTERN_DURATION milliseconds
    if (millis() - patternStartTime >= PATTERN_DURATION) {
        currentPattern = static_cast<Pattern>((currentPattern + 1) % 5);
        patternStartTime = millis();
        
        // Reset all LEDs when changing patterns
        allLEDs(GPIO_LOW);
        
        // Initialize new pattern
        switch (currentPattern) {
            case BINARY_COUNT:
                Serial.println("Pattern: Binary Count");
                break;
            case CHASE:
                Serial.println("Pattern: Chase");
                break;
            case ALL_FADE:
                Serial.println("Pattern: All Fade");
                initializePWM();
                break;
            case ALTERNATE_BLINK:
                Serial.println("Pattern: Alternate Blink");
                break;
            case WAVE:
                Serial.println("Pattern: Wave");
                initializePWM();
                break;
        }
    }

    // Execute current pattern
    switch (currentPattern) {
        case BINARY_COUNT:
            binaryCountPattern();
            break;
        case CHASE:
            chasePattern();
            break;
        case ALL_FADE:
            allFadePattern();
            break;
        case ALTERNATE_BLINK:
            alternateBlinkPattern();
            break;
        case WAVE:
            wavePattern();
            break;
    }
}

// Initialize PWM for all LEDs
void initializePWM() {
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio.pwmBegin(LED_PINS[i]);
    }
}

// Set all LEDs to same state
void allLEDs(uint8_t state) {
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio.digitalWrite(LED_PINS[i], state);
    }
}

// Pattern Implementations
void binaryCountPattern() {
    static unsigned long lastChange = 0;
    static uint8_t count = 0;
    
    if (millis() - lastChange >= BLINK_DELAY) {
        for (int i = 0; i < NUM_LEDS; i++) {
            gpio.digitalWrite(LED_PINS[i], (count & (1 << i)) ? GPIO_HIGH : GPIO_LOW);
        }
        count = (count + 1) % (1 << NUM_LEDS);
        lastChange = millis();
    }
}

void chasePattern() {
    static unsigned long lastChange = 0;
    static int currentLED = 0;
    
    if (millis() - lastChange >= CHASE_DELAY) {
        allLEDs(GPIO_LOW);
        gpio.digitalWrite(LED_PINS[currentLED], GPIO_HIGH);
        currentLED = (currentLED + 1) % NUM_LEDS;
        lastChange = millis();
    }
}

void allFadePattern() {
    static unsigned long lastChange = 0;
    static uint8_t brightness = 0;
    static bool increasing = true;
    
    if (millis() - lastChange >= FADE_STEP_DELAY) {
        for (int i = 0; i < NUM_LEDS; i++) {
            gpio.pwmWrite(LED_PINS[i], brightness);
        }
        
        if (increasing) {
            brightness++;
            if (brightness == 255) increasing = false;
        } else {
            brightness--;
            if (brightness == 0) increasing = true;
        }
        
        lastChange = millis();
    }
}

void alternateBlinkPattern() {
    static unsigned long lastChange = 0;
    static bool state = false;
    
    if (millis() - lastChange >= BLINK_DELAY) {
        for (int i = 0; i < NUM_LEDS; i++) {
            gpio.digitalWrite(LED_PINS[i], ((i % 2) == state) ? GPIO_HIGH : GPIO_LOW);
        }
        state = !state;
        lastChange = millis();
    }
}

void wavePattern() {
    static unsigned long lastChange = 0;
    static int phase = 0;
    
    if (millis() - lastChange >= FADE_STEP_DELAY) {
        for (int i = 0; i < NUM_LEDS; i++) {
            // Create a sine wave pattern with phase offset for each LED
            int brightness = 127 + (127 * sin((phase + (i * 90)) * PI / 180.0));
            gpio.pwmWrite(LED_PINS[i], brightness);
        }
        
        phase = (phase + 5) % 360;
        lastChange = millis();
    }
}

// Debug helper functions
void printDeviceInfo() {
    Serial.println("\nDevice Configuration:");
    Serial.print("I2C Address: 0x");
    Serial.println(I2C_ADDRESS, HEX);
    Serial.print("SDA Pin: ");
    Serial.println(SDA_PIN);
    Serial.print("SCL Pin: ");
    Serial.println(SCL_PIN);
    Serial.println("\nLED Pins:");
    for (int i = 0; i < NUM_LEDS; i++) {
        Serial.print("LED");
        Serial.print(i + 1);
        Serial.print(": GPIO");
        Serial.println(LED_PINS[i]);
    }
}

