// SC16IS752_GPIO_LEDTest.ino
#include "SC16IS752.h"
#include "SC16IS752_GPIO.h"

// Configuration
const uint8_t I2C_ADDRESS = 0x4D;
const int SDA_PIN = 23;
const int SCL_PIN = 21;
const uint32_t I2C_FREQ = 100000;

// GPIO Pin Assignments
const uint8_t LED_1 = 0;  // GPIO0
const uint8_t LED_2 = 1;  // GPIO1
const uint8_t LED_3 = 2;  // GPIO2
const uint8_t LED_4 = 3;  // GPIO3

// Test Patterns
const uint8_t NUM_LEDS = 4;
const uint8_t LEDS[NUM_LEDS] = {LED_1, LED_2, LED_3, LED_4};

SC16IS752_GPIO gpio(I2C_ADDRESS, Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

void setup() {
   Serial.begin(115200);
   delay(1000);
   Serial.println("\nSC16IS752 GPIO LED Test");

   if (!gpio.beginGPIO()) {
       Serial.println("GPIO initialization failed!");
       while(1) delay(100);
   }

   // Configure LED pins as outputs
   for (uint8_t i = 0; i < NUM_LEDS; i++) {
       SC16IS752_GPIO::PinConfig config = {
           .mode = SC16IS752_GPIO::GPIO_OUTPUT,
           .interruptMode = SC16IS752_GPIO::INTERRUPT_DISABLED,
           .initialState = false,
           .debounceTime = 0
       };
       
       if (!gpio.configurePin(LEDS[i], config)) {
           Serial.printf("Failed to configure LED %d\n", i + 1);
           while(1) delay(100);
       }
   }

   Serial.println("GPIO initialized successfully");
   printPinStates();
}

void loop() {
   // Different LED test patterns
   runningLight();
   delay(1000);
   
   allOn();
   delay(1000);
   
   allOff();
   delay(1000);
   
   blinkAll();
   delay(1000);
   
   alternatePattern();
   delay(1000);

   // Check for serial commands
   handleSerialCommands();
}

void runningLight() {
   Serial.println("Running Light Pattern");
   for (int i = 0; i < NUM_LEDS * 2; i++) {
       for (int j = 0; j < NUM_LEDS; j++) {
           gpio.digitalWrite(LEDS[j], j == (i % NUM_LEDS));
       }
       delay(200);
   }
}

void allOn() {
   Serial.println("All LEDs On");
   for (uint8_t i = 0; i < NUM_LEDS; i++) {
       gpio.digitalWrite(LEDS[i], HIGH);
   }
   printPinStates();
}

void allOff() {
   Serial.println("All LEDs Off");
   for (uint8_t i = 0; i < NUM_LEDS; i++) {
       gpio.digitalWrite(LEDS[i], LOW);
   }
   printPinStates();
}

void blinkAll() {
   Serial.println("Blink All Pattern");
   for (int i = 0; i < 4; i++) {
       allOn();
       delay(250);
       allOff();
       delay(250);
   }
}

void alternatePattern() {
   Serial.println("Alternate Pattern");
   for (int i = 0; i < 4; i++) {
       // Even LEDs on, odd LEDs off
       for (uint8_t j = 0; j < NUM_LEDS; j++) {
           gpio.digitalWrite(LEDS[j], j % 2 == 0);
       }
       delay(500);
       
       // Odd LEDs on, even LEDs off
       for (uint8_t j = 0; j < NUM_LEDS; j++) {
           gpio.digitalWrite(LEDS[j], j % 2 != 0);
       }
       delay(500);
   }
}

void handleSerialCommands() {
   if (Serial.available()) {
       char cmd = Serial.read();
       switch (cmd) {
           case '1':
           case '2':
           case '3':
           case '4':
               toggleLED(cmd - '1');
               break;
           case 'a':
               allOn();
               break;
           case 'x':
               allOff();
               break;
           case 'r':
               runningLight();
               break;
           case 'b':
               blinkAll();
               break;
           case 's':
               printStatus();
               break;
           default:
               printHelp();
               break;
       }
   }
}

void toggleLED(uint8_t led) {
   if (led < NUM_LEDS) {
       gpio.togglePin(LEDS[led]);
       Serial.printf("Toggled LED %d\n", led + 1);
       printPinStates();
   }
}

void printStatus() {
   Serial.println("\n=== GPIO Status ===");
   
   for (uint8_t i = 0; i < NUM_LEDS; i++) {
       SC16IS752_GPIO::GPIOStats stats = gpio.getGPIOStats(LEDS[i]);
       Serial.printf("LED %d:\n", i + 1);
       Serial.printf("  State: %s\n", stats.currentState ? "ON" : "OFF");
       Serial.printf("  Toggle Count: %lu\n", stats.toggleCount);
       Serial.printf("  Last Change: %lu ms ago\n", 
                    millis() - stats.lastChangeTime);
       Serial.printf("  Stable: %s\n", 
                    gpio.isPinStable(LEDS[i]) ? "Yes" : "No");
   }
}

void printPinStates() {
   Serial.print("LED States: ");
   for (uint8_t i = 0; i < NUM_LEDS; i++) {
       int state = gpio.digitalRead(LEDS[i]);
       Serial.printf("%d:%s ", i + 1, state ? "ON" : "OFF");
   }
   Serial.println();
}

void printHelp() {
   Serial.println("\nAvailable Commands:");
   Serial.println("1-4: Toggle individual LED");
   Serial.println("a: All LEDs on");
   Serial.println("x: All LEDs off");
   Serial.println("r: Running light pattern");
   Serial.println("b: Blink all pattern");
   Serial.println("s: Print status");
}


