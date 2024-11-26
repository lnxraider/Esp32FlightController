// RuncamMicroSwift3_RC_Control.ino

#include "SC16IS752.h"
#include "SC16IS752_UART.h"

// UART Configuration
const uint8_t I2C_ADDRESS = 0x4D;
const int SDA_PIN = 23;
const int SCL_PIN = 21;
const uint32_t UART_BAUD = 115200;

// RC Input Pins (adjust based on your connections)
const int RC_AUX1_PIN = 32;  // For menu access
const int RC_AUX2_PIN = 33;  // For up/down
const int RC_AUX3_PIN = 34;  // For left/right

// RC Signal thresholds
const int RC_MIN = 1000;    // Minimum RC pulse width (μs)
const int RC_MID = 1500;    // Middle RC pulse width (μs)
const int RC_MAX = 2000;    // Maximum RC pulse width (μs)
const int RC_DEADBAND = 50; // Deadband around center position

// Runcam Commands
const uint8_t RCAM_ENTER = 0x80;
const uint8_t RCAM_LEFT = 0x81;
const uint8_t RCAM_UP = 0x82;
const uint8_t RCAM_RIGHT = 0x83;
const uint8_t RCAM_DOWN = 0x84;
const uint8_t RCAM_OPEN_MENU = 0x85;

// Timing control
const unsigned long COMMAND_DELAY = 200;  // Minimum delay between commands
unsigned long lastCommandTime = 0;

// RC pulse timing variables
volatile unsigned long rcPulseStartTime[3] = {0, 0, 0};
volatile int rcPulseWidth[3] = {0, 0, 0};
volatile bool rcNewData[3] = {false, false, false};

// Initialize UART
SC16IS752_UART uart(I2C_ADDRESS, Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

void IRAM_ATTR rcInterrupt0() {
    static volatile unsigned long startTime = 0;
    if(digitalRead(RC_AUX1_PIN) == HIGH) {
        startTime = micros();
    } else {
        rcPulseWidth[0] = micros() - startTime;
        rcNewData[0] = true;
    }
}

void IRAM_ATTR rcInterrupt1() {
    static volatile unsigned long startTime = 0;
    if(digitalRead(RC_AUX2_PIN) == HIGH) {
        startTime = micros();
    } else {
        rcPulseWidth[1] = micros() - startTime;
        rcNewData[1] = true;
    }
}

void IRAM_ATTR rcInterrupt2() {
    static volatile unsigned long startTime = 0;
    if(digitalRead(RC_AUX3_PIN) == HIGH) {
        startTime = micros();
    } else {
        rcPulseWidth[2] = micros() - startTime;
        rcNewData[2] = true;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nRuncam RC Control System");

    // Initialize UART
    if (!uart.begin(UART_BAUD)) {
        Serial.println("UART initialization failed!");
        while(1) delay(100);
    }

    // Configure UART channel
    if (!uart.beginUART(SC16IS752::CHANNEL_A, UART_BAUD, SC16IS752_UART::UART_8N1)) {
        Serial.println("Failed to configure UART channel!");
        while(1) delay(100);
    }

    // Configure RC input pins
    pinMode(RC_AUX1_PIN, INPUT);
    pinMode(RC_AUX2_PIN, INPUT);
    pinMode(RC_AUX3_PIN, INPUT);

    // Attach interrupts for RC inputs
    attachInterrupt(digitalPinToInterrupt(RC_AUX1_PIN), rcInterrupt0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_AUX2_PIN), rcInterrupt1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RC_AUX3_PIN), rcInterrupt2, CHANGE);

    Serial.println("System ready!");
}

void loop() {
    processRCInputs();
    
    // Check for any responses from camera
    while (uart.available(SC16IS752::CHANNEL_A)) {
        int data = uart.read(SC16IS752::CHANNEL_A);
        if (data != -1) {
            Serial.printf("Camera response: 0x%02X\n", data);
        }
    }
}

void processRCInputs() {
    unsigned long currentTime = millis();
    if (currentTime - lastCommandTime < COMMAND_DELAY) {
        return;
    }

    // Process Menu Switch (AUX1)
    if (rcNewData[0]) {
        rcNewData[0] = false;
        if (isValidPulse(rcPulseWidth[0])) {
            if (rcPulseWidth[0] > RC_MAX - 200) {  // High position
                sendCameraCommand(RCAM_OPEN_MENU);
                Serial.println("Menu Toggle");
            }
        }
    }

    // Process Up/Down (AUX2)
    if (rcNewData[1]) {
        rcNewData[1] = false;
        if (isValidPulse(rcPulseWidth[1])) {
            if (rcPulseWidth[1] > RC_MAX - RC_DEADBAND) {
                sendCameraCommand(RCAM_UP);
                Serial.println("Up");
            } else if (rcPulseWidth[1] < RC_MIN + RC_DEADBAND) {
                sendCameraCommand(RCAM_DOWN);
                Serial.println("Down");
            }
        }
    }

    // Process Left/Right (AUX3)
    if (rcNewData[2]) {
        rcNewData[2] = false;
        if (isValidPulse(rcPulseWidth[2])) {
            if (rcPulseWidth[2] > RC_MAX - RC_DEADBAND) {
                sendCameraCommand(RCAM_RIGHT);
                Serial.println("Right");
            } else if (rcPulseWidth[2] < RC_MIN + RC_DEADBAND) {
                sendCameraCommand(RCAM_LEFT);
                Serial.println("Left");
            }
        }
    }
}

bool isValidPulse(int pulseWidth) {
    return (pulseWidth >= RC_MIN && pulseWidth <= RC_MAX);
}

bool sendCameraCommand(uint8_t command) {
    uint8_t packet[] = {0xCC, command, 0x00};
    size_t written = uart.write(SC16IS752::CHANNEL_A, packet, sizeof(packet));
    
    if (written != sizeof(packet)) {
        Serial.println("Failed to send command!");
        return false;
    }
    
    lastCommandTime = millis();
    return true;
}

