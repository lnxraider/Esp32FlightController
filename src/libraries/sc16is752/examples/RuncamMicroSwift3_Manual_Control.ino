// RuncamMicroSwift3_Control.ino

#include "SC16IS752.h"
#include "SC16IS752_UART.h"

// Configuration (from SC16IS752_UART_Examples.ino)
const uint8_t I2C_ADDRESS = 0x4D;
const int SDA_PIN = 23;
const int SCL_PIN = 21;
const uint32_t UART_BAUD = 115200;

// Runcam protocol commands
const uint8_t RCAM_ENTER = 0x80;
const uint8_t RCAM_LEFT = 0x81;
const uint8_t RCAM_UP = 0x82;
const uint8_t RCAM_RIGHT = 0x83;
const uint8_t RCAM_DOWN = 0x84;
const uint8_t RCAM_OPEN_MENU = 0x85;
const uint8_t RCAM_GET_SETTINGS = 0x86;
const uint8_t RCAM_SET_SETTINGS = 0x87;

// Initialize UART object
SC16IS752_UART uart(I2C_ADDRESS, Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nRuncam Micro Swift 3 Control");

    if (!uart.begin(UART_BAUD)) {
        Serial.println("UART initialization failed!");
        while(1) delay(100);
    }

    // Configure UART channel for Runcam (using Channel A)
    if (!uart.beginUART(SC16IS752::CHANNEL_A, UART_BAUD, SC16IS752_UART::UART_8N1)) {
        Serial.println("Failed to configure UART channel!");
        while(1) delay(100);
    }

    // Initial delay to allow camera to boot
    delay(500);
    
    Serial.println("Runcam control ready!");
    printHelp();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }

    // Check for any responses from camera
    while (uart.available(SC16IS752::CHANNEL_A)) {
        int data = uart.read(SC16IS752::CHANNEL_A);
        if (data != -1) {
            Serial.printf("Camera response: 0x%02X\n", data);
        }
    }
}

void handleCommand(char cmd) {
    switch (cmd) {
        case 'm': // Open menu
            sendCameraCommand(RCAM_OPEN_MENU);
            Serial.println("Opening menu");
            break;

        case 'e': // Enter
            sendCameraCommand(RCAM_ENTER);
            Serial.println("Enter pressed");
            break;

        case 'u': // Up
            sendCameraCommand(RCAM_UP);
            Serial.println("Up pressed");
            break;

        case 'd': // Down
            sendCameraCommand(RCAM_DOWN);
            Serial.println("Down pressed");
            break;

        case 'l': // Left
            sendCameraCommand(RCAM_LEFT);
            Serial.println("Left pressed");
            break;

        case 'r': // Right
            sendCameraCommand(RCAM_RIGHT);
            Serial.println("Right pressed");
            break;

        case 's': // Get status
            printStatus();
            break;

        case 'h': // Help
            printHelp();
            break;

        default:
            Serial.println("Unknown command!");
            break;
    }
}

bool sendCameraCommand(uint8_t command) {
    uint8_t packet[] = {0xCC, command, 0x00};
    size_t written = uart.write(SC16IS752::CHANNEL_A, packet, sizeof(packet));
    
    if (written != sizeof(packet)) {
        Serial.println("Failed to send command!");
        return false;
    }
    
    // Allow time for camera to process command
    delay(50);
    return true;
}

void printStatus() {
    Serial.println("\nUART Status:");
    SC16IS752_UART::UARTStatus status = uart.getStatus(SC16IS752::CHANNEL_A);
    
    Serial.printf("RX FIFO Level: %d\n", status.rxlvl);
    Serial.printf("TX FIFO Level: %d\n", status.txlvl);
    
    if (status.error != SC16IS752_UART::UARTError::NONE) {
        Serial.printf("Error: %d\n", status.error);
    }

    // Print transfer statistics
    SC16IS752_UART::TransferStats stats = uart.getTransferStats(SC16IS752::CHANNEL_A);
    Serial.printf("Bytes Sent: %lu\n", stats.bytesSent);
    Serial.printf("Bytes Received: %lu\n", stats.bytesReceived);
    Serial.printf("Errors: %lu\n", stats.errors);
}

void printHelp() {
    Serial.println("\nRuncam Micro Swift 3 Control Commands:");
    Serial.println("m - Open menu");
    Serial.println("e - Enter");
    Serial.println("u - Up");
    Serial.println("d - Down");
    Serial.println("l - Left");
    Serial.println("r - Right");
    Serial.println("s - Print status");
    Serial.println("h - Show this help");
}


