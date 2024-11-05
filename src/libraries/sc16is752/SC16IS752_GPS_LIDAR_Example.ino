/*
 * SC16IS752 Dual UART Example - GPS and LiDAR
 * 
 * Hardware Connections:
 * ESP32    -> SC16IS752
 * GPIO 21  -> SCL
 * GPIO 23  -> SDA
 * 3.3V     -> VCC
 * GND      -> GND
 * 
 * SC16IS752 UART A -> GPS Module (NEO-6M)
 * TX1      -> GPS RX
 * RX1      -> GPS TX
 * GND      -> GPS GND
 * 3.3V     -> GPS VCC
 * 
 * SC16IS752 UART B -> LiDAR Module (TFmini)
 * TX2      -> LiDAR RX
 * RX2      -> LiDAR TX
 * GND      -> LiDAR GND
 * 5V       -> LiDAR VCC (Note: Most LiDAR modules need 5V)
 */

#include <Wire.h>
#include "SC16IS752.h"

// I2C Configuration
#define I2C_ADDRESS 0x4D
#define SDA_PIN     23
#define SCL_PIN     21

// UART Configuration
#define GPS_BAUD    9600    // Common baud rate for GPS modules
#define LIDAR_BAUD  115200  // Common baud rate for TFmini

// Buffer sizes
#define GPS_BUFFER_SIZE   128
#define LIDAR_BUFFER_SIZE 32

// Create SC16IS752 instance
SC16IS752 uart(Wire);

// Buffers for received data
uint8_t gpsBuffer[GPS_BUFFER_SIZE];
uint8_t lidarBuffer[LIDAR_BUFFER_SIZE];

// GPS parsing state
struct GPSData {
    float latitude;
    float longitude;
    float altitude;
    int satellites;
    bool valid;
    char timeUTC[10];
    char dateUTC[10];
} gpsData;

// LiDAR data structure
struct LiDARData {
    uint16_t distance;  // Distance in cm
    uint16_t strength;  // Signal strength
    uint8_t temp;      // Temperature
    bool valid;
} lidarData;

void setup() {
    // Initialize debug serial port
    Serial.begin(115200);
    Serial.println("\nSC16IS752 GPS and LiDAR Example");

    // Initialize I2C and SC16IS752
    if (!uart.begin(I2C_ADDRESS)) {
        Serial.println("Failed to initialize SC16IS752!");
        while (1) delay(100);
    }

    // Initialize UART channels
    // Channel A - GPS
    if (uart.initializeUART(uart.CHANNEL_A, GPS_BAUD) != uart.OK) {
        Serial.println("Failed to initialize UART Channel A (GPS)!");
        while (1) delay(100);
    }

    // Channel B - LiDAR
    if (uart.initializeUART(uart.CHANNEL_B, LIDAR_BAUD) != uart.OK) {
        Serial.println("Failed to initialize UART Channel B (LiDAR)!");
        while (1) delay(100);
    }

    Serial.println("Initialization complete");
    Serial.println("Waiting for GPS and LiDAR data...\n");
}

void loop() {
    // Read and process GPS data
    readGPSData();
    
    // Read and process LiDAR data
    readLiDARData();
    
    // Display processed data
    displayData();
    
    // Small delay to prevent overwhelming serial output
    delay(100);
}

void readGPSData() {
    static String nmeaLine;
    
    while (uart.isRxAvailable(uart.CHANNEL_A)) {
        int byte = uart.readByte(uart.CHANNEL_A);
        if (byte < 0) continue;
        
        char c = (char)byte;
        
        if (c == '\n') {
            parseNMEA(nmeaLine);
            nmeaLine = "";
        } else if (c != '\r') {
            nmeaLine += c;
        }
    }
}

void parseNMEA(String &line) {
    if (line.startsWith("$GPGGA")) {
        // Parse Global Positioning System Fix Data
        int commaPositions[15];
        int commaCount = 0;
        
        // Find positions of commas
        for (int i = 0; i < line.length() && commaCount < 15; i++) {
            if (line.charAt(i) == ',') {
                commaPositions[commaCount++] = i;
            }
        }
        
        if (commaCount >= 9) {
            // Extract latitude
            String latStr = line.substring(commaPositions[1] + 1, commaPositions[2]);
            String latHemi = line.substring(commaPositions[2] + 1, commaPositions[3]);
            if (latStr.length() > 0) {
                gpsData.latitude = latStr.substring(0, 2).toFloat() + 
                                 latStr.substring(2).toFloat() / 60.0;
                if (latHemi == "S") gpsData.latitude = -gpsData.latitude;
            }
            
            // Extract longitude
            String lonStr = line.substring(commaPositions[3] + 1, commaPositions[4]);
            String lonHemi = line.substring(commaPositions[4] + 1, commaPositions[5]);
            if (lonStr.length() > 0) {
                gpsData.longitude = lonStr.substring(0, 3).toFloat() + 
                                  lonStr.substring(3).toFloat() / 60.0;
                if (lonHemi == "W") gpsData.longitude = -gpsData.longitude;
            }
            
            // Extract number of satellites
            String satStr = line.substring(commaPositions[6] + 1, commaPositions[7]);
            if (satStr.length() > 0) {
                gpsData.satellites = satStr.toInt();
            }
            
            // Extract altitude
            String altStr = line.substring(commaPositions[8] + 1, commaPositions[9]);
            if (altStr.length() > 0) {
                gpsData.altitude = altStr.toFloat();
            }
            
            gpsData.valid = true;
        }
    }
}

void readLiDARData() {
    static uint8_t lidarState = 0;
    static uint8_t checksum = 0;
    static uint8_t data[9];  // TFmini data frame is 9 bytes
    
    while (uart.isRxAvailable(uart.CHANNEL_B)) {
        int byte = uart.readByte(uart.CHANNEL_B);
        if (byte < 0) continue;
        
        switch (lidarState) {
            case 0: // Header 1
                if (byte == 0x59) {
                    data[0] = byte;
                    checksum = byte;
                    lidarState = 1;
                }
                break;
                
            case 1: // Header 2
                if (byte == 0x59) {
                    data[1] = byte;
                    checksum += byte;
                    lidarState = 2;
                } else {
                    lidarState = 0;
                }
                break;
                
            case 2: // Data bytes
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
                data[lidarState] = byte;
                checksum += byte;
                lidarState++;
                break;
                
            case 8: // Checksum
                if (checksum == byte) {
                    // Process valid frame
                    lidarData.distance = (data[3] << 8) | data[2];
                    lidarData.strength = (data[5] << 8) | data[4];
                    lidarData.temp = data[6];
                    lidarData.valid = true;
                }
                lidarState = 0;
                break;
        }
    }
}

void displayData() {
    static unsigned long lastDisplay = 0;
    const unsigned long DISPLAY_INTERVAL = 1000; // Update display every second
    
    if (millis() - lastDisplay >= DISPLAY_INTERVAL) {
        Serial.println("\n=== Sensor Data Update ===");
        
        // GPS Data
        if (gpsData.valid) {
            Serial.println("GPS Data:");
            Serial.print("  Latitude:    "); Serial.print(gpsData.latitude, 6); Serial.println("°");
            Serial.print("  Longitude:   "); Serial.print(gpsData.longitude, 6); Serial.println("°");
            Serial.print("  Altitude:    "); Serial.print(gpsData.altitude); Serial.println(" m");
            Serial.print("  Satellites:  "); Serial.println(gpsData.satellites);
        } else {
            Serial.println("GPS: No valid data");
        }
        
        // LiDAR Data
        if (lidarData.valid) {
            Serial.println("LiDAR Data:");
            Serial.print("  Distance:    "); Serial.print(lidarData.distance); Serial.println(" cm");
            Serial.print("  Strength:    "); Serial.println(lidarData.strength);
            Serial.print("  Temperature: "); Serial.print(lidarData.temp); Serial.println("°C");
        } else {
            Serial.println("LiDAR: No valid data");
        }
        
        Serial.println("========================");
        
        lastDisplay = millis();
    }
}

// Utility function to check UART status
void printUARTStatus(uint8_t channel) {
    SC16IS752::UARTStatus status = uart.getStatus(channel);
    
    Serial.print("UART Channel ");
    Serial.print(channel);
    Serial.println(" Status:");
    
    Serial.print("LSR: 0x");
    Serial.println(status.lsr, HEX);
    Serial.print("TX FIFO: ");
    Serial.println(status.txlvl);
    Serial.print("RX FIFO: ");
    Serial.println(status.rxlvl);
    
    if (status.error != uart.OK) {
        Serial.print("Error: ");
        Serial.println(status.error);
    }
}

