// SC16IS752_UART_Examples.ino

#include "SC16IS752.h"
#include "SC16IS752_UART.h"

// Connection Configuration
const uint8_t I2C_ADDRESS = 0x4D;
const int SDA_PIN = 23;
const int SCL_PIN = 21;
const uint32_t UART_BAUD = 115200;
const uint32_t MONITOR_INTERVAL = 1000;  // Status check interval

// Test Data
const char* TEST_MESSAGE = "Hello from Channel A!";
const uint8_t SMALL_PACKET[] = { 0x1A, 0x2B, 0x3C, 0x4D, 0x5E };
const size_t LARGE_PACKET_SIZE = 48;

SC16IS752_UART uart(I2C_ADDRESS, Wire, SC16IS752::I2CPins(SDA_PIN, SCL_PIN));

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nSC16IS752 UART Examples");

  // Initialize UART with crystal config
  SC16IS752::CrystalConfig crystalConfig;
  crystalConfig.frequency = 1843200;
  crystalConfig.useExternalClock = false;
  crystalConfig.usePrescaler = false;

  if (!uart.begin(UART_BAUD)) {
    Serial.println("UART initialization failed!");
    while (1) delay(100);
  }

  // Set crystal configuration
  if (!uart.setCrystalConfig(crystalConfig)) {
    Serial.println("Crystal config failed!");
    while (1) delay(100);
  }

  // Configure both channels
  configureBothChannels();

  // Run the examples
  basicTransferExample();
  //DebugSmallPacketTransferExample();
  packetTransferExample();
  flowControlExample();
  //rs485Example();
}

void loop() {
  static uint32_t lastMonitorTime = 0;

  // Periodic status monitoring
  if (millis() - lastMonitorTime >= MONITOR_INTERVAL) {
    monitorChannels();
    lastMonitorTime = millis();
  }
}

void configureBothChannels() {
  // Configure flow control
  SC16IS752_UART::FlowControlConfig flowConfig;
  flowConfig.mode = SC16IS752_UART::FLOW_RTSCTS;
  flowConfig.recoveryDelay = 5;
  flowConfig.maxRetries = 3;
  flowConfig.timeout = 1000;

  // Apply configuration to both channels
  for (uint8_t channel = 0; channel < 2; channel++) {
    uart.setFlowControl(channel, true);
    uart.setFlowControlConfig(channel, flowConfig);
    uart.setFIFOTriggerLevels(channel, 16, 8);  // RX=16, TX=8
  }

  Serial.println("Channels configured successfully");
}

void basicTransferExample() {
  Serial.println("\n=== Basic Transfer Example ===");

  // Send test message
  size_t written = uart.write(SC16IS752::CHANNEL_A,
                              (uint8_t*)TEST_MESSAGE,
                              strlen(TEST_MESSAGE));
  Serial.printf("Channel A transmitted %d bytes\n", written);

  // Wait for and read response
  delay(50);
  while (uart.available(SC16IS752::CHANNEL_B)) {
    int data = uart.read(SC16IS752::CHANNEL_B);
    if (data != -1) {
      Serial.printf("Channel B received: 0x%02X (%c)\n",
                    data, isprint(data) ? data : '.');
    }
  }

  // Get and print status
  printChannelStatus(SC16IS752::CHANNEL_A);
  printChannelStatus(SC16IS752::CHANNEL_B);
  //uart.resetChannel(SC16IS752::CHANNEL_A);
  //Serial.println("Reset Channel A");
  //uart.resetChannel(SC16IS752::CHANNEL_B);
  //Serial.println("Reset Channel B");
}

void SmallPacketTransferExample() {
  Serial.println("\n=== Packet Transfer Example ===");
  const uint8_t SMALL_PACKET[] = { 0x1A, 0x2B, 0x3C, 0x4D, 0x5E };

  uart.resetChannel(SC16IS752::CHANNEL_A);
  uart.resetChannel(SC16IS752::CHANNEL_B);
  delay(50);

  Serial.print("Sending small packet: ");
  printBuffer(SMALL_PACKET, sizeof(SMALL_PACKET));

  if (!uart.sendPacket(SC16IS752::CHANNEL_A, SMALL_PACKET, sizeof(SMALL_PACKET),
                       SC16IS752_UART::PACKET_SMALL)) {
    Serial.println("Send failed");
    return;
  }
  Serial.println("Small packet sent");

  uint8_t rxBuffer[128];
  size_t rxSize;

  delay(50);

  if (uart.receivePacket(SC16IS752::CHANNEL_B, rxBuffer, rxSize)) {
    Serial.print("Received packet: ");
    printBuffer(rxBuffer, rxSize);
  }
}

void DebugSmallPacketTransferExample() {
  Serial.println("\n=== Packet Transfer Example ===");

  // Small packet test
  const uint8_t SMALL_PACKET[] = { 0x1A, 0x2B, 0x3C, 0x4D, 0x5E };

  // Check channel states before sending
  Serial.println("\nChannel states before transmission:");
  printChannelStatus(SC16IS752::CHANNEL_A);
  printChannelStatus(SC16IS752::CHANNEL_B);

  uart.flush(SC16IS752::CHANNEL_A);
  delay(10);

  Serial.print("Sending small packet: ");
  printBuffer(SMALL_PACKET, sizeof(SMALL_PACKET));

  if (!uart.sendPacket(SC16IS752::CHANNEL_A, SMALL_PACKET, sizeof(SMALL_PACKET),
                       SC16IS752_UART::PACKET_SMALL)) {
    Serial.println("Failed to send small packet");
    printError(SC16IS752::CHANNEL_A);
    return;
  }

  delay(50);  // Allow transmission time

  Serial.println("\nChannel states after transmission:");
  printChannelStatus(SC16IS752::CHANNEL_A);
  printChannelStatus(SC16IS752::CHANNEL_B);

  Serial.println("Small packet sent");

  // Read raw received data
  /*
    Serial.print("Raw received data: ");
    delay(50);  // Allow transmission time
    while (uart.available(SC16IS752::CHANNEL_B)) {
        int data = uart.read(SC16IS752::CHANNEL_B);
        if (data != -1) {
            Serial.printf("%02X ", data);
        }
    }
    Serial.println();
    */
  uint8_t rxBuffer[128];
  size_t rxSize;

  if (!uart.receivePacket(SC16IS752::CHANNEL_B, rxBuffer, rxSize)) {
    Serial.printf("Failed at RX FIFO Level: %d\n", uart.available(SC16IS752::CHANNEL_B));
    Serial.println("Failed to receive packet");
    printError(SC16IS752::CHANNEL_B);
    return;
  }

  Serial.print("Received packet: ");
  printBuffer(rxBuffer, rxSize);
}

void packetTransferExample() {
  Serial.println("\n=== Packet Transfer Example ===");

  // Small packet test
  const uint8_t SMALL_PACKET[] = { 0x1A, 0x2B, 0x3C, 0x4D, 0x5E };
  bool smallResult = uart.sendPacket(SC16IS752::CHANNEL_A,
                                     SMALL_PACKET,
                                     sizeof(SMALL_PACKET),
                                     SC16IS752_UART::PACKET_SMALL);

  delay(20);

  if (smallResult) {
    Serial.println("Small packet sent");
    uint8_t rxBuffer[128];
    size_t rxSize;

    if (uart.receivePacket(SC16IS752::CHANNEL_B, rxBuffer, rxSize)) {
      Serial.printf("Received %d bytes: ", rxSize);
      printBuffer(rxBuffer, rxSize);

      // Validate received data
      if (rxSize == sizeof(SMALL_PACKET) && memcmp(rxBuffer, SMALL_PACKET, rxSize) == 0) {
        Serial.println("Small packet validated successfully");
      }
    }
  }

  delay(100);  // Allow time between transfers
  uart.resetChannel(SC16IS752::CHANNEL_A);
  uart.resetChannel(SC16IS752::CHANNEL_B);

  // Large packet test
  uint8_t largePacket[LARGE_PACKET_SIZE];
  for (size_t i = 0; i < LARGE_PACKET_SIZE; i++) {
    largePacket[i] = i & 0xFF;
  }

  bool largeResult = uart.sendPacket(SC16IS752::CHANNEL_A,
                                     largePacket,
                                     LARGE_PACKET_SIZE,
                                     SC16IS752_UART::PACKET_LARGE);

  delay(20);

  if (largeResult) {
    Serial.println("Large packet sent");
    uint8_t rxBuffer[128];
    size_t rxSize;

    if (uart.receivePacket(SC16IS752::CHANNEL_B, rxBuffer, rxSize)) {
      Serial.printf("Received %d bytes: ", rxSize);
      printBuffer(rxBuffer, rxSize);

      // Validate received data
      if (rxSize == LARGE_PACKET_SIZE && memcmp(rxBuffer, largePacket, rxSize) == 0) {
        Serial.println("Large packet validated successfully");
      }
    }
  }

  // Final cleanup
  delay(100);
  uart.resetChannel(SC16IS752::CHANNEL_A);
  uart.resetChannel(SC16IS752::CHANNEL_B);
}

void flowControlExample() {
  Serial.println("\n=== Flow Control Example ===");

  const size_t BURST_SIZE = 256;
  uint8_t burstData[BURST_SIZE];
  for (size_t i = 0; i < BURST_SIZE; i++) {
    burstData[i] = i & 0xFF;
  }

  // Send data in chunks with flow control
  size_t totalSent = 0;
  while (totalSent < BURST_SIZE) {
    if (uart.waitForTxReady(SC16IS752::CHANNEL_A)) {
      size_t remaining = BURST_SIZE - totalSent;
      size_t toSend = min(remaining, (size_t)32);

      size_t sent = uart.write(SC16IS752::CHANNEL_A,
                               &burstData[totalSent],
                               toSend);
      if (sent > 0) {
        totalSent += sent;
        Serial.printf("Sent %d bytes (total: %d/%d)\n",
                      sent, totalSent, BURST_SIZE);
      }
    }

    // Read any available data
    while (uart.available(SC16IS752::CHANNEL_B)) {
      uart.read(SC16IS752::CHANNEL_B);
    }
  }

  // Print transfer statistics
  SC16IS752_UART::TransferStats stats = uart.getTransferStats(SC16IS752::CHANNEL_A);
  printTransferStats(stats);
  stats = uart.getTransferStats(SC16IS752::CHANNEL_B);
  printTransferStats(stats);

  // Add cleanup
  delay(100);
  for (uint8_t channel = 0; channel < 2; channel++) {
    // Drain any remaining data
    uart.resetChannel(channel);
  }
}

void rs485Example() {
  Serial.println("\n=== RS-485 Example ===");

  // Enable RS-485 on Channel A
  uart.enableRS485(SC16IS752::CHANNEL_A, true);

  // Send addressed data packet
  uint8_t rs485Data[] = { 0x01, 0xFF, 0x55, 0xAA };  // Address 0x01
  if (uart.sendPacket(SC16IS752::CHANNEL_A,
                      rs485Data,
                      sizeof(rs485Data),
                      SC16IS752_UART::PACKET_CONTROL)) {
    Serial.println("RS-485 packet sent");
  }

  delay(100);
  uart.enableRS485(SC16IS752::CHANNEL_A, false);
}

void monitorChannels() {
  for (uint8_t channel = 0; channel < 2; channel++) {
    // Clear any pending data using public methods
    while (uart.available(channel)) {
      uart.read(channel);
    }
    uart.flush(channel);

    Serial.printf("\nChannel %c Status:\n", channel ? 'B' : 'A');
    SC16IS752_UART::UARTStatus status = uart.getStatus(channel);
    SC16IS752_UART::TransferStats stats = uart.getTransferStats(channel);

    printChannelStatus(channel);
    printTransferStats(stats);
  }
}

void printError(uint8_t channel) {
  SC16IS752_UART::UARTError error = uart.getLastUARTError(channel);
  if (error != SC16IS752_UART::NONE) {
    Serial.printf("Channel %c Error: %d\n", channel ? 'B' : 'A', error);
  }
}

void printChannelStatus(uint8_t channel) {
  SC16IS752_UART::UARTStatus status = uart.getStatus(channel);

  Serial.printf("Channel %c:\n", channel ? 'B' : 'A');
  Serial.printf("  TX FIFO: %d bytes free\n", status.txlvl);
  Serial.printf("  RX FIFO: %d bytes available\n", status.rxlvl);

  if (status.error != SC16IS752_UART::UARTError::NONE) {
    Serial.printf("  Error: %d\n", status.error);
  }
}

void printTransferStats(const SC16IS752_UART::TransferStats& stats) {
  Serial.println("Transfer Statistics:");
  Serial.printf("  Bytes TX/RX: %lu/%lu\n", stats.bytesSent, stats.bytesReceived);
  Serial.printf("  Transfers (OK/Fail): %lu/%lu\n",
                stats.successfulTransfers, stats.failedTransfers);
  Serial.printf("  Average Transfer Time: %.2f ms\n", stats.averageTransferTime);
  Serial.printf("  Peak Transfer Rate: %.2f bytes/sec\n", stats.peakTransferRate);

  if (stats.errors > 0) {
    Serial.printf("  Errors: %lu (CRC: %lu)\n", stats.errors, stats.crcErrors);
    Serial.printf("  Buffer Overruns/Underruns: %lu/%lu\n",
                  stats.bufferOverruns, stats.bufferUnderruns);
  }
}

void printBuffer(const uint8_t* buffer, size_t size) {
  for (size_t i = 0; i < size; i++) {
    Serial.printf("%02X ", buffer[i]);
    if ((i + 1) % 16 == 0) Serial.println();
  }
  Serial.println();
}
