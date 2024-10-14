#include <Arduino.h>
#include "Drone.h"
#include <esp_task_wdt.h>

// Create global instances of classes
Drone drone;

// FreeRTOS task handles
TaskHandle_t flightControlTaskHandle;
TaskHandle_t communicationTaskHandle;

void setup() {
    Serial.begin(115200);

    // Initialize the drone components
    if (!drone.initialize()) {
        Serial.println("Drone initialization failed!");
        while (1);
    }

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        flightControlTask,      // Task function
        "FlightControlTask",    // Name
        8192,                   // Stack size
        NULL,                   // Parameters
        2,                      // Priority (higher)
        &flightControlTaskHandle,// Task handle
        1                       // Core
    );

    xTaskCreatePinnedToCore(
        communicationTask,      // Task function
        "CommunicationTask",    // Name
        8192,                   // Stack size
        NULL,                   // Parameters
        1,                      // Priority (lower)
        &communicationTaskHandle,// Task handle
        1                       // Core
    );

    // Implement a watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
}

void loop() {
    esp_task_wdt_reset();

    // Empty loop; all tasks are handled by FreeRTOS
}

void flightControlTask(void *pvParameters) {
    while (true) {
        drone.updateFlightControl();
        vTaskDelay(20 / portTICK_PERIOD_MS);  // Control loop delay (50Hz)
    }
}

void communicationTask(void *pvParameters) {
    while (true) {
        drone.updateCommunication();
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Communication task delay (20Hz)
    }
}

