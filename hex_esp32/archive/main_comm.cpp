#include "CommManager.h"

// Command queue and comm manager
CommManager* comm;
QueueHandle_t commandQueue;  // Queue to hold commands

void communicationTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial communication to stabilize

    comm = new CommManager(CommManager::UART);
    comm->begin();

    // Create a FIFO queue to hold commands (with space for 10 commands)
    commandQueue = xQueueCreate(10, sizeof(String));

    // Create a task for communication handling (Core 0)
    xTaskCreatePinnedToCore(
        communicationTask,  // Function handling communication
        "CommTask",         // Task name
        4096,               // Stack size
        NULL,               // Task parameters (not used here)
        1,                  // Task priority
        NULL,               // Task handle (not used here)
        0                   // Run on Core 0
    );
}

void loop() {
    // This loop will handle control and movement
    unsigned long loopStartTime = millis();  // Start time to calculate loop duration

    // Check if there are commands in the queue
    String cmd;
    if (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {  // Non-blocking read
        // Process command here
        if (cmd == "start") {
            //Serial.println("Starting movement...");
            // Add logic to start movement
        } else if (cmd == "stop") {
            //Serial.println("Stopping movement...");
            // Add logic to stop movement
        } else {
            //Serial.println("Unknown command");
        }
    }

    // Control movement or other hardware tasks here (Example):
    // moveServo();  // Function that moves the servo based on some logic

    unsigned long loopEndTime = millis();  // End time to calculate loop duration
    unsigned long loopDuration = loopEndTime - loopStartTime;
    unsigned long delayTime = 100 - loopDuration;  // Calculate remaining time for 100ms delay

    // Ensure we don't have negative delays
    if (delayTime > 0) {
        delay(delayTime);  // Dynamic delay based on loop execution time
    }
}

void communicationTask(void *pvParameters) {
    // This task handles communication on Core 0

    while (true) {
        if (comm->available()) {
            String cmd = comm->read();  // Read incoming command
            //Serial.println("Command received: " + cmd);

            // Add the received command to the queue
            if (xQueueSend(commandQueue, &cmd, portMAX_DELAY) != pdPASS) {
                //Serial.println("Failed to add command to queue.");
            }
        }

        delay(10);  // Small delay to avoid overloading the CPU
    }
}
