#include "CommManager.h"
#include "ServoController.h"
#include "ServoHandler.h"
#include <Arduino.h>

ServoHandler* servoHandler;

// Command queue and comm manager
CommManager* comm;
QueueHandle_t commandQueue;  // Queue to hold commands
QueueHandle_t responseQueue;  // Queue to hold responses

const int CMD_MAX_LEN = 128;  // Max command string length

void communicationTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial communication to stabilize

    comm = new CommManager(CommManager::WIFI);
    comm->begin();

    // Create a FIFO queue to hold commands (with space for 10 commands)
    commandQueue = xQueueCreate(10, CMD_MAX_LEN);  // Now stores char arrays
    responseQueue = xQueueCreate(10, CMD_MAX_LEN);  // Now stores char arrays

    // Create a task for communication handling (Core 0)
    xTaskCreatePinnedToCore(
        communicationTask,  // Function handling communication
        "CommTask",         // Task name
        4096,               // Stack size
        NULL,               // Task parameters
        1,                  // Task priority
        NULL,               // Task handle
        0                   // Run on Core 0
    );

    ServoController* servoController = new ServoController();
    servoController->begin();  // Initialize the servo controller
    servoHandler = new ServoHandler(*servoController);  // Create the servo handler with the controller
}

void loop() {
    unsigned long loopStartTime = millis();  // Start time

    // Check if there are commands in the queue
    char cmdBuffer[CMD_MAX_LEN];
    if (xQueueReceive(commandQueue, cmdBuffer, 0) == pdTRUE) {
        String cmd(cmdBuffer);
        String response = servoHandler->handleSrvCommand(cmd);
        if (xQueueSend(responseQueue, response.c_str(), 0) != pdPASS) {
            Serial.println("Failed to send response.");
        }
    }

    servoHandler->servoController.update();  // Update servo controller

    unsigned long loopEndTime = millis();
    unsigned long loopDuration = loopEndTime - loopStartTime;
    unsigned long delayTime = 10 - loopDuration;

    if (delayTime > 0) {
        delay(delayTime);
    }
}

void communicationTask(void *pvParameters) {
    while (true) {
        if (comm->available()) {
            String incomingCmd = comm->read();
            //Serial.println("Command received: " + incomingCmd);

            char cmdBuffer[CMD_MAX_LEN];
            incomingCmd.toCharArray(cmdBuffer, CMD_MAX_LEN);

            if (xQueueSend(commandQueue, cmdBuffer, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to add command to queue.");
            }
        }

        delay(10);
        
        char resBuffer[CMD_MAX_LEN];
        if (xQueueReceive(responseQueue, resBuffer, 0) == pdTRUE) {
            String response(resBuffer);
            comm->send(response);  // Send the response back
        }
    }
}
