#include "CommManager.h"
#include "ServoController.h"
#include "ServoHandler.h"
#include "Consts.h"
#include <Arduino.h>

ServoHandler* servoHandler;

// Command queue and comm manager
CommManager* comm;
QueueHandle_t commandQueue;  // Queue to hold commands
QueueHandle_t responseQueue;  // Queue to hold responses

void communicationTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial communication to stabilize

    comm = new CommManager(CommManager::BLUETOOTH);
    comm->begin();

    // Create a FIFO queue to hold commands (with space for 10 commands)
    commandQueue = xQueueCreate(10, __CMD_MAX_LEN__);  // Now stores char arrays
    responseQueue = xQueueCreate(10, __CMD_MAX_LEN__);  // Now stores char arrays

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

    int count = 0;

    // Check if there are commands in the queue
    char cmdBuffer[__CMD_MAX_LEN__];
    if (xQueueReceive(commandQueue, cmdBuffer, 0) == pdTRUE) {
        Serial.println("Command received: " + String(cmdBuffer));
        String cmd(cmdBuffer);
        String response = servoHandler->handleSrvCommand(cmd);
        // if (xQueueSend(responseQueue, response.c_str(), 0) != pdPASS) {
        //     Serial.println("Failed to send response.");
        // }
    }
    // else {
    //     Serial.println("No command received, checking for manual PWM...");
    // }

    servoHandler->servoController.update();  // Update servo controller

    unsigned long loopEndTime = millis();
    long loopDuration = loopEndTime - loopStartTime;
    long delayTime = __DELTA_TIME__ - loopDuration;

    // Serial.print("Loop duration: ");
    // Serial.print(loopDuration);
    // Serial.print("ms, Delay time: ");
    // Serial.print(delayTime);
    // Serial.print("ms, loopEndTime: " + String(loopEndTime));
    // Serial.print(", loopStartTime: " + String(loopStartTime));
    // Serial.println(", delta time: " + String(__DELTA_TIME__));

    if (delayTime < 0) {
        // Serial.println("Loop took too long: " + String(loopDuration) + "ms, skipping delay.");
    }
    if (delayTime > 0) {
        // Serial.println("Loop duration: " + String(loopDuration) + "ms, delaying for: " + String(delayTime) + "ms");
        delay(delayTime);
        // Serial.println("Delay complete.");
    }
}

void communicationTask(void *pvParameters) {
    while (true) {
        if (comm->available()) {
            String incomingCmd = comm->read();
            // Serial.println("Command received: " + incomingCmd);

            char cmdBuffer[__CMD_MAX_LEN__];
            incomingCmd.toCharArray(cmdBuffer, __CMD_MAX_LEN__);

            if (xQueueSend(commandQueue, cmdBuffer, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to add command to queue.");
            }
            // Serial.println("Command added to queue: " + String(cmdBuffer));
        }

        delay(__DELTA_TIME__);
        
        char resBuffer[__CMD_MAX_LEN__];
        if (xQueueReceive(responseQueue, resBuffer, 0) == pdTRUE) {
            String response(resBuffer);
            comm->send(response);  // Send the response back
        }
    }
}
