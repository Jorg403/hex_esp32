#include <Arduino.h>
#include "ServoHandler.h"

ServoHandler::ServoHandler(ServoController& controller)
    : servoController(controller) {}

short int __PRINT_ALTERNATION__ = 0;
String ServoHandler::handleSrvCommand(const String& command) {
    int firstSpace = command.indexOf(' ');
    String action = (firstSpace == -1) ? command : command.substring(0, firstSpace);
    String params = (firstSpace == -1) ? "" : command.substring(firstSpace + 1);

    if (action == "set_positions") {
        setPositions(params);
    } else if (action == "set_position") {
        int servoId, position;
        if (sscanf(params.c_str(), "%d %d", &servoId, &position) == 2) {
            servoController.setServo(servoId, position);
        }
    } else if (action == "set_manualPWM") {
        int servoId, pwmValue;
        if (sscanf(params.c_str(), "%d %d", &servoId, &pwmValue) == 2) {
            servoController.setManualPWM(servoId, pwmValue);
        }
    } else if (action == "enable") {
        int servoId;
        if (sscanf(params.c_str(), "%d", &servoId) == 1) {
            servoController.enableServo(servoId);
        }
    } else if (action == "disable") {
        int servoId;
        if (sscanf(params.c_str(), "%d", &servoId) == 1) {
            servoController.disableServo(servoId);
        }
    } else if (action == "set_speed") {
        int servoId, speed;
        if (sscanf(params.c_str(), "%d %d", &servoId, &speed) == 2) {
            servoController.setSpeed(servoId, speed);
        }
    } else if (action == "print_status") {
        return servoController.getStatus();
    } else if (action == "stop") {
        stopMovements();
    } else {
        return "Unknown command: " + action;
    }
    __PRINT_ALTERNATION__ = (__PRINT_ALTERNATION__ + 1) % 6;
    // returns success with |-,\-,-\,-|,-/ or /- depending on value of __PRINT_ALTERNATION__
    return String(__PRINT_ALTERNATION__ == 0 ? "|-" : (__PRINT_ALTERNATION__ == 1 ? "\\-" : (__PRINT_ALTERNATION__ == 2 ? "-\\" : (__PRINT_ALTERNATION__ == 3 ? "-|" : (__PRINT_ALTERNATION__ == 4 ? "-/" : "/-"))))) + "Success";
}

void ServoHandler::setPositions(const String& params) {
    if (params.length() > 256) {
        //Serial.println("Params too long!");
        return;
    }

    char buffer[params.length() + 1];
    params.toCharArray(buffer, sizeof(buffer));

    char* token = strtok(buffer, " ");
    while (token != NULL) {
        int servoId = atoi(token);
        token = strtok(NULL, " ");
        if (token == NULL) break;
        int position = atoi(token);
        servoController.setServo(servoId, position);
        token = strtok(NULL, " ");
    }
}

void ServoHandler::stopMovements() {
    servoController.setSpeeds({0, 0});
}
