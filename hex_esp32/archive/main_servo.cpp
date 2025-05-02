#include <Arduino.h>
#include "ServoController.h"

String _command;
ServoController _servos;

void handleCommand(String cmd);
void readCommand();
void update();

void setup() {
    Serial.begin(115200);

    _servos.begin();
    _servos.printInstructions(Serial);

    _command = "";
}

void loop() {
    readCommand();
    update();
}

void handleCommand(String cmd) {
  _servos.handleCommand(cmd);
}

void readCommand() {
    if (Serial.available()) {
        char c = Serial.read();
        //Serial.print(c);
        if (c == '\n') {
            _command.trim();
            //Serial.println("Received: " + _command);
            handleCommand(_command);
            _command = "";
        } else {
            _command += c;
        }
    }
}

void update() {
    _servos.updateSweep();
}
