#include "ServoController.h"
#include "Consts.h"

ServoController::ServoController() : pwm(Adafruit_PWMServoDriver()) {
    servoAngles = {90, 90};  // Default positions
    servoSpeeds = {0, 0};    // Default speed
    servoEnabled[0] = true;
    servoEnabled[1] = true;
}

void ServoController::begin() {
    pwm.begin();
    pwm.setPWMFreq(60);
}

void ServoController::setServo(int servoIdx, int angle) {
    if (servoIdx >= 0 && servoIdx < servoAngles.size()) {
        servoAngles[servoIdx] = constrain(angle, 0, 180);
    }
}

void ServoController::setServos(std::vector<int> angles) {
    for (int i = 0; i < angles.size() && i < servoAngles.size(); i++) {
        servoAngles[i] = constrain(angles[i], 0, 180);
    }
}

void ServoController::moveServo(int servoIdx, int offset) {
    if (servoIdx >= 0 && servoIdx < servoAngles.size()) {
        servoAngles[servoIdx] += offset;
        servoAngles[servoIdx] = constrain(servoAngles[servoIdx], 0, 180);
    }
}

void ServoController::enableServo(int servoIdx) {
    if (servoIdx >= 0 && servoIdx < 2) {
        servoEnabled[servoIdx] = true;
    }
}

void ServoController::disableServo(int servoIdx) {
    if (servoIdx >= 0 && servoIdx < 2) {
        servoEnabled[servoIdx] = false;
    }
}

void ServoController::enableServos() {
    servoEnabled[0] = true;
    servoEnabled[1] = true;
}

void ServoController::disableServos() {
    servoEnabled[0] = false;
    servoEnabled[1] = false;
}

void ServoController::setSpeeds(std::vector<int> speeds) {
    for (int i = 0; i < speeds.size() && i < servoSpeeds.size(); i++) {
        servoSpeeds[i] = speeds[i];
    }
}

void ServoController::setSpeed(int servoIdx, int speed) {
    if (servoIdx >= 0 && servoIdx < 2) {
        servoSpeeds[servoIdx] = speed;
    }
}

void ServoController::update() {
    for (int i = 0; i < 2; i++) {
        if (servoEnabled[i]) {
            servoAngles[i] += (servoSpeeds[i] * __DELTA_TIME__)/1000.0;
            if (servoAngles[i] < 0) servoAngles[i] = 0;
            else if (servoAngles[i] > 180) servoAngles[i] = 180;
            move(i, servoAngles[i]);
        }
    }
}

void ServoController::setManualPWM(int servoNum, int pulse) {
    disableServo(servoNum);  // Disable the servo for manual control
    pwm.setPWM(servoNum, 0, pulse);
}

void ServoController::move(int servoNum, float angle) {
    int pulse = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoNum, 0, pulse);
}

String ServoController::getStatus() {
    String status = "===ServoController Status===\n";
    for (int i = 0; i < servoAngles.size(); i++) {
        status += "Servo " + String(i) + ": Angle = " + String(servoAngles[i]) +
                  ", Speed = " + String(servoSpeeds[i]) +
                  ", Enabled = " + String(servoEnabled[i] ? "Yes" : "No") + "\n";
    }
    return status;
}
