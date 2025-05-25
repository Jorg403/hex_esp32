#include "ServoController.h"
#include "Consts.h"

ServoController::ServoController() : pwm(Adafruit_PWMServoDriver()) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (i%3 == 0) {
            servoAngles.push_back(0.0);
        } else if (i%3 == 1) {
            servoAngles.push_back(0.0);
        } else {
            servoAngles.push_back(0.0);
        }
        servoSpeeds.push_back(0);     // Initialize speeds to 0
        servoEnabled[i] = true;      // Initialize all servos as disabled
    }
}

void ServoController::begin() {
    pwm.begin();
    pwm.setPWMFreq(60);
}

void ServoController::setServo(int servoIdx, int angle) {
    if (servoIdx >= 0 && servoIdx < SERVO_COUNT) {
        servoAngles[servoIdx] = constrain(angle, __SERVO_MIN_POS__[servoIdx], __SERVO_MAX_POS__[servoIdx]);
    }
}

void ServoController::setServos(std::vector<int> angles) {
    for (int i = 0; i < angles.size() && i < SERVO_COUNT; i++) {
        servoAngles[i] = constrain(angles[i], __SERVO_MIN_POS__[i], __SERVO_MAX_POS__[i]);
    }
}

void ServoController::moveServo(int servoIdx, int offset) {
    if (servoIdx >= 0 && servoIdx < SERVO_COUNT) {
        servoAngles[servoIdx] += offset;
        servoAngles[servoIdx] = constrain(servoAngles[servoIdx], __SERVO_MIN_POS__[servoIdx], __SERVO_MAX_POS__[servoIdx]);
    }
}

void ServoController::enableServo(int servoIdx) {
    if (servoIdx >= 0 && servoIdx < SERVO_COUNT) {
        servoEnabled[servoIdx] = true;
    }
}

void ServoController::disableServo(int servoIdx) {
    if (servoIdx >= 0 && servoIdx < SERVO_COUNT) {
        servoEnabled[servoIdx] = false;
    }
}

void ServoController::enableServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        servoEnabled[i] = true;
    }
}

void ServoController::disableServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        servoEnabled[i] = false;
    }
}

void ServoController::setSpeeds(std::vector<int> speeds) {
    for (int i = 0; i < speeds.size() && i < SERVO_COUNT; i++) {
        servoSpeeds[i] = speeds[i];
    }
}

void ServoController::setSpeed(int servoIdx, int speed) {
    if (servoIdx >= 0 && servoIdx < SERVO_COUNT) {
        servoSpeeds[servoIdx] = speed;
    }
}

void ServoController::update() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servoEnabled[i]) {
            servoAngles[i] += (servoSpeeds[i] * __DELTA_TIME__)/1000.0;
            servoAngles[i] = constrain(servoAngles[i], __SERVO_MIN_POS__[i], __SERVO_MAX_POS__[i]);
            move(i, servoAngles[i]);
        }
    }
}

void ServoController::setManualPWM(int servoNum, int pulse) {
    disableServo(servoNum);  // Disable the servo for manual control
    pwm.setPWM(servoNum, 0, pulse);
}

void ServoController::move(int servoNum, float angle) {
    int pulse = map((int)angle, -90, 0, __SERVO_neg90_PWMs__[servoNum], __SERVO_0_PWMs__[servoNum]);
    pwm.setPWM(servoNum, 0, pulse);
}

String ServoController::getStatus() {
    String status = "===ServoController Status===\n";
    for (int i = 0; i < SERVO_COUNT; i++) {
        status += "Servo " + String(i) + ": Angle = " + String(servoAngles[i]) +
                  ", Speed = " + String(servoSpeeds[i]) +
                  ", Enabled = " + String(servoEnabled[i] ? "Yes" : "No") + "\n";
    }
    return status;
}
