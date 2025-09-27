#include "ServoController.h"
#include "Consts.h"

ServoController::ServoController() : pwm(DualPWM(0x40, 0x41)) {
    for (int i = 0; i < ARM_COUNT; i++) {
        servoAngles.push_back(0.0);
        servoAngles.push_back(0.0);
        servoAngles.push_back(0.0);
        
        // servoSpeeds.push_back(0);
        // servoSpeeds.push_back(0);
        // servoSpeeds.push_back(0);
        
        servoEnabled.push_back(true);
        servoEnabled.push_back(true);
        servoEnabled.push_back(true);
    }
}

void ServoController::begin() {
    pwm.begin();
    pwm.setPWMFreq(50);
    for (int i = 0; i < ARM_COUNT*3; i++) {
        servoAngles[i] = constrain(servoAngles[i], __SERVO_MIN_POS__[i%3], __SERVO_MAX_POS__[i%3]);
        move(i, servoAngles[i]);
        delay(1000);  // Small delay to allow servo to move
    }
    Serial.println("ServoController initialized.");
}

void ServoController::setServo(int servoIdx, int angle) {
    if (servoIdx >= 0 && servoIdx < ARM_COUNT*3) {
        servoAngles[servoIdx] = constrain(angle, __SERVO_MIN_POS__[servoIdx%3], __SERVO_MAX_POS__[servoIdx%3]);
    }
}

void ServoController::setServos(std::vector<int> angles) {
    for (int i = 0; i < angles.size() && i < ARM_COUNT*3; i++) {
        Serial.print("Setting servo ");
        Serial.print(i);
        Serial.print(" to angle: ");
        Serial.println(angles[i]);
        servoAngles[i] = constrain(angles[i], __SERVO_MIN_POS__[i%3], __SERVO_MAX_POS__[i%3]);
    }
}

void ServoController::moveServo(int servoIdx, int offset) {
    if (servoIdx >= 0 && servoIdx < ARM_COUNT*3) {
        servoAngles[servoIdx] += offset;
        servoAngles[servoIdx] = constrain(servoAngles[servoIdx], __SERVO_MIN_POS__[servoIdx%3], __SERVO_MAX_POS__[servoIdx%3]);
    }
}

void ServoController::enableServo(int servoIdx) {
    if (servoIdx >= 0 && servoIdx < ARM_COUNT*3) {
        servoEnabled[servoIdx] = true;
    }
}

void ServoController::disableServo(int servoIdx) {
    if (servoIdx >= 0 && servoIdx < ARM_COUNT*3) {
        servoEnabled[servoIdx] = false;
    }
}

void ServoController::enableServos() {
    for (int i = 0; i < ARM_COUNT*3; i++) {
        servoEnabled[i] = true;
    }
}

void ServoController::disableServos() {
    for (int i = 0; i < ARM_COUNT*3; i++) {
        servoEnabled[i] = false;
    }
}

// void ServoController::setSpeeds(std::vector<int> speeds) {
//     for (int i = 0; i < speeds.size() && i < SERVO_COUNT; i++) {
//         servoSpeeds[i] = speeds[i];
//     }
// }

// void ServoController::setSpeed(int servoIdx, int speed) {
//     if (servoIdx >= 0 && servoIdx < SERVO_COUNT) {
//         servoSpeeds[servoIdx] = speed;
//     }
// }

void ServoController::update() {
    // Serial.println("Updating servos...");
    for (int i = 0; i < ARM_COUNT*3; i++) {
        // Serial.print("Servo ");
        // Serial.print(i);
        if (servoEnabled[i]) {
            // Serial.print(" enabled, angle: ");
            // Serial.println(servoAngles[i]);
            // servoAngles[i] += (servoSpeeds[i] * __DELTA_TIME__)/1000.0;
            servoAngles[i] = constrain(servoAngles[i], __SERVO_MIN_POS__[i%3], __SERVO_MAX_POS__[i%3]);
            move(i, servoAngles[i]);
        }
    }
}

void ServoController::setManualPWM(int servoNum, int pulse) {
    Serial.print("Setting manual PWM for servo ");
    Serial.print(servoNum);
    disableServo(servoNum);  // Disable the servo for manual control
    pwm.setPWM(servoNum, 0, pulse);
}

void ServoController::move(int servoNum, float angle) {
    int pulse = map((int)angle, -90, 0, __SERVO_neg90_PWMs__[servoNum], __SERVO_0_PWMs__[servoNum]);
    pwm.setPWM(servoNum, 0, pulse);
    // Serial.print(" to angle: ");
    // Serial.println(angle);
}

String ServoController::getStatus() {
    String status = "===ServoController Status===\n";
    for (int i = 0; i < ARM_COUNT*3; i++) {
        status += "Servo " + String(i) + ": Angle = " + String(servoAngles[i]) +
                //   ", Speed = " + String(servoSpeeds[i]) +
                  ", Enabled = " + String(servoEnabled[i] ? "Yes" : "No") + "\n";
    }
    return status;
}
