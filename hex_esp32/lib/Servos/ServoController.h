#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "DualPWM.hpp"

class ServoController {
public:
    ServoController();
    void begin();
    
    // Functions to set and get servo positions
    void setServo(int servoIdx, int angle);
    void setServos(std::vector<int> angles);
    void moveServo(int servoIdx, int offset);
    
    // Functions to enable/disable servos
    void enableServo(int servoIdx);
    void disableServo(int servoIdx);
    void enableServos();
    void disableServos();
    
    // Main update function to move servos based on stored positions
    void update();
    void setManualPWM(int servoNum, int pwmValue);
    String getStatus();  // Print servo status for debugging
    
private:
    DualPWM pwm;
    std::vector<float> servoAngles;  // Vector to store servo angles
    std::vector<bool> servoEnabled;  // Array to store enabled state of each servo
    
    const int ARM_COUNT = 6;  // Number of arms

    // Helper function to move the servo
    void move(int servoNum, float angle);
};

#endif