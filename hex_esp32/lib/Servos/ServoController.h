#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <vector>

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
    
    // Function to set speed
    void setSpeeds(std::vector<int> speeds);
    void setSpeed(int servoIdx, int speed);
    
    // Main update function to move servos based on stored positions
    void update();
    void setManualPWM(int servoNum, int pwmValue);
    String getStatus();  // Print servo status for debugging
    
private:
    Adafruit_PWMServoDriver pwm;
    std::vector<float> servoAngles;  // Vector to store servo angles
    std::vector<int> servoSpeeds;  // Vector to store speed of each servo
    const int SERVO_COUNT = 3;  // Number of servos
    bool servoEnabled[3];  // Array to store enabled state of each servo

    // Helper function to move the servo
    void move(int servoNum, float angle);
};

#endif