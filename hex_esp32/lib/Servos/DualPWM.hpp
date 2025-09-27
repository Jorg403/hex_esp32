#ifndef DUAL_PWM_HPP
#define DUAL_PWM_HPP

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Clase para manejar dos PCA9685 como un solo controlador
class DualPWM {
public:
    DualPWM(uint8_t addrRight, uint8_t addrLeft)
        : pcaLeft(addrLeft), pcaRight(addrRight) {}

    void begin() {
        pcaRight.begin();
        pcaLeft.begin();
    }

    void setPWMFreq(float freq) {
        pcaRight.setPWMFreq(freq);
        pcaLeft.setPWMFreq(freq);delay(1000);
    }

    void setPWM(uint8_t servoNum, uint16_t on, uint16_t off) {
        if (servoNum < 9) {
            pcaRight.setPWM(servoNum, on, off);
        } else {
            pcaLeft.setPWM(servoNum - 9, on, off);
        }
    }

private:
    Adafruit_PWMServoDriver pcaLeft;
    Adafruit_PWMServoDriver pcaRight;
};

#endif // DUAL_PWM_HPP
