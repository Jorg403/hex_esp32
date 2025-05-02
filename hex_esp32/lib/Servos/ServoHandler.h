#include <Arduino.h>
#include "ServoController.h"

class ServoHandler {
public:
    ServoHandler(ServoController& controller);
    String handleSrvCommand(const String& command);
    void setPositions(const String& params);
    void stopMovements();
    ServoController& servoController;
};