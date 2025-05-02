#include "../IComm.h"

class BluetoothComm : public IComm {
public:
    void begin() override {
        //Serial.println("Not implemented yet: BluetoothComm::begin()");
    }

    bool available() override {
        //Serial.println("Not implemented yet: BluetoothComm::available()");
        return false; // Placeholder
    }

    String read() override {
        //Serial.println("Not implemented yet: BluetoothComm::read()");
        return ""; // Placeholder
    }

    void send(const String& data) override {
        //Serial.println("Not implemented yet: BluetoothComm::send()");
    }
};
