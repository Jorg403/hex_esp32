#ifndef BLUETOOTHCOMM_H
#define BLUETOOTHCOMM_H

#include "../IComm.h"
#include <Arduino.h>

class BluetoothComm : public IComm {
public:
    explicit BluetoothComm();

    void begin() override;
    bool available() override;
    String read() override;
    void send(const String& data) override;
};

#endif
