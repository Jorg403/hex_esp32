#ifndef UARTCOMM_H
#define UARTCOMM_H

#include "../IComm.h"
#include <Arduino.h>

class UARTComm : public IComm {
private:
    Stream& serialPort;

public:
    explicit UARTComm(Stream& port = Serial);

    void begin() override;
    bool available() override;
    String read() override;
    void send(const String& data) override;
};

#endif
