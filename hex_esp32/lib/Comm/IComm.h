#ifndef ICOMM_H
#define ICOMM_H

#include <Arduino.h>

class IComm {
public:
    virtual void begin() = 0;
    virtual bool available() = 0;
    virtual String read() = 0;
    virtual void send(const String& data) = 0;
    virtual ~IComm() {}
};

#endif
