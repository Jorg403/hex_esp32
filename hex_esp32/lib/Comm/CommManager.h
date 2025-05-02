#ifndef COMMMANAGER_H
#define COMMMANAGER_H

#include "IComm.h"
#include "./UARTcomm/UARTcomm.h"
#include "./WifiComm/WifiComm.h"
#include "./BluetoothComm/BluetoothComm.h"

class CommManager : public IComm {
private:
    IComm* commImpl;

public:
    enum CommMode { UART, WIFI, BLUETOOTH };

    CommManager(CommMode mode) {
        switch (mode) {
            case UART:
                commImpl = new UARTComm();
                break;
            case WIFI:
                commImpl = new WifiComm();
                break;
            case BLUETOOTH:
                commImpl = new BluetoothComm();
                break;
            default:
                commImpl = new UARTComm();  // fallback
        }
    }

    ~CommManager() {
        delete commImpl;
    }

    void begin() override {
        commImpl->begin();
    }

    bool available() override {
        return commImpl->available();
    }

    String read() override {
        return commImpl->read();
    }

    void send(const String& data) override {
        commImpl->send(data);
    }
};

#endif
