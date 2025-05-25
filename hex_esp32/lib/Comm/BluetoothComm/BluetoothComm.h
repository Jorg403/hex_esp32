#ifndef BLUETOOTHCOMM_H
#define BLUETOOTHCOMM_H

#include "../IComm.h"
#include <BluetoothSerial.h>

class BluetoothComm : public IComm {
private:
    BluetoothSerial _btSerial;
    String _incomingData;
    
public:
    BluetoothComm();  // constructor
    void begin() override;
    bool available() override;
    String read() override;
    void send(const String& data) override;
};

#endif
