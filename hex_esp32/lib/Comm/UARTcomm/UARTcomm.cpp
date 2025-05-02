#include "UARTComm.h"

UARTComm::UARTComm(Stream& port) : serialPort(port) {}

void UARTComm::begin() {
    // serialPort.begin(115200);
}

bool UARTComm::available() {
    return serialPort.available() > 0;
}

String UARTComm::read() {
    return serialPort.readStringUntil('\n');
}

void UARTComm::send(const String& data) {
    serialPort.println(data);
}
