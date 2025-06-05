#include "BluetoothComm.h"

BluetoothComm::BluetoothComm() {}

void BluetoothComm::begin() {
    if (!_btSerial.begin("ESP32_BT")) {  // Nombre Bluetooth
        Serial.println("Error inicializando Bluetooth");
    } else {
        Serial.println("Bluetooth iniciado. Visible como 'ESP32_BT'");
    }
}

bool BluetoothComm::available() {
    if (_btSerial.available()) {
        _incomingData = _btSerial.readStringUntil('\n');  // lee hasta newline o timeout
        _incomingData.trim();  // elimina saltos de línea y espacios extras
        return true;
    }
    return false;
}

String BluetoothComm::read() {
    String temp = _incomingData;
    _incomingData = "";
    return temp;
}

void BluetoothComm::send(const String& data) {
    // _btSerial.println(data);
    // Serial.println(data);  // log
}