#ifndef WIFICOMM_H
#define WIFICOMM_H

#include "../IComm.h"
#include <WiFi.h>
#include <WebServer.h>

class WifiComm : public IComm {
private:
    const char* _ssid = "Hackback";
    const char* _password = "biczocho";
    WebServer _server;
    String _incomingData;
    IPAddress _brainIP;

public:
    WifiComm(); // constructor declared
    void begin() override;
    bool available() override;
    String read() override;
    void send(const String& data) override;
};

#endif
