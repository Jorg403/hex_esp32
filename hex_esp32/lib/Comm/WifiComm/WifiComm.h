#ifndef WIFICOMM_H
#define WIFICOMM_H

#include "../IComm.h"
#include "WiFiConsts.h"
#include <WiFi.h>
#include <WebServer.h>

class WifiComm : public IComm {
private:
    const char* _ssid = __PERSONAL_SSID__;
    const char* _password = __PERSONAL_PASSWORD__;
    const char* _ssid2 = __PERSONAL_SSID2__;
    const char* _password2 = __PERSONAL_PASSWORD2__;
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
