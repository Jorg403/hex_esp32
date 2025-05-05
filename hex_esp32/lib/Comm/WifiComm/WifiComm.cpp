#include "WifiComm.h"
#include <WiFi.h>
#include <WebServer.h>

WifiComm::WifiComm() : _server(80) {}

void WifiComm::begin() {
    while(WiFi.status() != WL_CONNECTED) {
        WiFi.begin(_ssid, _password);
        Serial.print("Connecting to WiFi");
        for (int i=0; WiFi.status() != WL_CONNECTED && i<15; i++) {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.begin(_ssid2, _password2);
            Serial.print("Connecting to WiFi2");
            for (int i=0; WiFi.status() != WL_CONNECTED && i<15; i++) {
                delay(500);
                Serial.print(".");
            }
        }
    }
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());


    _server.on("/", HTTP_GET, [this]() {
        if (_server.hasArg("cmd")) {
            _incomingData = _server.arg("cmd");
        }
        _server.send(200, "text/plain", "");
        _brainIP = _server.client().remoteIP();
    });
    _server.begin();
}

bool WifiComm::available() {
    _server.handleClient();
    return !_incomingData.isEmpty();
}

String WifiComm::read() {
    String temp = _incomingData;
    _incomingData = "";
    return temp;
}

void WifiComm::send(const String& data) {
    // String ip = "172.20.104.190";
    // if (WiFi.status() == WL_CONNECTED) {
    //     WiFiClient client;
    //     if (client.connect(ip.c_str(), 8080)) {
    //         client.print("cmd=" + data);
    //         client.stop();
    //     } else {
    //         Serial.println("Failed to connect to brain IP: " + _brainIP.toString());
    //     }
    // } else {
    //     Serial.println("WiFi not connected. Cannot send data.");
    // }
    Serial.println(data);
}
