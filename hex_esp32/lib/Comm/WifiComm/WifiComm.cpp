#include "WifiComm.h"
#include <WiFi.h>
#include <WebServer.h>

WifiComm::WifiComm() : _server(80) {}

void WifiComm::begin() {
    WiFi.begin(_ssid, _password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
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
