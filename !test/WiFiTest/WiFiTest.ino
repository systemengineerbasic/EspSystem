#include <WiFi.h>

// 接続先のSSIDとパスワード
const char ssid[] = "XXXXXXXXXX";
const char passwd[] = "XXXXXXXXXX";

void setup() {
    Serial.begin(9600);

    // Connect WiFi
    connectWiFi();
}
void loop() {
    // Reconnect
    if ( WiFi.status() == WL_DISCONNECTED ) {
        connectWiFi();
    }
}

/**
 * Connect WiFi
 */
void connectWiFi()
{
    WiFi.begin(ssid, passwd);
    Serial.print("WiFi connecting...");
    while(WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();
    Serial.print("connected.");
    Serial.println(WiFi.localIP());
}

