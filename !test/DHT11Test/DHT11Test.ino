#include <WiFi.h>
#include "DHT.h"
#include <Ambient.h>

const char* ssid = "HUMAX-FD2A6";
const char* password = "XTk4NWN4NTM4X";

WiFiClient client;
Ambient ambient;

DHT dht(33,DHT11);  //2番ポート

unsigned int channelId = 20947; // AmbientのチャネルID
const char* writeKey = "10cdb0759b1a687e"; // ライトキー

void setup() {
    Serial.begin(115200);
    while (!Serial) ;

    WiFi.begin(ssid, password);  // Wi-Fiネットワークに接続する
    while (WiFi.status() != WL_CONNECTED) {  // 接続したか調べる
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());  // ローカルIPアドレスをプリントする

    ambient.begin(channelId, writeKey, &client); // チャネルIDとライトキーを指定してAmbientの初期化

    dht.begin();
}

void loop() {
    float temp = dht.readHumidity();
    float humid = dht.readTemperature();
    Serial.printf("temp: %.2f, humid: %.2f\r\n", temp, humid);

    ambient.set(1, temp);  // Ambientのデータ1に温度をセットする
    ambient.set(2, humid);  // データ2に湿度をセットする
    ambient.send();  // Ambientに送信する

    delay(300 * 1000);
}
