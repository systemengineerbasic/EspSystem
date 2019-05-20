                          /*
 *  LEDSignal_CloudMQTT2.ino
 *  Copyright (c) 2018 
 *  Modified from Masami Yamakawa's "cloudmqtt_led.ino".
 *  setupWifi(), reconnect() and parts of callback() functions are
 *    Copyright (c) 2008-2015 Nicholas O'Leary
 *  This software is released under the MIT License.
 *  http://opensource.org/licenses/mit-license.php
 *  
 *  An example to demonstrate analogRead and analogWrite over MQTT.
 *  cloudmqtt_led.ino subscriber
 *  cloudmqtt_light.ino publisher
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define WIFI_ENABLE	(0)

//const char* ssid = "SPWN_H36_ED08D1";
//const char* password = "ht3yf214i7e89ty";
const char* ssid = "L01_B0E5ED682BFE";
const char* password = "d3qart6mhna8m8j";
//自分で設定した CloudMQTT の Instance info から取得
const char* mqttServer = "m15.cloudmqtt.com";
const char* mqttDeviceId = "Signal01";
const char* mqttUser = "niccngso";
const char* mqttPassword = "6UKWREecCBYB";
const int mqtt_port = 13415;
const char* mqttTopic = "KM/Signal2";
const int RED_PIN = 13;
const int YELLOW_PIN = 12;
const int BLUE_PIN = 14;

//Connect through TLS1.1
//WiFiClientSecure espClient;
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastUpdateMillis = 0;
unsigned int value = 0;

void setup() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  Serial.begin(9600);
#if WIFI_ENABLE
  WiFi.mode(WIFI_STA);
  setupWifi();
  client.setServer(mqttServer, mqtt_port);
//  client.setCallback(callback);
#endif
}

void setupWifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

#if WIFI_ENABLE
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif
}

/*
 *  ボタンが押されたことを確認するために MQTT に Callback する処理
void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // JSON形式で送られたメッセージの中身を取り出すためのバッファを準備
  StaticJsonBuffer<200> jsonBuffer;
  // JSON形式の中身を取り出して、rootに入れる
  JsonObject& root = jsonBuffer.parseObject((char*) payload);

  // rootからbuttonを値を取り出してinValueに入れる
  int  inValue = root["button"];

  Serial.print("inValue:");
  Serial.println(inValue);
  // IO13 ピンにつながっている LED の電圧を Value 値で変更させることで、LED の光量を変動させる。
  analogWrite(13,inValue);
}
*/

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttDeviceId, mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe(mqttTopic);
      Serial.print("Subscribe:");
      Serial.println(mqttTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
#if WIFI_ENABLE
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
#endif
  // payloadに{"deviceName":"Signal01","LED":光っている信号の色}をセット
  digitalWrite(BLUE_PIN, HIGH);
  Serial.println("BLUE");

    String payload1 = "{\"deviceName\":\"";
    payload1 += mqttDeviceId;
    payload1 += "\",\"LED\":";
    payload1 += "\"BLUE\"";
    payload1 += "}";
    Serial.print("Publish message: ");
    Serial.println(payload1);
    // payloadにセットされたJSON形式メッセージを投稿
    client.publish(mqttTopic, (char*) payload1.c_str());

  delay (5000);
  digitalWrite(BLUE_PIN, LOW);

  digitalWrite(YELLOW_PIN, HIGH);
  Serial.println("YELLOW");

    String payload2 = "{\"deviceName\":\"";
    payload2 += mqttDeviceId;
    payload2 += "\",\"LED\":";
    payload2 += "\"YELLOW\"";
    payload2 += "}";
    Serial.print("Publish message: ");
    Serial.println(payload2);
    // payloadにセットされたJSON形式メッセージを投稿
    client.publish(mqttTopic, (char*) payload2.c_str());

  delay (2000);
  digitalWrite(YELLOW_PIN, LOW);

  digitalWrite(RED_PIN, HIGH);
  Serial.println("RED");
    
    String payload3 = "{\"deviceName\":\"";
    payload3 += mqttDeviceId;
    payload3 += "\",\"LED\":";
    payload3 += "\"RED\"";
    payload3 += "}";
    Serial.print("Publish message: ");
    Serial.println(payload3);
    // payloadにセットされたJSON形式メッセージを投稿
    client.publish(mqttTopic, (char*) payload3.c_str());

  delay (5000);
  digitalWrite(RED_PIN, LOW);
  
}
