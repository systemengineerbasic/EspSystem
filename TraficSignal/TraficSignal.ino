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

#define WIFI_ENABLE	(1)

// Signal Mode
#define	SM_AUTO			(0)
#define	SM_BLUE			(1)
#define	SM_YELLOW		(2)
#define	SM_RED			(3)

// Signal COLOR
#define	SCOLOR_BLUE		(0)
#define	SCOLOR_YELLOW	(1)
#define	SCOLOR_RED		(2)

const char* ssid = "L01_B0E5ED682BFE";
const char* password = "d3qart6mhna8m8j";
//自分で設定した CloudMQTT の Instance info から取得
const char* mqttServer = "m10.cloudmqtt.com";
const char* mqttDeviceId = "Signal01";
const char* mqttUser = "rkinkmdk";
const char* mqttPassword = "IAzU6d_IdX0W";
const int mqtt_port = 14519;
const char* mqttTopic_Signal = "KM/Signal";
const char* mqttTopic_Command = "KM/Command";


const int RED_PIN = 13;
const int YELLOW_PIN = 12;
const int BLUE_PIN = 14;

//Connect through TLS1.1
//WiFiClientSecure espClient;
WiFiClient espClient;
PubSubClient client(espClient);

QueueHandle_t g_xQueue_Command;


int g_signal_mode = SM_AUTO;
int g_signal_color = SCOLOR_RED;
int g_signal_timing_count = 0;


void MQTT_callback(char* topic, byte* payload, unsigned int length) 
{
	//----- JSON形式のデータを取り出す
	StaticJsonDocument<200> doc;
	// Deserialize
	deserializeJson(doc, payload);
	// extract the data
	JsonObject object = doc.as<JsonObject>();
	if(strcmp(topic, mqttTopic_Command) == 0) {
		const char* led = object["Signal"];
		if(led != NULL) {
			int signal_mode = 0;
			if(strcmp(led, "BLUE") == 0) {
				signal_mode = SM_BLUE;
			}
			else if(strcmp(led, "YELLOW") == 0) {
				signal_mode = SM_YELLOW;
			}
			else if(strcmp(led, "RED") == 0) {
				signal_mode = SM_RED;
			}
			else {
				signal_mode = SM_AUTO;
			}
			// Send Q-message to command queue
	        xQueueSend(g_xQueue_Command, &signal_mode, 0);
			Serial.print("COMMAND:");
			Serial.println(led);
		}
	}
}

void osTask_MQTT(void* param)
{
	for(;;) {
		vTaskDelay(200);

#if WIFI_ENABLE
		if (!client.connected()) {
			reconnect();
		}
		client.loop();
#endif
	}

}

void osTask_Signal(void* param)
{
	for(;;) {
		int q_data = 0;
		int wait_time = 1000; // [ms]
		BaseType_t	xStatus = xQueueReceive(g_xQueue_Command, &q_data, wait_time/portTICK_RATE_MS);
		if(xStatus) { // if you receive some data from queue.
			g_signal_mode = q_data;
			g_signal_timing_count = 0; // Reset counter
			
			Serial.println("Receive queue");
			if(g_signal_mode == SM_BLUE) {
				set_signal_color(SCOLOR_BLUE);
			}
			else if(g_signal_mode == SM_YELLOW) {
				set_signal_color(SCOLOR_YELLOW);
			}
			else if(g_signal_mode == SM_RED) {
				set_signal_color(SCOLOR_RED);
			}
		}
		
		// LED signal rotation(B -> Y -> R)
		if(g_signal_mode == SM_AUTO) {
			g_signal_timing_count ++;
			if(g_signal_color == SCOLOR_BLUE) {
				if(g_signal_timing_count >= 5) { // Blue -> Yellow
					set_signal_color(SCOLOR_YELLOW);
					g_signal_timing_count = 0;
				}
			}
			else if(g_signal_color == SCOLOR_YELLOW) {
				if(g_signal_timing_count >= 2) { // Yellow -> Red
					set_signal_color(SCOLOR_RED);
					g_signal_timing_count = 0;
				}
			}
			else {
				if(g_signal_timing_count >= 5) { // Red -> Blue
					set_signal_color(SCOLOR_BLUE);
					g_signal_timing_count = 0;
				}
			}
		}

	}

}

//
// set LED signal color
//
void set_signal_color(int color)
{
	String str_color;
	g_signal_color = color;
	
	// Turn on/off for each LED
	if(g_signal_color == SCOLOR_BLUE) {
		str_color = "BLUE";
		digitalWrite(BLUE_PIN, HIGH);
		digitalWrite(YELLOW_PIN, LOW);
		digitalWrite(RED_PIN, LOW);
	}
	else if(g_signal_color == SCOLOR_YELLOW) {
		str_color = "YELLOW";
		digitalWrite(BLUE_PIN, LOW);
		digitalWrite(YELLOW_PIN, HIGH);
		digitalWrite(RED_PIN, LOW);
	}
	else {
		str_color = "RED";
		digitalWrite(BLUE_PIN, LOW);
		digitalWrite(YELLOW_PIN, LOW);
		digitalWrite(RED_PIN, HIGH);
	}

	// Create JSON formatted message
	String payload = "{\"deviceName\":\"";
	payload += mqttDeviceId;
	payload += "\",\"LED\":\"";
	payload += str_color;
	payload += "\"}";
	Serial.print("Publish message: ");
	Serial.println(payload);

	// Publish JSON formatted message to MQTT broker
	client.publish(mqttTopic_Signal, (char*) payload.c_str());
}

void setup()
{
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  Serial.begin(9600);
#if WIFI_ENABLE
  WiFi.mode(WIFI_STA);
  setupWifi();
  client.setServer(mqttServer, mqtt_port);
  // topicをsubscribeしたときのコールバック関数を登録
  client.setCallback(MQTT_callback);
#endif

  // Create queue for receive command from web.
  g_xQueue_Command = xQueueCreate(8, sizeof(int32_t));
  // Create tasks 
  xTaskCreatePinnedToCore(osTask_MQTT, "osTask_MQTT", 2048, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(osTask_Signal, "osTask_Signal", 4096, NULL, 3, NULL, 0);

  set_signal_color(SCOLOR_RED);

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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttDeviceId, mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe(mqttTopic_Command);
      Serial.print("Subscribe:");
      Serial.println(mqttTopic_Command);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(5000);
    }
  }
}
void loop() {
}
