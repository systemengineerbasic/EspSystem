#include <WiFi.h>
#include <PubSubClient.h>

// 接続先のSSIDとパスワード
const char ssid[] = "XXXXXXXXXX";
const char password[] = "XXXXXXXXXX";

// 自分で設定した CloudMQTT.com サイトの Instance info から取得
const char* mqttServer = "m15.cloudmqtt.com";
const char* mqttDeviceId = "RoboCar";
const char* mqttUser = "niccngso";
const char* mqttPassword = "6UKWREecCBYB";
const int mqttPort = 13415;

// Subscribe する MQTT Topic 名
const char* mqttTopic_Signal = "KM/Signal";
const char* mqttTopic_Query = "KM/Query";

//Connect WiFi Client and MQTT(PubSub) Client
WiFiClient espClient;
PubSubClient g_mqtt_client(espClient);

void MQTT_callback(char* topic, byte* payload, unsigned int length) 
{
	Serial.print("received MQTT topic:"); 
	Serial.println(topic); 
	Serial.print("length = "); 
	Serial.println(length); 
	Serial.print("Payload : ");
	Serial.println((char*)payload);
	
	Serial.println();
}

// MQTT Client が接続できなかったら接続できるまで再接続を試みるための MQTT_reconnect 関数
void MQTT_reconnect() 
{
	// Loop until we're reconnected
	while (!g_mqtt_client.connected()) {
		Serial.print("Attempting MQTT connection...");
		// Attempt to connect
		if (g_mqtt_client.connect(mqttDeviceId, mqttUser, mqttPassword)) {
			Serial.println("connected");
			g_mqtt_client.subscribe(mqttTopic_Signal);
		} 
		else {
			Serial.print("failed, rc=");
			Serial.print(g_mqtt_client.state());
			Serial.println(" try again in 5 seconds");
			// Wait 5sec before retrying
			delay(1000);
		}
	}
}

void MQTT_publish_query()
{
	char payload[64];
	sprintf(payload, "MQTT test query by Nishimura");
	// MQTT brokerへpublish
	g_mqtt_client.publish(mqttTopic_Query, payload);
}



void setup() 
{
    Serial.begin(9600);

    // Connect WiFi
    WiFi.begin(ssid, password);

	int cnt = 0;
	int is_success = true;
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print("WiFi connecting to ");
		Serial.println(ssid);
		cnt ++;
		if(cnt > 30) {
			is_success = false;
			break;
		}
	}
	if(is_success) {
		// WiFi アクセスポイントから付与されたIPアドレスを表示
		Serial.print("Connected to the WiFi network. IP address: ");
		Serial.println(WiFi.localIP());
		// MQTT Serverの設定
		g_mqtt_client.setServer(mqttServer, mqttPort);
		// topicをsubscribeしたときのコールバック関数を登録
		g_mqtt_client.setCallback(MQTT_callback);
		// MQTT brokerとの接続
		MQTT_reconnect();
	}
	else {
		Serial.println("[Errol] Cannot connected to the WiFi network");
	}

}

int count = 0;
void loop() 
{
	if(!g_mqtt_client.connected()) {
		MQTT_reconnect();
	}
	g_mqtt_client.loop();
	
	count ++;
	if(count >= 10) {
		MQTT_publish_query();
		count = 0;
	}
	
	delay(200);
}

