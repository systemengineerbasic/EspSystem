/*
 *  Refer from WiFiClient sample sketch.
 */

#include <WiFi.h>
#include <PubSubClient.h>

// Wifi アクセスポイントの情報
const char* ssid = "HUMAX-C4130";
const char* password = "LGNWLTNmMTdFX";

// 自分で設定した CloudMQTT.xom サイトの Instance info から取得
const char* mqttServer = "m16.cloudmqtt.com";
const char* mqttDeviceId = "KMCar001";
const char* mqttUser = "vsscjrry";
const char* mqttPassword = "kurgC_M_VZmF";
const int mqttPort = 17555;

// Subscribe する MQTT Topic 名
const char* mqttTopic = "KM/Signal";

//Connect WiFi Client and MQTT(PubSub) Client
WiFiClient espClient;
PubSubClient client(espClient);

/* 
 *  Subscribe している Topic にメッセージが来た時に処理させる Callback 関数を設定。
 *  ここでは単にメッセージを取り出しているだけ。
 *  JSON 形式にしてるけど、あんまり必要なさそうであれば、Topic と Message だけで判別させたい。
 *  でも JSON 形式にしておくと、後から判別させたりする際に使いやすいから、どうするか。
 */
void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
  Serial.println();
  Serial.println("-----------------------");

 // ArduinoJson.h ライブラリが使えたら、バッファを準備した後、JSON の中身を取り出して root に入れて、そのあと取り出す。
 /*
  * 以下、Sample code 例。https://arduinojson.org/v5/assistant/
  * const size_t capacity = JSON_OBJECT_SIZE(1) + 10;
  * DynamicJsonBuffer jsonBuffer(capacity);
  * JsonObject& root = jsonBuffer.parseObject((char*)payload);
  * const char* signalColor = root["LED"]; // "BLUE"
  * あとは、signalColor の色で条件分岐して、モーターの動作を制御。
  */
 
}

// MQTT Client が接続できなかったら接続できるまで再接続を試みるための reconnect 関数
// わかりにくいので、エレスクでは単純に、setup() の中で MQTT に接続して、mqttTopic を Subscribe させておくだけでよさそう。
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttDeviceId, mqttUser, mqttPassword)) {
      Serial.println("connected");
      // mqttTopic で設定した MQTT Topic を Subscribe する
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


// 電源を入れて最初に実行する設定 (WiFi 接続と MQTT サーバーへの接続)
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }
  Serial.println("Connected to the WiFi network");

// 念のため接続した WiFi アクセスポイントから付与された IP アドレスを確認できるようにした。
  Serial.print("WiFi connected IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

// ループさせる処理。
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
