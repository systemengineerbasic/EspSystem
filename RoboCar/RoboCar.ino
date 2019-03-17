#include <Wire.h>
#include <RPR-0521RS.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "BluetoothSerial.h"
#include "esp_system.h"
#include "dev_MPU6050.h"
#include "freertos/task.h"

//================================================================
// 定数定義
//================================================================

//---------------------- PIN ----------------------
// Motor
#define	IO_PIN_MOTOR_1			(14)
#define	IO_PIN_MOTOR_2			(12)
#define	IO_PIN_MOTOR_3			(13)
#define	IO_PIN_MOTOR_4			(23)
#define	IO_PIN_MOTOR_ENA		(16)
#define	IO_PIN_MOTOR_ENB		(27)
// LED
#define	IO_PIN_LED				(18)
#define	IO_PIN_LED2				(2)
// US Sendor
#define	IO_PIN_US_ECHO			(36)
#define	IO_PIN_US_TRIG			( 5)
// Servo
#define	IO_PIN_SERVO			(25)
// 赤外線
#define IO_PIN_INFRARED			(19)
// Line Tracking Sensor
#define IO_PIN_LINETRACK_LEFT	(26)
#define IO_PIN_LINETRACK_CENTER	(17)
#define IO_PIN_LINETRACK_RIGHT	(39)
// I2C
#define IO_PIN_SDA 				SDA
#define IO_PIN_SCL 				SCL

//---------------------- DACのchannel ----------------------
#define DAC_CH_MOTOR_A			(0)
#define DAC_CH_MOTOR_B			(1)
#define DAC_CH_SERVO			(2)

//---------------------- Motor関連 ----------------------
#define	MOTOR_RIGHT_FRONT		0x01
#define	MOTOR_RIGHT_REAR		0x02
#define	MOTOR_LEFT_FRONT		0x04
#define	MOTOR_LEFT_REAR			0x08

#define	MOTOR_RIGHT				(MOTOR_RIGHT_FRONT|MOTOR_RIGHT_REAR)
#define	MOTOR_LEFT				(MOTOR_LEFT_FRONT|MOTOR_LEFT_REAR)
#define	MOTOR_FRONT				(MOTOR_RIGHT_FRONT|MOTOR_LEFT_FRONT)
#define	MOTOR_REAR				(MOTOR_RIGHT_REAR|MOTOR_LEFT_REAR)

#define	MOTOR_DIR_STOP				(0)
#define	MOTOR_DIR_FWD				(1)
#define	MOTOR_DIR_REV				(2)

//---------------------- RoboCar制御関連 ----------------------
#define	STATE_MOTOR_STOP				(0)
#define	STATE_MOTOR_MOVING_FORWARD		(1)
#define	STATE_MOTOR_MOVING_BACKWARD		(2)
#define	STATE_MOTOR_TURNING_RIGHT		(3)
#define	STATE_MOTOR_TURNING_LEFT		(4)
#define	STATE_MOTOR_TURNING_RIGHT_BACK	(5)
#define	STATE_MOTOR_TURNING_LEFT_BACK	(6)
#define	STATE_MOTOR_ROTATING_RIGHT		(7)
#define	STATE_MOTOR_ROTATING_LEFT		(8)

#define	CTRLMODE_AUTO_DRIVE			(0)
#define	CTRLMODE_MANUAL_DRIVE		(1)
#define	CTRLMODE_LINE_TRACKING		(2)

#define	COMMAND_CODE_STOP			(0)	// 引数なし
#define	COMMAND_CODE_FORWARD		(1) // 引数なし
#define	COMMAND_CODE_MOTOR_SPEED	(2)	// [1] Speed

#define	MOTOR_SPEED_MIN				(40)
#define	MOTOR_SPEED_MAX				(255)

//---------------------- 信号機関連 ----------------------
#define	SIGNAL_COLOR_BLACK			(0)
#define	SIGNAL_COLOR_GREEN			(1)
#define	SIGNAL_COLOR_YELLOW			(2)
#define	SIGNAL_COLOR_RED			(3)

//================================================================
// 変数定義
//================================================================

//---------------------- OS関連 ----------------------
QueueHandle_t g_xQueue_Serial;
SemaphoreHandle_t g_xMutex = NULL;

RPR0521RS rpr0521rs;

int8_t	g_is_signal_recieved = 0;
int		g_log_level = 3; // 表示するlogのレベル(0～99)
float g_proficiency_score = 0.0;	// 熟練度


int	g_state_motor = STATE_MOTOR_STOP;
int g_motor_speed = 170;
int g_motor_speed_on_left_turn = 230;
int g_motor_speed_on_right_turn = 230;
int g_lr_level_on_left_turn = 90;
int g_lr_level_on_right_turn = 110;
int g_motor_speed_right;
int g_motor_speed_left;
int	g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;

int	g_trafic_signal_color = SIGNAL_COLOR_RED;

int g_stop_distance = 20;	// [cm]

float servo_coeff_a;
float servo_coeff_b;


// Wifi アクセスポイントの情報
//const char* ssid = "SPWH_H32_F37CDD"; // WiFiルータ1
//const char* ssid = "SPWH_H32_5AE424"; // WiFiルータ2
//const char* password = "********";
//const char* password = "19iyteirq5291f2"; // WiFiルータ1
//const char* password = "jaffmffm04mf01i"; // WiFiルータ2
const char* ssid = "HUMAX-C4130";
const char* password = "LGNWLTNmMTdFX";

// 自分で設定した CloudMQTT.xom サイトの Instance info から取得
const char* mqttServer = "m16.cloudmqtt.com";
const char* mqttDeviceId = "KMCar001";
const char* mqttUser = "vsscjrry";
const char* mqttPassword = "kurgC_M_VZmF";
const int mqttPort = 17555;

// Subscribe する MQTT Topic 名
const char* mqttTopic_Signal = "KM/Signal";
const char* mqttTopic_Sensor = "KM/Sensor";
const char* mqttTopic_Status = "KM/Status";
const char* mqttTopic_Command = "KM/Command";
const char* mqttTopic_Param = "KM/Param";
const char* mqttTopic_Query = "KM/Query";

//Connect WiFi Client and MQTT(PubSub) Client
WiFiClient espClient;
PubSubClient g_mqtt_client(espClient);



#define LT_R (!digitalRead(IO_PIN_LINETRACK_RIGHT))
#define LT_M (!digitalRead(IO_PIN_LINETRACK_CENTER))
#define LT_L (!digitalRead(IO_PIN_LINETRACK_LEFT))
enum {
	TRACK_EVENT_XXX	= 0,
	TRACK_EVENT_XXO,
	TRACK_EVENT_XOX,
	TRACK_EVENT_XOO,
	TRACK_EVENT_OXX,
	TRACK_EVENT_OXO,
	TRACK_EVENT_OOX,
	TRACK_EVENT_OOO_RED,
	TRACK_EVENT_OOO_YELLOW,
	TRACK_EVENT_OOO_GREEN,

	TRACK_EVENT_NUM
};

enum {
	STATE_ROTATO_LEFT = 0,	
	STATE_GO_FORWARD,	
	STATE_ROTATO_RIGHT,	
	STATE_WAIT,	

	STATE_NUM	
};
int g_next_event_table[TRACK_EVENT_NUM][STATE_NUM] =
{
//				Left				Center				Right				Wait
/*XXX*/			STATE_ROTATO_LEFT,	STATE_ROTATO_LEFT,	STATE_ROTATO_RIGHT,	STATE_WAIT,
/*XXO*/			STATE_ROTATO_RIGHT,	STATE_ROTATO_RIGHT,	STATE_ROTATO_RIGHT,	STATE_WAIT,
/*XOX*/			STATE_GO_FORWARD,	STATE_GO_FORWARD,	STATE_GO_FORWARD,	STATE_WAIT,
/*XOO*/			STATE_GO_FORWARD,	STATE_GO_FORWARD,	STATE_ROTATO_RIGHT,	STATE_WAIT,
/*OXX*/			STATE_ROTATO_LEFT,	STATE_ROTATO_LEFT,	STATE_ROTATO_LEFT,	STATE_WAIT,
/*OXO*/			STATE_ROTATO_LEFT,	STATE_ROTATO_RIGHT,	STATE_ROTATO_RIGHT,	STATE_WAIT,
/*OOX*/			STATE_ROTATO_LEFT,	STATE_GO_FORWARD,	STATE_GO_FORWARD,	STATE_WAIT,
/*OOO(RED)*/	STATE_WAIT,			STATE_WAIT,			STATE_WAIT,			STATE_WAIT,

/*OOO(YELLOW)*/	STATE_WAIT,			STATE_WAIT,			STATE_WAIT,			STATE_WAIT,
/*OOO(GREEN)*/	STATE_ROTATO_LEFT,	STATE_GO_FORWARD,	STATE_ROTATO_RIGHT,	STATE_GO_FORWARD,
};

int	g_cur_state = STATE_GO_FORWARD;
#define carSpeed 150

float	g_temperature = 0.0;
float	g_max_diff_axl = 0.0;
unsigned short g_ps_val;
float g_als_val;



/* 
 *  Subscribe している Topic にメッセージが来た時に処理させる Callback 関数を設定。
 *  ここでは単にメッセージを取り出しているだけ。
 *  JSON 形式にしてるけど、あんまり必要なさそうであれば、Topic と Message だけで判別させたい。
 *  でも JSON 形式にしておくと、後から判別させたりする際に使いやすいから、どうするか。
 */
void MQTT_callback(char* topic, byte* payload, unsigned int length) 
{
	//----- JSON形式のデータを取り出す
	StaticJsonDocument<200> doc;
	// Deserialize
	deserializeJson(doc, payload);
	// extract the data
	JsonObject object = doc.as<JsonObject>();
	if(strcmp(topic, mqttTopic_Signal) == 0) {
		const char* led = object["LED"];
		if(led != NULL) {
			g_is_signal_recieved = 1; // 信号情報受信済み
			if(strcmp(led, "GREEN") == 0) {
				g_trafic_signal_color = SIGNAL_COLOR_GREEN;
			}
			else if(strcmp(led, "YELLOW") == 0) {
				g_trafic_signal_color = SIGNAL_COLOR_YELLOW;
			}
			else if(strcmp(led, "RED") == 0) {
				g_trafic_signal_color = SIGNAL_COLOR_RED;
			}
			Serial.print("LED:");
			Serial.println(g_trafic_signal_color);
		}
	}
	else if(strcmp(topic, mqttTopic_Command) == 0) {
		if(!object["Stop"].isNull()) {
			int getstr = 's';
			xQueueSend(g_xQueue_Serial, &getstr, 100);
		}
		if(!object["MotorSpeed"].isNull()) {
			int speed = object["MotorSpeed"].as<int>();
			if((MOTOR_SPEED_MIN<=speed) && (speed<=MOTOR_SPEED_MAX)) {
				g_motor_speed = speed;
			}
		}
		if(!object["MotorSpeedLR"].isNull()) {
			int left = object["MotorSpeedLR"][0].as<int>();
			int right = object["MotorSpeedLR"][1].as<int>();
			int left_l = object["MotorSpeedLR"][2].as<int>();
			int right_l = object["MotorSpeedLR"][3].as<int>();
			if((MOTOR_SPEED_MIN<=left) && (left<=MOTOR_SPEED_MAX)) {
				g_motor_speed_on_left_turn = left;
			}
			if((MOTOR_SPEED_MIN<=right) && (right<=MOTOR_SPEED_MAX)) {
				g_motor_speed_on_right_turn = right;
			}
			if((0<=left_l) && (left_l<=MOTOR_SPEED_MAX)) {
				g_lr_level_on_left_turn = left_l;
			}
			if((0<=right_l) && (right_l<=MOTOR_SPEED_MAX)) {
				g_lr_level_on_right_turn = right_l;
			}
		}
	}
	else if(strcmp(topic, mqttTopic_Query) == 0) {
		if(!object["Id"].isNull()) {
			const char* code = object["Id"];
			if(strcmp(code, "Param") == 0) {
				doc["MtSpd"] = g_motor_speed;
				doc["MtSpd_LT"] = g_motor_speed_on_left_turn;
				doc["MtSpd_RT"] = g_motor_speed_on_right_turn;
				doc["MtLv_LT"] = g_lr_level_on_left_turn;
				doc["MtLv_RT"] = g_lr_level_on_right_turn;
				char payload[200];
				serializeJson(doc, payload);
				g_mqtt_client.publish(mqttTopic_Param, payload);
			}
		}
	}
}

// MQTT Client が接続できなかったら接続できるまで再接続を試みるための MQTT_reconnect 関数
void MQTT_reconnect() {
  // Loop until we're reconnected
  while (!g_mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (g_mqtt_client.connect(mqttDeviceId, mqttUser, mqttPassword)) {
      Serial.println("connected");
      g_mqtt_client.subscribe(mqttTopic_Signal);
      g_mqtt_client.subscribe(mqttTopic_Command);
      g_mqtt_client.subscribe(mqttTopic_Query);
    } else {
      Serial.print("failed, rc=");
      Serial.print(g_mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5sec before retrying
      vTaskDelay(5000);
    }
  }
}

void MQTT_publish_query(const char* id)
{
	// JSONフォーマット作成
	StaticJsonDocument<64> doc;
	doc["Id"] = id;
	char payload[64];
	serializeJson(doc, payload);
	// MQTT brokerへpublish
	g_mqtt_client.publish(mqttTopic_Query, payload);
}


void LOG_output(const char str[], int level=99)
{
	if(g_log_level <= level) {
		Serial.println(str);
	}	
}

void SERVO_set_angle(int angle)
{
	if(angle < -90.0) {
		angle = -90.0;
	}
	if(angle > 90.0) {
		angle = 90.0;
	}
	angle -= 10;

	int val = (int)(servo_coeff_a*angle + servo_coeff_b);
	ledcWrite(DAC_CH_SERVO, val);
	vTaskDelay(500);
	ledcWrite(DAC_CH_SERVO, 0);	
}

float SR04_get_distance(unsigned long max_dist=100)   // return:[cm](timeout発生時は10000.0)  max_dist:[cm]
{
	// Pulse発生
	digitalWrite(IO_PIN_US_TRIG, LOW);   
	delayMicroseconds(2);
	digitalWrite(IO_PIN_US_TRIG, HIGH);  
	delayMicroseconds(20);
	digitalWrite(IO_PIN_US_TRIG, LOW);   
	
	// 反射波到達までの時間計測(timeout発生時は0)
	unsigned long duration = pulseIn(IO_PIN_US_ECHO, HIGH, max_dist*58);  
	float dist;
	if(duration == 0) {
		dist = 10000.0;
	}
	else {
		dist = duration * 0.017; // [cm]
	}

	return dist;
}  

void MOTOR_set_speed_left(int dir, int speed)
{
	ledcWrite(DAC_CH_MOTOR_A, speed);  
	g_motor_speed_right = speed;

	if(dir == MOTOR_DIR_FWD) {
		digitalWrite(IO_PIN_MOTOR_1,HIGH);
		digitalWrite(IO_PIN_MOTOR_2,LOW);
	}
	else if(dir == MOTOR_DIR_REV) {
		digitalWrite(IO_PIN_MOTOR_1,LOW);
		digitalWrite(IO_PIN_MOTOR_2,HIGH);
	}
}

void MOTOR_set_speed_right(int dir, int speed)
{
	ledcWrite(DAC_CH_MOTOR_B, speed);  
	g_motor_speed_left = speed;

	if(dir == MOTOR_DIR_FWD) {
		digitalWrite(IO_PIN_MOTOR_3,LOW);
		digitalWrite(IO_PIN_MOTOR_4,HIGH);
	}
	else if(dir == MOTOR_DIR_REV) {
		digitalWrite(IO_PIN_MOTOR_3,HIGH);
		digitalWrite(IO_PIN_MOTOR_4,LOW);
	}
}

int RoboCar_us_get_distance()   // return:[cm](timeout発生時は10000)
{
	float dist; 
	float dist_sum = 0.0; 
	int i;
	for(i = 0; i < 5; i ++) {
		dist = SR04_get_distance(50); 
		if(dist > 9000.0) {
			// timeoutが一度でも計測されたときは前方に何もないと見なす
			return	10000;
		}
		dist_sum += dist;
	}
	
	return	(int)(dist_sum*0.2); // 5で割る代わりに0.2を掛ける
}

void RoboCar_move_forward(int speed)
{
	MOTOR_set_speed_right(MOTOR_DIR_FWD, speed);
	MOTOR_set_speed_left(MOTOR_DIR_FWD, speed);
	
	g_state_motor = STATE_MOTOR_MOVING_FORWARD;
	g_cur_state = STATE_GO_FORWARD;
	
	LOG_output("go forward", 1);
}

void RoboCar_move_backward(int speed)
{
	MOTOR_set_speed_right(MOTOR_DIR_REV, speed);
	MOTOR_set_speed_left(MOTOR_DIR_REV, speed);
	
	g_state_motor = STATE_MOTOR_MOVING_BACKWARD;
	
	LOG_output("go backward", 1);
}

void RoboCar_turn_left(int dir, int speed, int level)
{
	if(level < 0) {
		level = 0;
	}
	int speed_right = speed;
	int speed_left = speed - level;
	if(speed_left < 0) {
		speed_left = 0;
	}
		
	MOTOR_set_speed_right(dir, speed_right);
	MOTOR_set_speed_left(dir, speed_left);

	g_state_motor = (dir==MOTOR_DIR_FWD) ? STATE_MOTOR_TURNING_LEFT : STATE_MOTOR_TURNING_LEFT_BACK;
	
	LOG_output("turn left!", 1);
}

void RoboCar_turn_right(int dir, int speed, int level)
{
	if(level < 0) {
		level = 0;
	}
	int speed_right = speed - level;
	int speed_left = speed;
	if(speed_right < 0) {
		speed_right = 0;
	}
	MOTOR_set_speed_right(dir, speed_right);
	MOTOR_set_speed_left(dir, speed_left);
	
	g_state_motor = (dir==MOTOR_DIR_FWD) ? STATE_MOTOR_TURNING_RIGHT : STATE_MOTOR_TURNING_RIGHT_BACK;
	
	LOG_output("turn right!", 1);
}

void RoboCar_rotate_left(int speed)
{
	MOTOR_set_speed_right(MOTOR_DIR_FWD, speed);
	MOTOR_set_speed_left(MOTOR_DIR_REV, speed);
	
	g_state_motor = STATE_MOTOR_ROTATING_LEFT;
	g_cur_state = STATE_ROTATO_LEFT;
	
	LOG_output("rotate left!", 1);
}

void RoboCar_rotate_right(int speed)
{
	MOTOR_set_speed_right(MOTOR_DIR_REV, speed);
	MOTOR_set_speed_left(MOTOR_DIR_FWD, speed);
	
	g_state_motor = STATE_MOTOR_ROTATING_RIGHT;
	g_cur_state = STATE_ROTATO_RIGHT;
	
	LOG_output("rotate right!", 1);
}

void RoboCar_stop()
{
	MOTOR_set_speed_left(0, 0);
	MOTOR_set_speed_right(0, 0);

	g_state_motor = STATE_MOTOR_STOP;
	g_cur_state = STATE_WAIT;

	LOG_output("Stop!", 1);
}

int LTrace_create_event()
{
	int event;
	int sensor = ((LT_L&0x1)<<2) | ((LT_M&0x1)<<1) | ((LT_R&0x1)<<0);
	
	if(sensor == 0x7) {
		if(g_trafic_signal_color == SIGNAL_COLOR_RED) {
			event = TRACK_EVENT_OOO_RED;
		}
		else if(g_trafic_signal_color == SIGNAL_COLOR_YELLOW) {
			event = TRACK_EVENT_OOO_YELLOW;
		}
		else {
			event = TRACK_EVENT_OOO_GREEN;
		}
	}
	else {
		event = sensor;
	}
	
	return	event;
}

void osTask_sensor(void* param)
{
	int error;
	float	acc_x, acc_y, acc_z;
	float	gyro_x, gyro_y, gyro_z;
	BaseType_t xStatus;
	byte rc;
	float pre_abs_axl = 0;

	xSemaphoreGive(g_xMutex);
	for(;;) {
		vTaskDelay(50);

		// 加速度、角速度、温度を取得
		error = MPU6050_get_all(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z, &g_temperature);
		// 衝撃検出
		float abs_axl = (acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
		float diff_axl = abs_axl - pre_abs_axl;
		if(diff_axl < 0) {
			diff_axl = -diff_axl;
		}
		pre_abs_axl = abs_axl;

		// 照度・近接センサの値を取得
		unsigned short ps_val;
		float als_val;
		rc = rpr0521rs.get_psalsval(&ps_val, &als_val);
		if(rc == 0) {
		}

		if(diff_axl < 0.5) {
			digitalWrite(IO_PIN_LED,LOW);
		}
		else {
			digitalWrite(IO_PIN_LED,HIGH);
			if(diff_axl > 1.5) { // 衝撃が大きかった時はstopさせる
				int getstr = 's';
				xQueueSend(g_xQueue_Serial, &getstr, 100);
			}
		}

		if(als_val > 10.0) {
			digitalWrite(IO_PIN_LED2,LOW);
		}
		else {
			digitalWrite(IO_PIN_LED2,HIGH);
		}

		// ▼▼▼ [排他制御区間]開始 ▼▼▼
		xStatus = xSemaphoreTake(g_xMutex, 0);
		if(diff_axl > g_max_diff_axl) {
			g_max_diff_axl = diff_axl;
		}
		g_ps_val = ps_val;
		g_als_val = als_val;
		xSemaphoreGive(g_xMutex);
		// ▲▲▲ [排他制御区間]開始 ▲▲▲
	}

}

void osTask_WiFi(void* param)
{
	for(;;) {
		vTaskDelay(200);

		if (!g_mqtt_client.connected()) {
			MQTT_reconnect();
		}
		g_mqtt_client.loop();
	}

}

void osTask_disp(void* param)
{
	BaseType_t xStatus;
	float	temperature = 0.0;
	float	max_diff_axl = 0.0;
	unsigned short ps_val;
	float als_val;
	portTickType wakeupTime = xTaskGetTickCount();
	int		count = 0;

	xSemaphoreGive(g_xMutex);
	for(;;) {
		vTaskDelayUntil(&wakeupTime, 3000);
		
		// ▼▼▼ [排他制御区間]開始 ▼▼▼
		xStatus = xSemaphoreTake(g_xMutex, 0);
		temperature = g_temperature;
		max_diff_axl = g_max_diff_axl;
		ps_val = g_ps_val;
		als_val = g_als_val;
		g_max_diff_axl = 0.0; // リセット
		xSemaphoreGive(g_xMutex);
		// ▲▲▲ [排他制御区間]開始 ▲▲▲
		
		g_proficiency_score += max_diff_axl;

		// 熟練度表示
		count ++;
		if(count > 10) {
			count = 0;
			Serial.print("Score = ");
			Serial.print(g_proficiency_score, 2);
			Serial.println();
		}

		//----- センサ値をMTQQ brokerへpublis
		// JSONフォーマット作成
		StaticJsonDocument<200> doc_in;
		doc_in["AxlDiff"] = max_diff_axl;
		doc_in["Temp"] = temperature;
		doc_in["Bright"] = als_val;
		doc_in["Prox"] = ps_val;
		doc_in["P-Score"] = g_proficiency_score;
		// payloadにセットさ_れたJSON形式メッセージをpublish
		char payload[200];
		serializeJson(doc_in, payload);
		g_mqtt_client.publish(mqttTopic_Sensor, payload);
		
		if(g_is_signal_recieved == 0) {
			// 信号機情報を一度も受信していない場合は要求する
			MQTT_publish_query("Signal");
		}
	}
	
}

void osTask_robo_car(void* param)
{
	portTickType 	wait_tick = portMAX_DELAY;

	for(;;) {
		int getstr = 0;
		BaseType_t	xStatus = xQueueReceive(g_xQueue_Serial, &getstr, wait_tick);
		//----- Mode切り替え -----
		if(getstr == 'a') {
			LOG_output("Auto Drive Mode", 5);
			g_ctrl_mode = CTRLMODE_AUTO_DRIVE;
			wait_tick = portMAX_DELAY;
			RoboCar_stop();
		}
		else if(getstr == 'm') {
			LOG_output("manual Drive Mode", 5);
			g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;
			wait_tick = portMAX_DELAY;
			RoboCar_stop();
		}
		else if(getstr == 't') {
			LOG_output("Line Trace Mode", 5);
			g_ctrl_mode = CTRLMODE_LINE_TRACKING;
			wait_tick = 10/portTICK_RATE_MS; // 10[ms]
			RoboCar_move_forward(carSpeed);
		}

		//----- RoboCar制御 -----
		if(g_ctrl_mode == CTRLMODE_MANUAL_DRIVE) { // ----- Manual Drive Mode
			if(getstr=='f') {
				RoboCar_move_forward(g_motor_speed);
			}
			else if(getstr=='b') {
				RoboCar_move_backward(g_motor_speed);
			}
			else if(getstr=='l') {
				RoboCar_rotate_left(g_motor_speed);
			}
			else if(getstr=='r') {
				RoboCar_rotate_right(g_motor_speed);
			}
			else if(getstr=='L') {
				RoboCar_turn_left(MOTOR_DIR_FWD, g_motor_speed_on_left_turn, g_lr_level_on_left_turn);
			}
			else if(getstr=='R') {
				RoboCar_turn_right(MOTOR_DIR_FWD, g_motor_speed_on_right_turn, g_lr_level_on_right_turn); // 微調整
			}
			else if(getstr=='C') {
				RoboCar_turn_left(MOTOR_DIR_REV, g_motor_speed_on_left_turn, g_lr_level_on_left_turn);
			}
			else if(getstr=='D') {
				RoboCar_turn_right(MOTOR_DIR_REV, g_motor_speed_on_right_turn, g_lr_level_on_right_turn);
			}
			else if(getstr=='s') {
				RoboCar_stop();		 
			}
		}
		else if(g_ctrl_mode == CTRLMODE_LINE_TRACKING) {  // ----- Line Trace Mode
			if(getstr=='s') {
				g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;
				RoboCar_stop();		 
			}		
			else {
				int event = LTrace_create_event();
				int next_state = g_next_event_table[event][g_cur_state];
				if(next_state != g_cur_state) {
					if(next_state == STATE_ROTATO_LEFT) {
						Serial.println("STATE_ROTATO_LEFT");
						RoboCar_rotate_left(carSpeed);
					}
					else if(next_state == STATE_GO_FORWARD) {
						Serial.println("STATE_GO_FORWARD");
						RoboCar_move_forward(carSpeed);
					}
					else if(next_state == STATE_ROTATO_RIGHT) {
						Serial.println("STATE_ROTATO_RIGHT");
						RoboCar_rotate_right(carSpeed);
					}
					else if(next_state == STATE_WAIT) {
						Serial.println("STATE_WAIT");
						RoboCar_stop();
					}
				}
			}
		}
		else if(g_ctrl_mode == CTRLMODE_AUTO_DRIVE) { // ----- Manual Drive Mode
			int right_distance = 0, left_distance = 0, middle_distance = 0;

			if(getstr=='s') {
				g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;
				RoboCar_stop();		 
			}		
			else {
				middle_distance = RoboCar_us_get_distance();

				if(middle_distance <= g_stop_distance) {     
					RoboCar_stop();
					vTaskDelay(500); 	  
					SERVO_set_angle(-80);  
					vTaskDelay(1000);      
					right_distance = RoboCar_us_get_distance();

					vTaskDelay(500);
					SERVO_set_angle(0);              
					vTaskDelay(1000);                                                  
					SERVO_set_angle(80);              
					vTaskDelay(1000); 
					left_distance = RoboCar_us_get_distance();

					vTaskDelay(500);
					SERVO_set_angle(0);              
					vTaskDelay(1000);
					if((right_distance<=g_stop_distance) && (left_distance<=g_stop_distance)) {
						RoboCar_move_backward(g_motor_speed);
						vTaskDelay(180);
					}
					else if((right_distance>9000) && (left_distance>9000)) {
						RoboCar_rotate_right(180); // L/Rのどちらでもよい
						vTaskDelay(500);
					}
					else if(right_distance>left_distance) {
						RoboCar_rotate_right(180);
						vTaskDelay(500);
					}
					else if(right_distance<left_distance) {
						RoboCar_rotate_left(150);
						vTaskDelay(500);
					}
					else {
						RoboCar_move_forward(g_motor_speed);
					}
				}  
				else {
					RoboCar_move_forward(g_motor_speed);
				}
			}
		}
	}
	
}

void osTask_Serial(void* param)
{
	BaseType_t xStatus;

	for(;;) {
		vTaskDelay(30);
		
		int getstr = -1;
		if (Serial.available() > 0) { // 受信したデータが存在する
			// Serial Portから一文字読み込み
			getstr = Serial.read();
		}

		// 障害物検出
		int dist = RoboCar_us_get_distance();
	    if(dist <= g_stop_distance) {
			if((g_state_motor==STATE_MOTOR_MOVING_FORWARD) || 
				(g_state_motor==STATE_MOTOR_TURNING_RIGHT) ||
				(g_state_motor==STATE_MOTOR_TURNING_LEFT)) 
			{
				getstr = 's';
			}
		}

		if(getstr != -1) {
	        xStatus = xQueueSend(g_xQueue_Serial, &getstr, 100);
	        if(xStatus != pdPASS) {
				Serial.println(getstr);
			}
		}
	}

}

void setup()
{
	byte rc;

	g_ctrl_mode = CTRLMODE_MANUAL_DRIVE;

	// Serial Port(USB) 初期設定(115200bpsだとBluetoothが正しく通信できない)
	Serial.begin(9600);
	// I2C 初期設定
	Wire.begin(IO_PIN_SDA, IO_PIN_SCL);
	// WiFi初期設定
	WiFi.begin(ssid, password);
	int cnt = 0;
	int is_success = true;
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print("Connecting to ");
		Serial.println(ssid);
		cnt ++;
		if(cnt > 30) {
			is_success = false;
			break;
		}
	}
	if(is_success) {
		Serial.println("Connected to the WiFi network.");
		// WiFi アクセスポイントから付与されたIPアドレス
		Serial.print("WiFi connected IP address: ");
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


	pinMode(IO_PIN_US_ECHO, INPUT);    
	pinMode(IO_PIN_US_TRIG, OUTPUT);  
	pinMode(IO_PIN_LED, OUTPUT);
	pinMode(IO_PIN_LED2, OUTPUT);
	pinMode(IO_PIN_MOTOR_1,OUTPUT);
	pinMode(IO_PIN_MOTOR_2,OUTPUT);
	pinMode(IO_PIN_MOTOR_3,OUTPUT);
	pinMode(IO_PIN_MOTOR_4,OUTPUT);
	pinMode(IO_PIN_MOTOR_ENA,OUTPUT);
	pinMode(IO_PIN_MOTOR_ENB,OUTPUT);
	pinMode(IO_PIN_SERVO,OUTPUT);
	pinMode(IO_PIN_LINETRACK_LEFT, INPUT);    
	pinMode(IO_PIN_LINETRACK_CENTER, INPUT);    
	pinMode(IO_PIN_LINETRACK_RIGHT, INPUT);    

	// Motorの初期設定(DACへの割当て)
	ledcSetup(DAC_CH_MOTOR_A, 980, 8);
	ledcSetup(DAC_CH_MOTOR_B, 980, 8);
	ledcAttachPin(IO_PIN_MOTOR_ENA, DAC_CH_MOTOR_A);
	ledcAttachPin(IO_PIN_MOTOR_ENB, DAC_CH_MOTOR_B);

	RoboCar_stop();

	// Servoの初期設定
	float servo_min = 26.0;  // (26/1024)*20ms ≒ 0.5 ms  (-90°)
	float servo_max = 123.0; // (123/1024)*20ms ≒ 2.4 ms (+90°)
	servo_coeff_a = (servo_max-servo_min)/180.0;
	servo_coeff_b = (servo_max+servo_min)/2.0;
	ledcSetup(DAC_CH_SERVO, 50, 10);  // 0ch 50 Hz 10bit resolution
	ledcAttachPin(IO_PIN_SERVO, DAC_CH_SERVO); 

    SERVO_set_angle(0);//********xxxxx setservo position according to scaled value
    delay(500); 

  	// 照度・近接センサの初期設定
	rc = rpr0521rs.init();
	if(rc != 0) {
		Serial.println("[Error] cannot initialize RPR-0521.");
	}

	// 加速度センサ初期化
	MPU6050_init(&Wire);

	// Task間通信用のQueue生成
	g_xQueue_Serial = xQueueCreate(8, sizeof(int32_t));
	// Task生成(優先度は数が大きいほど優先度高)
	g_xMutex = xSemaphoreCreateMutex();
	xTaskCreatePinnedToCore(osTask_sensor, "osTask_sensor", 2048, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(osTask_WiFi, "osTask_WiFi", 2048, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(osTask_disp, "osTask_disp", 2048, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(osTask_robo_car, "osTask_robo_car", 2048, NULL, 3, NULL, 0);
	xTaskCreatePinnedToCore(osTask_Serial, "osTask_Serial", 2048, NULL, 4, NULL, 0);

	// LED点灯
	for(int i = 0; i < 3; i ++) {
		digitalWrite(IO_PIN_LED, HIGH);
		delay(1000);
		digitalWrite(IO_PIN_LED, LOW);
		delay(1000);
	}

	// payloadにセットされたJSON形式メッセージを投稿
	char text[200];
	sprintf(text, "{\"Init\":\"Pass\"}");
	g_mqtt_client.publish(mqttTopic_Status, text);

	Serial.println("Completed setup program successfully.");
}

void loop()
{


}


