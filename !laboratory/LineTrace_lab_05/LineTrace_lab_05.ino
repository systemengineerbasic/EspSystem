//==============================================================================
//
//		LineTrace_lab_05.ino
//			
//			State Machineを用いたLineTrace制御
//			黒い横線を検出すると停車する
//
//==============================================================================
#include "BluetoothSerial.h"
#include "my_cmd.h"


// Motor Pin
#define	IO_PIN_MOTOR_1			(14)
#define	IO_PIN_MOTOR_2			(12)
#define	IO_PIN_MOTOR_3			(13)
#define	IO_PIN_MOTOR_4			(23)
#define	IO_PIN_MOTOR_ENA		(16)
#define	IO_PIN_MOTOR_ENB		(27)
// Line Tracking Sensor Pin
#define IO_PIN_LINETRACK_LEFT	(26)
#define IO_PIN_LINETRACK_CENTER	(17)
#define IO_PIN_LINETRACK_RIGHT	(39)

// Motor Speed
#define	MOTOR_SPEED				(150)

//---------------------- Trafic Signal ----------------------
#define	SIGNAL_COLOR_BLACK			(0)
#define	SIGNAL_COLOR_BLUE			(1)
#define	SIGNAL_COLOR_YELLOW			(2)
#define	SIGNAL_COLOR_RED			(3)


// State Machine Event
enum {
	TRACK_EVENT_NONE = -1,
	TRACK_EVENT_XXX	= 0,
	TRACK_EVENT_XXO,
	TRACK_EVENT_XOX,
	TRACK_EVENT_XOO,
	TRACK_EVENT_OXX,
	TRACK_EVENT_OXO,
	TRACK_EVENT_OOX,
	TRACK_EVENT_OOO_RED,
	TRACK_EVENT_OOO_BLUE,

	TRACK_EVENT_NUM
};

// State Machine State
enum {
	STATE_ROTATO_LEFT=0,	
	STATE_GO_FORWARD,	
	STATE_ROTATO_RIGHT,	
	STATE_STOP,	

	STATE_NUM	
};

// 状態遷移テーブル
int g_next_state_table[TRACK_EVENT_NUM][STATE_NUM] =
{
//              Left                    Foward                  Right                   Stop
/*XXX*/         STATE_ROTATO_LEFT,      STATE_ROTATO_LEFT,      STATE_ROTATO_RIGHT,     STATE_STOP,
/*XXO*/         STATE_ROTATO_RIGHT,     STATE_ROTATO_RIGHT,     STATE_ROTATO_RIGHT,     STATE_STOP,
/*XOX*/         STATE_GO_FORWARD,       STATE_GO_FORWARD,       STATE_GO_FORWARD,       STATE_STOP,
/*XOO*/         STATE_GO_FORWARD,       STATE_GO_FORWARD,       STATE_ROTATO_RIGHT,     STATE_STOP,    
/*OXX*/         STATE_ROTATO_LEFT,      STATE_ROTATO_LEFT,      STATE_ROTATO_LEFT,      STATE_STOP,
/*OXO*/         STATE_ROTATO_LEFT,      STATE_ROTATO_RIGHT,     STATE_ROTATO_RIGHT,     STATE_STOP,    
/*OOX*/         STATE_ROTATO_LEFT,      STATE_GO_FORWARD,       STATE_GO_FORWARD,       STATE_STOP,    
/*OOO_RED*/     STATE_STOP,             STATE_STOP,             STATE_STOP,             STATE_STOP,
/*OOO_BLUE*/    STATE_ROTATO_LEFT,      STATE_GO_FORWARD,       STATE_ROTATO_RIGHT,     STATE_GO_FORWARD,
};


int g_cur_state;    // Current state
int	g_trafic_signal_color = SIGNAL_COLOR_RED;
int g_motor_speed = 150;

BluetoothSerial SerialBT;
Stream* g_pSerial=&Serial;


//=====================================
// Command procedures
//=====================================
void    _cmd__serial(int argc, char* argv[])
{
    if(argc > 1) {
        if(strcmp(argv[1], "bt")==0) {
            g_pSerial = &SerialBT;
        }
        else if(strcmp(argv[1], "usb")==0) {
            g_pSerial = &Serial;
        }
    }
        
}

void    _cmd__signal(int argc, char* argv[])
{
    if(argc > 1) {
        if(strcmp(argv[1], "r")==0) {
            g_trafic_signal_color = SIGNAL_COLOR_RED;
            g_pSerial->println("Red");
        }
        else if(strcmp(argv[1], "y")==0) {
            g_trafic_signal_color = SIGNAL_COLOR_YELLOW;
            g_pSerial->println("Yellow");
        }
        else if(strcmp(argv[1], "b")==0) {
            g_trafic_signal_color = SIGNAL_COLOR_BLUE;
            g_pSerial->println("Blue");
        }
    }
}

//=====================================
// Command table
//=====================================
T_command_info  g_command_table[] = {
    {"serial",      _cmd__serial},
    {"signal",      _cmd__signal},
    // The last line must be NULL
    {NULL,          NULL},
};


//===================================================================
// MODULE   : read_sensor_*()
// FUNCTION : センサーを読む(Left/Center/Right)
// RETURN   : なし
//===================================================================
int read_sensor_L()
{
	return	!digitalRead(IO_PIN_LINETRACK_LEFT);
}
int read_sensor_C()
{
	return	!digitalRead(IO_PIN_LINETRACK_CENTER);
}
int read_sensor_R()
{
	return	!digitalRead(IO_PIN_LINETRACK_RIGHT);
}

//===================================================================
// MODULE   : forward()
// FUNCTION : 直進
// RETURN   : なし
//===================================================================
void forward()
{
	ledcWrite(0, g_motor_speed);  
	ledcWrite(1, g_motor_speed);  
	digitalWrite(IO_PIN_MOTOR_1, HIGH);
	digitalWrite(IO_PIN_MOTOR_2, LOW);
	digitalWrite(IO_PIN_MOTOR_3, LOW);
	digitalWrite(IO_PIN_MOTOR_4, HIGH);
	
	g_cur_state = STATE_GO_FORWARD;
	
	g_pSerial->println("go forward!");
}

//===================================================================
// MODULE   : left()
// FUNCTION : 左回転
// RETURN   : なし
//===================================================================
void left()
{
	ledcWrite(0, g_motor_speed);  
	ledcWrite(1, g_motor_speed);  
	digitalWrite(IO_PIN_MOTOR_1, LOW);
	digitalWrite(IO_PIN_MOTOR_2, HIGH);
	digitalWrite(IO_PIN_MOTOR_3, LOW);
	digitalWrite(IO_PIN_MOTOR_4, HIGH);

	g_cur_state = STATE_ROTATO_LEFT;

	g_pSerial->println("go left!");
}

//===================================================================
// MODULE   : right()
// FUNCTION : 右回転
// RETURN   : なし
//===================================================================
void right()
{
	ledcWrite(0, g_motor_speed);  
	ledcWrite(1, g_motor_speed);  
	digitalWrite(IO_PIN_MOTOR_1, HIGH);
	digitalWrite(IO_PIN_MOTOR_2, LOW);
	digitalWrite(IO_PIN_MOTOR_3, HIGH);
	digitalWrite(IO_PIN_MOTOR_4, LOW); 

	g_cur_state = STATE_ROTATO_RIGHT;

	g_pSerial->println("go right!");
} 

//===================================================================
// MODULE   : stop()
// FUNCTION : 停止
// RETURN   : なし
//===================================================================
void stop()
{
	ledcWrite(0, 0);  
	ledcWrite(1, 0);  

	g_cur_state = STATE_STOP;

	g_pSerial->println("Stop!");
} 

//===================================================================
// MODULE   : create_event()
// FUNCTION : State Machineの入力Event生成
// RETURN   : Event
//===================================================================
int create_event()
{
    // Get tracking sensor value
	int sensor_L = read_sensor_L();
	int sensor_C = read_sensor_C();
	int sensor_R = read_sensor_R();
	int sensor = ((sensor_L&0x1)<<2) | ((sensor_C&0x1)<<1) | ((sensor_R&0x1)<<0);
	
	int event;
	if(sensor == 7) { // "OOO"
        if(g_trafic_signal_color == SIGNAL_COLOR_BLUE) {
            event = TRACK_EVENT_OOO_BLUE;
        }
        else {
            event = TRACK_EVENT_OOO_RED;
        }
    }
    else { // except for "OOO"
        event = sensor;
    }
	
	return	event;
}

//===================================================================
// MODULE   : get_next_state()
// FUNCTION : State Machineのeventに対する遷移後の状態を取得
// RETURN   : Next State
//===================================================================
int get_next_state(int cur_state, int event) 
{
	if((event < 0) || (TRACK_EVENT_NUM <= event)) {
		// Invalid event ID
		return	g_cur_state;
	}
	
	if((cur_state < 0) || (STATE_NUM <= cur_state)) {
		// Invalid state ID
		return	g_cur_state;
	}
	
	return	g_next_state_table[event][cur_state];
}

//===================================================================
// MODULE   : Task_line_trace()
// FUNCTION : [Task] ライントレースカー制御
// RETURN   : なし
//===================================================================
void Task_line_trace(void* param)
{
	for(;;) {
		vTaskDelay(10);

		// Create event
		int event = create_event();
		if(event != -1) { // if event occurs
			// Get next state
			int next_state = get_next_state(g_cur_state, event);
			
			if(next_state != g_cur_state) { // Next state is different from current state => State transition occurs
				// Process accoring to state
				if(next_state == STATE_ROTATO_LEFT) {
					left();
				}
				else if(next_state == STATE_GO_FORWARD) {
					forward();
				}
				else if(next_state == STATE_ROTATO_RIGHT) {
					right();
				}
				else if(next_state == STATE_STOP) {
					stop();
				}
			}
		}
	}
	
}

void Task_serial_cmd(void* param)
{
    char    command_line[256];
    int     cmd_index = 0;
    for(;;) {
        vTaskDelay(10);

        if(g_pSerial->available() > 0) { // received data
            char getstr = g_pSerial->read(); // Read data from serial-port
            command_line[cmd_index] = getstr;
            cmd_index ++;
            if(getstr == '\n') { // Detect "LF"(enter key)
                command_line[cmd_index-1] = '\0';
                cmd_index = 0;
                
                // Parse and execute command
                parse_and_exec_cmd(command_line, g_command_table);
            }
        }
        
    }
    
}


//===================================================================
// MODULE   : setup()
// FUNCTION : 初期設定
// RETURN   : なし
//===================================================================
void setup()
{
	// Serial通信の初期化
	Serial.begin(115200);
	
	// I/O Pin属性設定
	pinMode(IO_PIN_LINETRACK_LEFT, INPUT);
	pinMode(IO_PIN_LINETRACK_CENTER, INPUT);
	pinMode(IO_PIN_LINETRACK_RIGHT, INPUT);
	pinMode(IO_PIN_MOTOR_ENA, OUTPUT);
	pinMode(IO_PIN_MOTOR_ENB, OUTPUT);
	pinMode(IO_PIN_MOTOR_1, OUTPUT);
	pinMode(IO_PIN_MOTOR_2, OUTPUT);
	pinMode(IO_PIN_MOTOR_3, OUTPUT);
	pinMode(IO_PIN_MOTOR_4, OUTPUT);

	// DAC
	ledcSetup(0, 980, 8);
	ledcSetup(1, 980, 8);
	ledcAttachPin(IO_PIN_MOTOR_ENA, 0);
	ledcAttachPin(IO_PIN_MOTOR_ENB, 1);

	xTaskCreatePinnedToCore(Task_line_trace, "Task_line_trace", 2048, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(Task_serial_cmd, "Task_serial_cmd", 2048, NULL, 1, NULL, 0);
	
	// 最初は直進
	forward();
}

//===================================================================
// MODULE   : loop()
// FUNCTION : メインループ
// RETURN   : なし
//===================================================================
void loop() 
{
	// 処理は全てTaskで行うのでメインループ内では何もしない
}

