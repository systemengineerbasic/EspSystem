//==============================================================================
//
//  LineTrace_lab_05.ino
//
//      Line tracking by RoboCar.
//      If the car will stop when acrossing black line.
//
//==============================================================================
#include "BluetoothSerial.h"
#include "my_cmd.h"
#include <Wire.h>
#include <RPR-0521RS.h>


//---------------------- PIN ----------------------
// Motor
#define IO_PIN_MOTOR_1          (14)
#define IO_PIN_MOTOR_2          (12)
#define IO_PIN_MOTOR_3          (13)
#define IO_PIN_MOTOR_4          (23)
#define IO_PIN_MOTOR_ENA        (16)
#define IO_PIN_MOTOR_ENB        (27)
// Line Tracking Sensor
#define IO_PIN_LINETRACK_LEFT   (26)
#define IO_PIN_LINETRACK_CENTER (17)
#define IO_PIN_LINETRACK_RIGHT  (39)

//---------------------- D/A converter channel ----------------------
#define DAC_CH_MOTOR_A          (0)
#define DAC_CH_MOTOR_B          (1)
#define DAC_CH_SERVO            (2)

//---------------------- Trafic Signal ----------------------
#define SIGNAL_COLOR_BLACK      (0)
#define SIGNAL_COLOR_BLUE       (1)
#define SIGNAL_COLOR_YELLOW     (2)
#define SIGNAL_COLOR_RED        (3)

//---------------------- RoboCar ----------------------
// Motor derection
#define MOTOR_DIR_STOP          (0)
#define MOTOR_DIR_FWD           (1)
#define MOTOR_DIR_REV           (2)

//---------------------- State machine ----------------------
// State Machine Event
enum {
    TRACK_EVENT_NONE = -1,
    TRACK_EVENT_XXX = 0,
    TRACK_EVENT_XXO,
    TRACK_EVENT_XOX,
    TRACK_EVENT_XOO,
    TRACK_EVENT_OXX,
    TRACK_EVENT_OXO,
    TRACK_EVENT_OOX,
    TRACK_EVENT_OOO_RED,
    TRACK_EVENT_TURNED_BLUE,

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

// Table of state 
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
/*Turned blue*/ STATE_ROTATO_LEFT,      STATE_GO_FORWARD,       STATE_ROTATO_RIGHT,     STATE_GO_FORWARD,
};


BluetoothSerial SerialBT;

int g_cur_state;    // Current state
int g_trafic_signal_color = SIGNAL_COLOR_RED;
int g_motor_speed = 200;

Stream* g_pSerial=&Serial; // Selected serial port (USB is default)

SemaphoreHandle_t g_xMutex_Signal = NULL;   // for critical section for signal color

RPR0521RS rpr0521rs;

#define PIN_SDA SDA
#define PIN_SCL SCL



//===================================================================
// Command procedures
//===================================================================
void _cmd__test(int argc, char* argv[])
{
    g_pSerial->println("Test OK.");
}

void _cmd__serial(int argc, char* argv[])
{
    if(argc > 1) {
        if(strcmp(argv[1], "bt")==0) {
            g_pSerial = &SerialBT;
            g_pSerial->println("Bluetooth was selected.");
        }
        else if(strcmp(argv[1], "usb")==0) {
            g_pSerial = &Serial;
            g_pSerial->println("USB was selected.");
        }
    }
}

void _cmd__speed(int argc, char* argv[])
{
    if(argc > 1) {
        int speed = atoi(argv[1]);
        g_motor_speed = speed;
    }
}

void _cmd__signal(int argc, char* argv[])
{
    if(argc > 1) {
        int color = SIGNAL_COLOR_BLACK;
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
        
        if(color != SIGNAL_COLOR_BLACK) {
            xSemaphoreTake(g_xMutex_Signal, portMAX_DELAY);
            // ▼▼▼ Start critical section ▼▼▼
            g_trafic_signal_color = color;
            // ▲▲▲ End critical section ▲▲▲
            xSemaphoreGive(g_xMutex_Signal);
        }
    }
}

//===================================================================
// Command table
//===================================================================
T_command_info  g_command_table[] = {
    {"test",        _cmd__test},
    {"serial",      _cmd__serial},
    {"speed",       _cmd__speed},
    {"signal",      _cmd__signal},
    // The last line must be NULL
    {NULL,          NULL},
};


//===================================================================
// MODULE   : read_sensor_L/C/R()
// FUNCTION : Read photo sensor(Left/Center/Right)
// RETURN   : N/A
//===================================================================
int read_sensor_L()
{
    return !digitalRead(IO_PIN_LINETRACK_LEFT);
}
int read_sensor_C()
{
    return !digitalRead(IO_PIN_LINETRACK_CENTER);
}
int read_sensor_R()
{
    return !digitalRead(IO_PIN_LINETRACK_RIGHT);
}

//===================================================================
// MODULE   : MOTOR_init()
// FUNCTION : Initialize motor setup
// RETURN   : N/A
//===================================================================
void MOTOR_init()
{
    // Initialize I/O pin
    pinMode(IO_PIN_MOTOR_1, OUTPUT);
    pinMode(IO_PIN_MOTOR_2, OUTPUT);
    pinMode(IO_PIN_MOTOR_3, OUTPUT);
    pinMode(IO_PIN_MOTOR_4, OUTPUT);
    pinMode(IO_PIN_MOTOR_ENA, OUTPUT);
    pinMode(IO_PIN_MOTOR_ENB, OUTPUT);

    // Assign the pins to D/A converter channels
    ledcSetup(DAC_CH_MOTOR_A, 980, 8);
    ledcSetup(DAC_CH_MOTOR_B, 980, 8);
    ledcAttachPin(IO_PIN_MOTOR_ENA, DAC_CH_MOTOR_A);
    ledcAttachPin(IO_PIN_MOTOR_ENB, DAC_CH_MOTOR_B);
}

//===================================================================
// MODULE   : MOTOR_set_speed_left()/MOTOR_set_speed_right()
// FUNCTION : set left/right motor speed
// RETURN   : N/A
//===================================================================
void MOTOR_set_speed_left(int dir, int speed)
{
    ledcWrite(DAC_CH_MOTOR_A, speed);  

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

    if(dir == MOTOR_DIR_FWD) {
        digitalWrite(IO_PIN_MOTOR_3,LOW);
        digitalWrite(IO_PIN_MOTOR_4,HIGH);
    }
    else if(dir == MOTOR_DIR_REV) {
        digitalWrite(IO_PIN_MOTOR_3,HIGH);
        digitalWrite(IO_PIN_MOTOR_4,LOW);
    }
}

//===================================================================
// MODULE   : RoboCar_move_forward()/RoboCar_move_backward()
// FUNCTION : Go forward/backward
// RETURN   : N/A
//===================================================================
void RoboCar_move_forward(int speed)
{
    MOTOR_set_speed_right(MOTOR_DIR_FWD, speed);
    MOTOR_set_speed_left(MOTOR_DIR_FWD, speed);
    
    g_cur_state = STATE_GO_FORWARD;
}

void RoboCar_move_backward(int speed)
{
    MOTOR_set_speed_right(MOTOR_DIR_REV, speed);
    MOTOR_set_speed_left(MOTOR_DIR_REV, speed);
}

//===================================================================
// MODULE   : RoboCar_turn_left()/RoboCar_turn_right()
// FUNCTION : Curve to the left/right
// RETURN   : N/A
//===================================================================
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
}

//===================================================================
// MODULE   : RoboCar_rotate_left()/RoboCar_rotate_right()
// FUNCTION : Rotate to the left/right
// RETURN   : N/A
//===================================================================
void RoboCar_rotate_left(int speed)
{
    MOTOR_set_speed_right(MOTOR_DIR_FWD, speed);
    MOTOR_set_speed_left(MOTOR_DIR_REV, speed);

    g_cur_state = STATE_ROTATO_LEFT;
}

void RoboCar_rotate_right(int speed)
{
    MOTOR_set_speed_right(MOTOR_DIR_REV, speed);
    MOTOR_set_speed_left(MOTOR_DIR_FWD, speed);

    g_cur_state = STATE_ROTATO_RIGHT;
}

//===================================================================
// MODULE   : RoboCar_stop()
// FUNCTION : Stop
// RETURN   : N/A
//===================================================================
void RoboCar_stop()
{
    MOTOR_set_speed_left(0, 0);
    MOTOR_set_speed_right(0, 0);

    g_cur_state = STATE_STOP;
}

//===================================================================
// MODULE   : create_event()
// FUNCTION : Create event ID for the state machine
// RETURN   : Event ID
//===================================================================
int create_event()
{
    static int is_first = 1;
    static int pre_signal_color;

    xSemaphoreTake(g_xMutex_Signal, portMAX_DELAY);
    // ▼▼▼ Start critical section ▼▼▼
    int signal_color = g_trafic_signal_color;
    // ▲▲▲ End critical section ▲▲▲
    xSemaphoreGive(g_xMutex_Signal);
    if(is_first) {
        pre_signal_color = signal_color;
        is_first = 0;
    }

    // Get tracking sensor value
    int sensor_L = read_sensor_L();
    int sensor_C = read_sensor_C();
    int sensor_R = read_sensor_R();
    int sensor = ((sensor_L&0x1)<<2) | ((sensor_C&0x1)<<1) | ((sensor_R&0x1)<<0);
    
    int event = TRACK_EVENT_NONE;
    if((signal_color==SIGNAL_COLOR_BLUE) && (pre_signal_color!=SIGNAL_COLOR_BLUE)) { // The signal turned blue
        event = TRACK_EVENT_TURNED_BLUE;
    }
    else {
        if(sensor == 7) { // "OOO"
            if(signal_color == SIGNAL_COLOR_RED) {
                event = TRACK_EVENT_OOO_RED;
            }
        }
        else { // except for "OOO"
            event = sensor;
        }
    }
	pre_signal_color = signal_color;
    
    return event;
}

//===================================================================
// MODULE   : get_next_state()
// FUNCTION : Get next state of the state machine
// RETURN   : Next State
//===================================================================
int get_next_state(int cur_state, int event) 
{
    if((event < 0) || (TRACK_EVENT_NUM <= event)) { // Invalid event ID
        return g_cur_state;
    }
    
    if((cur_state < 0) || (STATE_NUM <= cur_state)) { // Invalid state ID
        return g_cur_state;
    }
    
    return g_next_state_table[event][cur_state];
}

//===================================================================
// MODULE   : Task_line_trace()
// FUNCTION : [Task] Control RoboCar for line tracking
// RETURN   : N/A
//===================================================================
void Task_line_trace(void* param)
{
	xSemaphoreGive(g_xMutex_Signal);
    for(;;) {
        vTaskDelay(10);

        // Create event
        int event = create_event();
        if(event != TRACK_EVENT_NONE) { // if event occurs
            // Get next state
            int next_state = get_next_state(g_cur_state, event);
            
            if(next_state != g_cur_state) { // Next state is different from current state => State transition occurs
                // Process accoring to state
                if(next_state == STATE_ROTATO_LEFT) {
                    RoboCar_rotate_left(g_motor_speed);
                }
                else if(next_state == STATE_GO_FORWARD) {
                    RoboCar_move_forward(g_motor_speed);
                }
                else if(next_state == STATE_ROTATO_RIGHT) {
                    RoboCar_rotate_right(g_motor_speed);
                }
                else if(next_state == STATE_STOP) {
                    RoboCar_stop();
                }
            }
        }
    }
}

//===================================================================
// MODULE   : Task_serial_cmd()
// FUNCTION : [Task] Parse and execute command from serial port
// RETURN   : N/A
//===================================================================
void Task_serial_cmd(void* param)
{
    char    command_line[256];
    int     cmd_index = 0;

	xSemaphoreGive(g_xMutex_Signal);
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
// FUNCTION : Initialization
// RETURN   : N/A
//===================================================================
void setup()
{
    // Initialize USB serial
    Serial.begin(115200);
    // Initialize Bluetooth serial
    SerialBT.begin("ESP32-12135x");

	Wire.begin(PIN_SDA, PIN_SCL);

	byte rc;
	rc = rpr0521rs.init();

    // Initialize motor
    MOTOR_init();

    // Initialize line tracking sensor pin
    pinMode(IO_PIN_LINETRACK_LEFT, INPUT);
    pinMode(IO_PIN_LINETRACK_CENTER, INPUT);
    pinMode(IO_PIN_LINETRACK_RIGHT, INPUT);

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(Task_line_trace, "Task_line_trace", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(Task_serial_cmd, "Task_serial_cmd", 2048, NULL, 1, NULL, 0);
    
    // Create semaphore
	g_xMutex_Signal = xSemaphoreCreateMutex();

    // Go forward
    RoboCar_stop();
}

//===================================================================
// MODULE   : loop()
// FUNCTION : main loop
// RETURN   : N/A
//===================================================================
void loop() 
{
    // All proccess should be executed in FreeRTOS tasks.
	int error;
	unsigned short ps_val;
	float als_val;

	error = rpr0521rs.get_psalsval(&ps_val, &als_val);
	if(error == 0) {
		// 近接センサー
		Serial.print("Prox. = ");
		Serial.print(ps_val);
		Serial.print("\t||\t");
		// 照度センサー
		Serial.print("Bright. = ");
		Serial.print(als_val);

		Serial.println();
	}
	else {
		Serial.println("[Error] cannot get sensor data.");
	}

	delay(1000);
}

