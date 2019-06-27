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
#include <MPU6050.h>
#include <my_led_button.h>
#include "RoboCar.h"


//---------------------- PIN ----------------------
// LED
#define IO_PIN_LED_1            ( 2)
#define IO_PIN_LED_2            (18)
// Button
#define IO_PIN_BUTTON_1         ( 0)

//---------------------- D/A converter channel ----------------------
#define DAC_CH_SERVO            (2)
#define DAC_CH_LED_1            (3)
#define DAC_CH_LED_2            (4)

//---------------------- Trafic Signal ----------------------
#define SIGNAL_COLOR_BLACK      (0)
#define SIGNAL_COLOR_BLUE       (1)
#define SIGNAL_COLOR_YELLOW     (2)
#define SIGNAL_COLOR_RED        (3)

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

// Command Mode
enum {
    CMD_MODE_KEY,
    CMD_MODE_LINE
};

BluetoothSerial SerialBT;

int g_cur_state;    // Current state
int g_trafic_signal_color = SIGNAL_COLOR_RED;

int g_robocar_setting_speed_fwd = 200;
int g_robocar_setting_speed_back = 200;
int g_robocar_setting_speed_turn = 200;
int g_robocar_setting_speed_rotate = 200;
int g_robocar_setting_lr_level = 40;
int g_robocar_slow_down_delta = 50;

Stream* g_pSerial=&Serial; // Selected serial port (USB is default)

SemaphoreHandle_t g_xMutex_Signal = NULL;   // for critical section for signal color
SemaphoreHandle_t g_xMutex_Sensor = NULL;   // for critical section for reading sensor 

QueueHandle_t g_xQueue_Sensor = NULL;
QueueHandle_t g_xQueue_Command = NULL;

RPR0521RS rpr0521rs;
int g_mpu6050_enable  = 0; // if MPU-6050 is enabled
int g_rpr0521rs_enable  = 0; // if RPR-0521RS is enabled
float   g_brightness_thresh = 100.0; // Bright and dark threshold 
float   g_impact_thresh = 0.5; // Threshold for impact detection
my_led      g_led1(IO_PIN_LED_1, DAC_CH_LED_1);
my_led      g_led2(IO_PIN_LED_2, DAC_CH_LED_2);

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
        g_robocar_setting_speed_fwd = speed;
        g_robocar_setting_speed_back = speed;
        g_robocar_setting_speed_turn = speed;
        g_robocar_setting_speed_rotate = speed;
        RoboCar_set_speed_fwd(g_robocar_setting_speed_fwd);
        RoboCar_set_speed_back(g_robocar_setting_speed_back);
        RoboCar_set_speed_turn(g_robocar_setting_speed_turn);
        RoboCar_set_speed_rotate(g_robocar_setting_speed_rotate);
    }
}

void _cmd__signal(int argc, char* argv[])
{
    if(argc > 1) {
        int color = SIGNAL_COLOR_BLACK;
        if(strcmp(argv[1], "r")==0) {
            color = SIGNAL_COLOR_RED;
            g_pSerial->println("Red");
        }
        else if(strcmp(argv[1], "y")==0) {
            color = SIGNAL_COLOR_YELLOW;
            g_pSerial->println("Yellow");
        }
        else if(strcmp(argv[1], "b")==0) {
            color = SIGNAL_COLOR_BLUE;
            g_pSerial->println("Blue");
        }
        
        if(color != SIGNAL_COLOR_BLACK) {
            xSemaphoreTake(g_xMutex_Signal, portMAX_DELAY);
            // ������ Start critical section ������
            g_trafic_signal_color = color;
            // ������ End critical section ������
            xSemaphoreGive(g_xMutex_Signal);
        }
    }
}

void _cmd__brightth(int argc, char* argv[])
{
    int error = 1;
    if(argc > 1) {
        float thresh = atof(argv[1]);
        if(thresh > 0.0) {
            g_brightness_thresh = thresh;
        }
        error = 0;
    }
    g_pSerial->print("current brightness thresh = ");
    g_pSerial->println(g_brightness_thresh);
    
    if(error) {
        char txt[128];
        sprintf(txt, "[Usage] %s thresh(>0)", argv[0]);
        g_pSerial->println(txt);
    }
}

void _cmd__impactth(int argc, char* argv[])
{
    int error = 1;
    if(argc > 1) {
        float thresh = atof(argv[1]);
        if(thresh > 0.0) {
            g_impact_thresh = thresh;
        }
        error = 0;
    }
    g_pSerial->print("current impact thresh = ");
    g_pSerial->println(g_impact_thresh);
    
    if(error) {
        char txt[128];
        sprintf(txt, "[Usage] %s thresh(>0)", argv[0]);
        g_pSerial->println(txt);
    }
}

void _cmd__sensor(int argc, char* argv[])
{
    int error;

    if(g_mpu6050_enable) { // MPU-6050 was initialized
    	float	axl_x, axl_y, axl_z;
    	float	gyro_x, gyro_y, gyro_z;
    	float	temperature;
        xSemaphoreTake(g_xMutex_Sensor, portMAX_DELAY); // ������ Start critical section ������
		error = MPU6050_get_all(&axl_x, &axl_y, &axl_z, &gyro_x, &gyro_y, &gyro_z, &temperature);
        xSemaphoreGive(g_xMutex_Sensor); // ������ End critical section ������
    	if(error == 0) {
            char txt[128];
            sprintf(txt, "  Acceleration = (%f, %f, %f)", axl_x, axl_y, axl_z);
    		Serial.println(txt);
            sprintf(txt, "  Gyro = (%f, %f, %f)", gyro_x, gyro_y, gyro_z);
    		Serial.println(txt);
            sprintf(txt, "  Temperature = %f", temperature);
    		Serial.println(txt);
        }
    	else {
    		Serial.println("[Error] cannot read MPU-6050.");
    	}
    }
    
    if(g_rpr0521rs_enable) { // RPR-0521RS was initialized
    	unsigned short ps_val;
    	float als_val;
        xSemaphoreTake(g_xMutex_Sensor, portMAX_DELAY); // ������ Start critical section ������
    	error = rpr0521rs.get_psalsval(&ps_val, &als_val);
        xSemaphoreGive(g_xMutex_Sensor); // ������ End critical section ������
    	if(error == 0) {
    		// Proximity
    		Serial.print("  Proximity = ");
    		Serial.println(ps_val);
    		// Brightness
    		Serial.print("  Brightness = ");
    		Serial.println(als_val);

    		Serial.println();
    	}
    	else {
    		Serial.println("[Error] cannot read RPR-0521RS.");
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
    {"brightth",    _cmd__brightth},
    {"impactth",    _cmd__impactth},
    {"sensor",      _cmd__sensor},
    // The last line must be NULL
    {NULL,          NULL},
};


//===================================================================
// MODULE   : create_event()
// FUNCTION : Create event ID for the state machine
// RETURN   : Event ID
//===================================================================
int create_event()
{
    static int is_first = 1;
    static int pre_signal_color;

    xSemaphoreTake(g_xMutex_Signal, portMAX_DELAY); // ������ Start critical section ������
    int signal_color = g_trafic_signal_color;
    xSemaphoreGive(g_xMutex_Signal); // ������ End critical section ������
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
// MODULE   : Task_RoboCar()
// FUNCTION : [Task] Control RoboCar for line tracking
// RETURN   : N/A
//===================================================================
void Task_RoboCar(void* param)
{
    portTickType 	wait_tick = 10/portTICK_RATE_MS; // 10[ms];
    
    xSemaphoreGive(g_xMutex_Signal);
    xSemaphoreGive(g_xMutex_Sensor);
    for(;;) {
		int q_data = 0;
		BaseType_t	xStatus = xQueueReceive(g_xQueue_Sensor, &q_data, wait_tick);
		if(xStatus) { // if you receive some data from queue.
    		if(q_data == 1) {
                RoboCar_set_speed_fwd(g_robocar_setting_speed_fwd);
                RoboCar_set_speed_back(g_robocar_setting_speed_back);
                RoboCar_set_speed_turn(g_robocar_setting_speed_turn);
                RoboCar_set_speed_rotate(g_robocar_setting_speed_rotate);
            }
            else {
                RoboCar_set_speed_fwd(g_robocar_setting_speed_fwd-g_robocar_slow_down_delta);
                RoboCar_set_speed_back(g_robocar_setting_speed_back-g_robocar_slow_down_delta);
                RoboCar_set_speed_turn(g_robocar_setting_speed_turn-g_robocar_slow_down_delta);
                RoboCar_set_speed_rotate(g_robocar_setting_speed_rotate-g_robocar_slow_down_delta);
            }
        }

        // Create event
        int event = create_event();
        if(event != TRACK_EVENT_NONE) { // if event occurs
            // Get next state
            int next_state = get_next_state(g_cur_state, event);
            
            if(next_state != g_cur_state) { // Next state is different from current state => State transition occurs
                // Process accoring to state
                if(next_state == STATE_ROTATO_LEFT) {
                    RoboCar_rotate_left();
                }
                else if(next_state == STATE_GO_FORWARD) {
                    RoboCar_move_forward();
                }
                else if(next_state == STATE_ROTATO_RIGHT) {
                    RoboCar_rotate_right();
                }
                else if(next_state == STATE_STOP) {
                    RoboCar_stop();
                }
                g_cur_state = next_state;
            }
        }
    }}

//===================================================================
// MODULE   : Task_serial_cmd()
// FUNCTION : [Task] Parse and execute command from serial port
// RETURN   : N/A
//===================================================================
void Task_serial_cmd(void* param)
{
    int     cmd_mode = CMD_MODE_LINE;
    char    command_line[128];
    int     cmd_index = 0;

    xSemaphoreGive(g_xMutex_Signal);
    xSemaphoreGive(g_xMutex_Sensor);
    for(;;) {
        vTaskDelay(10);
        
        if(g_pSerial->available() > 0) { // received data
            char getstr = g_pSerial->read(); // Read data from serial-port
            if(cmd_mode == CMD_MODE_LINE) {
                command_line[cmd_index] = getstr;
                cmd_index ++;
                if(getstr == '\n') { // Detect "LF"(enter key)
                    command_line[cmd_index-1] = '\0';
                    cmd_index = 0;

                    // "key" command
                    char    temp[8];
                    strncpy(temp, command_line, 7);
                    temp[7] = '\0';
                    char* p = strtok(temp, " ");
                    if(p) {
                        if(strcmp(p, "key") == 0) {
                            g_pSerial->println("Changed to Key command mode.");
                            cmd_mode = CMD_MODE_KEY;
                        }
                    }

                    // Parse and execute command
                    parse_and_exec_cmd(command_line, g_command_table);
                }
            }
            else { // CMD_MODE_KEY
                if(getstr == 'z') { // change to Line mode.
                    g_pSerial->println("Changed to Line command mode.");
                    cmd_mode = CMD_MODE_LINE;
                    cmd_index = 0;
                }
                else if(getstr == 't') {
                    parse_and_exec_cmd("test", g_command_table);
                }
            }
        }
    }
}

//===================================================================
// MODULE   : Task_sensor()
// FUNCTION : [Task] get sensor data
// RETURN   : N/A
//===================================================================
void Task_sensor(void* param)
{
    xSemaphoreGive(g_xMutex_Signal);
    xSemaphoreGive(g_xMutex_Sensor);
	byte rc;
	float pre_abs_axl = 0;
    int   cur_brigth_status=-1;
	portTickType wakeupTime = xTaskGetTickCount();
    for(;;) {
        vTaskDelayUntil(&wakeupTime, 50);

        if(g_mpu6050_enable) { // RPR-0521RS was initialized
    		// Get acceleration and temperature
        	float	axl_x, axl_y, axl_z;
        	float	gyro_x, gyro_y, gyro_z;
        	float	temperature;
            xSemaphoreTake(g_xMutex_Sensor, portMAX_DELAY); // ������ Start critical section ������
    		int error = MPU6050_get_all(&axl_x, &axl_y, &axl_z, &gyro_x, &gyro_y, &gyro_z, &temperature);
            xSemaphoreGive(g_xMutex_Sensor); // ������ End critical section ������
    		// Detect impact
    		float abs_axl = (axl_x*axl_x + axl_y*axl_y + axl_z*axl_z);
    		float diff_axl = abs_axl - pre_abs_axl;
    		if(diff_axl < 0) { // abs
    			diff_axl = -diff_axl;
    		}
    		pre_abs_axl = abs_axl;
            if(diff_axl < g_impact_thresh) {
                g_led1.turn(0);
            }
            else {
                g_led1.turn(1);
            }
                
        }

        if(g_rpr0521rs_enable) { // RPR-0521RS was initialized
            unsigned short ps_val;
            float als_val;
            xSemaphoreTake(g_xMutex_Sensor, portMAX_DELAY); // ������ Start critical section ������
            rpr0521rs.get_psalsval(&ps_val, &als_val);
            xSemaphoreGive(g_xMutex_Sensor); // ������ End critical section ������
            int   brigth_status;
            if(als_val > g_brightness_thresh) { // Bright 
                brigth_status = 1;
            }
            else { // Dark
                brigth_status = 0;
            }
            if(cur_brigth_status != brigth_status) {
                xQueueSend(g_xQueue_Sensor, &brigth_status, 0); // Send Q-message to RoboCar-Task
                if(brigth_status == 1) {
                    g_led2.turn(1);
                }
                else {
                    g_led2.turn(0);
                }
                cur_brigth_status = brigth_status;
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

	Wire.begin(SDA, SCL);

    // Initialize RPR-0521RS
    byte rc = rpr0521rs.init();
    if(rc == 0) {
        g_rpr0521rs_enable = 1;
    }

    // Initialize MPU-6050
	int err = MPU6050_init(&Wire);
    if(err == 0) {
        g_mpu6050_enable = 1;
    }

    // Initialize RoboCar
    RoboCar_init();

    // Create semaphore
    g_xMutex_Signal = xSemaphoreCreateMutex();
    g_xMutex_Sensor = xSemaphoreCreateMutex();

    // Create queue 
    g_xQueue_Sensor = xQueueCreate(8, sizeof(int));
    g_xQueue_Command = xQueueCreate(8, sizeof(int));

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(Task_RoboCar, "Task_RoboCar", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(Task_serial_cmd, "Task_serial_cmd", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(Task_sensor, "Task_sensor", 2048, NULL, 3, NULL, 0);
    
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
}

