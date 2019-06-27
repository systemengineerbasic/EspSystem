#include <Arduino.h>
#include "RoboCar.h"

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

//---------------------- RoboCar ----------------------
// Motor derection
#define MOTOR_DIR_STOP          (0)
#define MOTOR_DIR_FWD           (1)
#define MOTOR_DIR_REV           (2)

// RoboCar control status
enum {
    RC_CTRL_MOVE_FORWARD = 0,
    RC_CTRL_TURN_FWD_LEFT,
    RC_CTRL_TURN_FWD_RIGHT,
    RC_CTRL_MOVE_BACKWARD,
    RC_CTRL_TURN_BACK_LEFT,
    RC_CTRL_TURN_BACK_RIGHT,
    RC_CTRL_ROTATE_LEFT,
    RC_CTRL_ROTATE_RIGHT,
    RC_CTRL_STOP
};


int g_control_state;
int g_speed_fwd = 200;
int g_speed_back = 200;
int g_speed_rotate = 200;
int g_speed_turn = 200;

int g_lr_level = 40;


//===================================================================
// MODULE   : min_max_hold()
// FUNCTION : val is clipped by hold_min/hold_max;
// RETURN   : holded value
//===================================================================
int min_max_hold(int val, int hold_min, int hold_max)
{
    int ret = val;
    
    if(val < hold_min ) {
        ret = val;
    }
    else if(val > hold_max) {
        ret = val;
    }
    
    return val;
}   

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
// MODULE   : MOTOR_set_power_left()/MOTOR_set_power_right()
// FUNCTION : set left/right motor power
// RETURN   : N/A
//===================================================================
void MOTOR_set_power_left(int speed) // speed:0-255
{
    ledcWrite(DAC_CH_MOTOR_A, speed);  
}
void MOTOR_set_power_right(int speed) // speed:0-255
{
    ledcWrite(DAC_CH_MOTOR_B, speed);  
}

//===================================================================
// MODULE   : MOTOR_set_dir_left()/MOTOR_set_dir_right()
// FUNCTION : set left/right direction of motor rotation
// RETURN   : N/A
//===================================================================
void MOTOR_set_dir_left(int dir)
{
    if(dir == MOTOR_DIR_FWD) {
        digitalWrite(IO_PIN_MOTOR_1, HIGH);
        digitalWrite(IO_PIN_MOTOR_2, LOW);
    }
    else if(dir == MOTOR_DIR_REV) {
        digitalWrite(IO_PIN_MOTOR_1, LOW);
        digitalWrite(IO_PIN_MOTOR_2, HIGH);
    }
}
void MOTOR_set_dir_right(int dir)
{
    if(dir == MOTOR_DIR_FWD) {
        digitalWrite(IO_PIN_MOTOR_3, LOW);
        digitalWrite(IO_PIN_MOTOR_4, HIGH);
    }
    else if(dir == MOTOR_DIR_REV) {
        digitalWrite(IO_PIN_MOTOR_3, HIGH);
        digitalWrite(IO_PIN_MOTOR_4, LOW);
    }
}

//===================================================================
// MODULE   : RoboCar_control()
// FUNCTION : Control speed and direction of the RoboCar.
// RETURN   : N/A
//===================================================================
void RoboCar_control()
{
    // Speed
    switch(g_control_state) {
        case RC_CTRL_MOVE_FORWARD:
            MOTOR_set_power_left(g_speed_fwd);
            MOTOR_set_power_right(g_speed_fwd);
            break;
        case RC_CTRL_MOVE_BACKWARD:
            MOTOR_set_power_left(g_speed_back);
            MOTOR_set_power_right(g_speed_back);
            break;
        case RC_CTRL_TURN_FWD_LEFT:
        case RC_CTRL_TURN_BACK_LEFT:
            MOTOR_set_power_left(g_speed_turn-g_lr_level);
            MOTOR_set_power_right(g_speed_turn);
            break;
        case RC_CTRL_TURN_FWD_RIGHT:
        case RC_CTRL_TURN_BACK_RIGHT:
            MOTOR_set_power_left(g_speed_turn);
            MOTOR_set_power_right(g_speed_turn-g_lr_level);
            break;
        case RC_CTRL_ROTATE_LEFT:
        case RC_CTRL_ROTATE_RIGHT:
            MOTOR_set_power_left(g_speed_rotate);
            MOTOR_set_power_right(g_speed_rotate);
            break;
        case RC_CTRL_STOP:
            MOTOR_set_power_left(0);
            MOTOR_set_power_right(0);
            break;
    }

    // Direction
    switch(g_control_state) {
        case RC_CTRL_MOVE_FORWARD:
        case RC_CTRL_TURN_FWD_LEFT:
        case RC_CTRL_TURN_FWD_RIGHT:
        case RC_CTRL_STOP:
            MOTOR_set_dir_left(MOTOR_DIR_FWD);
            MOTOR_set_dir_right(MOTOR_DIR_FWD);
            break;
        case RC_CTRL_MOVE_BACKWARD:
        case RC_CTRL_TURN_BACK_LEFT:
        case RC_CTRL_TURN_BACK_RIGHT:
            MOTOR_set_dir_left(MOTOR_DIR_REV);
            MOTOR_set_dir_right(MOTOR_DIR_REV);
            break;
        case RC_CTRL_ROTATE_LEFT:
            MOTOR_set_dir_left(MOTOR_DIR_REV);
            MOTOR_set_dir_right(MOTOR_DIR_FWD);
            break;
        case RC_CTRL_ROTATE_RIGHT:
            MOTOR_set_dir_left(MOTOR_DIR_FWD);
            MOTOR_set_dir_right(MOTOR_DIR_REV);
            break;
    }
}

//===================================================================
// FUNCTION : Initialize the RoboCar
// RETURN   : N/A
//===================================================================
void RoboCar_init()
{
    // Initialize MOTORs
    MOTOR_init();

    // Initialize line tracking sensor pin
    pinMode(IO_PIN_LINETRACK_LEFT, INPUT);
    pinMode(IO_PIN_LINETRACK_CENTER, INPUT);
    pinMode(IO_PIN_LINETRACK_RIGHT, INPUT);

}

//===================================================================
// FUNCTION : Control behavior of the RoboCar
// RETURN   : N/A
//===================================================================
void RoboCar_move_forward()
{
    g_control_state = RC_CTRL_MOVE_FORWARD;
    RoboCar_control();
}
void RoboCar_move_backward()
{
    g_control_state = RC_CTRL_MOVE_BACKWARD;
    RoboCar_control();
}
void RoboCar_turn_fwd_left()
{
    g_control_state = RC_CTRL_TURN_FWD_LEFT;
    RoboCar_control();
}
void RoboCar_turn_back_left()
{
    g_control_state = RC_CTRL_TURN_BACK_LEFT;
    RoboCar_control();
}
void RoboCar_turn_fwd_right()
{
    g_control_state = RC_CTRL_TURN_FWD_RIGHT;
    RoboCar_control();
}
void RoboCar_turn_back_right()
{
    g_control_state = RC_CTRL_TURN_BACK_RIGHT;
    RoboCar_control();
}
void RoboCar_rotate_left()
{
    g_control_state = RC_CTRL_ROTATE_LEFT;
    RoboCar_control();
}
void RoboCar_rotate_right()
{
    g_control_state = RC_CTRL_ROTATE_RIGHT;
    RoboCar_control();
}
void RoboCar_stop()
{
    g_control_state = RC_CTRL_STOP;
    RoboCar_control();
}

//===================================================================
// FUNCTION : Set speed of the RoboCar
// RETURN   : N/A
//===================================================================
void RoboCar_set_speed_fwd(int speed)
{
    g_speed_fwd = min_max_hold(speed, 0, 255);
    RoboCar_control();
}
void RoboCar_set_speed_back(int speed)
{
    g_speed_back = min_max_hold(speed, 0, 255);
    RoboCar_control();
}
void RoboCar_set_speed_turn(int speed)
{
    g_speed_turn = min_max_hold(speed, 0, 255);
    RoboCar_control();
}
void RoboCar_set_speed_rotate(int speed)
{
    g_speed_rotate = min_max_hold(speed, 0, 255);
    RoboCar_control();
}

