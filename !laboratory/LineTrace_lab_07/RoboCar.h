
#ifndef _ROBOCAR_H_
#define _ROBOCAR_H_


// Read photo sensor(Left/Center/Right)
int read_sensor_L();
int read_sensor_C();
int read_sensor_R();

// Initialize the RoboCar
void RoboCar_init();

// Control behavior of the RoboCar
void RoboCar_move_forward();
void RoboCar_move_backward();
void RoboCar_turn_fwd_left();
void RoboCar_turn_back_left();
void RoboCar_turn_fwd_right();
void RoboCar_turn_back_right();
void RoboCar_rotate_left();
void RoboCar_rotate_right();
void RoboCar_stop();

// Set speed of the RoboCar
void RoboCar_set_speed_fwd(int speed);
void RoboCar_set_speed_back(int speed);
void RoboCar_set_speed_turn(int speed);
void RoboCar_set_speed_rotate(int speed);

#endif
