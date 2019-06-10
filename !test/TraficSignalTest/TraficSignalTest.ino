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

// Signal Mode
#define	SM_AUTO			(0)
#define	SM_BLUE			(1)
#define	SM_YELLOW		(2)
#define	SM_RED			(3)

// Signal COLOR
#define	SCOLOR_BLUE		(0)
#define	SCOLOR_YELLOW	(1)
#define	SCOLOR_RED		(2)

const int RED_PIN = 13;
const int YELLOW_PIN = 12;
const int BLUE_PIN = 14;

QueueHandle_t g_xQueue_Command;


int g_signal_mode = SM_AUTO;
int g_signal_color = SCOLOR_RED;
int g_signal_timing_count = 0;


void osTask_Receive(void* param)
{
	for(;;) {
		vTaskDelay(200);

        if(Serial.available() > 0) { // received data
            int getstr = Serial.read(); // Read data from serial-port

			int signal_mode = -1;
            if(getstr == 'b') {
                signal_mode = SM_BLUE;
            }
            else if(getstr == 'y') {
                signal_mode = SM_YELLOW;
            }
            else if(getstr == 'r') {
                signal_mode = SM_RED;
            }
            else if(getstr == 'a') {
                signal_mode = SM_AUTO;
            }
            
            if(signal_mode != -1) {
                // Send Q-message to command queue
                xQueueSend(g_xQueue_Command, &signal_mode, 0);
    		}
    	}
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

    	Serial.println("Signal color : BLUE");
	}
	else if(g_signal_color == SCOLOR_YELLOW) {
		str_color = "YELLOW";
		digitalWrite(BLUE_PIN, LOW);
		digitalWrite(YELLOW_PIN, HIGH);
		digitalWrite(RED_PIN, LOW);

    	Serial.println("Signal color : YELLOW");
	}
	else {
		str_color = "RED";
		digitalWrite(BLUE_PIN, LOW);
		digitalWrite(YELLOW_PIN, LOW);
		digitalWrite(RED_PIN, HIGH);

    	Serial.println("Signal color : RED");
	}
}

void setup()
{
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  Serial.begin(115200);

  // Create queue for receive command from web.
  g_xQueue_Command = xQueueCreate(8, sizeof(int32_t));
  // Create tasks 
  xTaskCreatePinnedToCore(osTask_Receive, "osTask_Receive", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(osTask_Signal, "osTask_Signal", 4096, NULL, 3, NULL, 0);

  set_signal_color(SCOLOR_RED);

}

void loop() {
}
