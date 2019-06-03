#include "BluetoothSerial.h"

#define	PIN_LED		(2)
#define	PIN_BUTTON	(0)

BluetoothSerial SerialBT;


//=====================================
// Initialize LED
//=====================================
void init_led()
{
    pinMode(PIN_LED, OUTPUT);
    set_led_status(0);
}

//=====================================
// Setup LED status(On or Off)
//=====================================
void set_led_status(int status) // status : 0->Off, not zero->On 
{
    if(status) { 
        digitalWrite(PIN_LED, HIGH);    // turn on LED.
    }
    else {
        digitalWrite(PIN_LED, LOW);     // turn off LED.
    }
}


//=====================================
// Initialization
//=====================================
void setup()
{
	Serial.begin(115200);
	SerialBT.begin("ESP32-XXXXX");
    // Initialize LED
    init_led();
}

//=====================================
// Main loop
//=====================================
void loop()
{
    int data;
    
	if (SerialBT.available()) {
		data = SerialBT.read();
		
		Serial.print(data);
		
		if(data == 'b') {
            set_led_status(1);
        }
        else if(data == 'n') {
            set_led_status(0);
        }
	}

	delay(20);
}