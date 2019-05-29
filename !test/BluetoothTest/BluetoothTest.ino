#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup()
{
	Serial.begin(9600);
	SerialBT.begin("ESP32");
}

int count = 0;
void loop()
{
	if (Serial.available()) {
		SerialBT.write(Serial.read());
	}
	if (SerialBT.available()) {
		Serial.write(SerialBT.read());
	}

	if(count > 50) {
		count = 0;
		SerialBT.println("[Bluetooth] Hello World");
	}
	count ++;

	delay(20);
}