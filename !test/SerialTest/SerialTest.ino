
void setup() 
{
	Serial.begin(115200);
}

void loop() 
{
	int i;
	int time;
	for(i = 0; i < 5; i ++) {
		// get spendingtime
		time = millis();
		// output to serial-monitor
		Serial.print(time); // time(without line feed)
		Serial.print("[msec], ");
		// wait for 1000msec
		delay(1000);
	}
	Serial.println(); // line feed
	Serial.println("Hello World"); // output string (and line feed)
	Serial.println(); // line feed

	delay(2000);
}
