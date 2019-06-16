#define MODE_HELLO_WORLD    (0)
#define MODE_HELLO_KM       (1)


int g_display_mode;
int g_count = 0;

void setup() 
{
	Serial.begin(115200);
	
	g_display_mode = MODE_HELLO_WORLD;
}

void loop() 
{
	if(Serial.available() > 0) {
        // read character from serial port
		int getstr = Serial.read();
		if(getstr == 'w') {
            g_display_mode = MODE_HELLO_WORLD;
        }
		else if(getstr == 'k') {
            g_display_mode = MODE_HELLO_KM;
        }
    }

	if(g_count > 50) {
        g_count = 0;
		int time = millis(); // get time
    	if(g_display_mode == MODE_HELLO_WORLD) {
        	Serial.print("Hello World "); // output string (and line feed)
        }
        else if(g_display_mode == MODE_HELLO_KM) {
        	Serial.print("Hello KM "); // output string (and line feed)
        }
        else {
        	Serial.print("Hello ??? "); // output string (and line feed)
        }
        
        char txt[128];
        sprintf(txt, "%d [msec]\n", time);
        Serial.print(txt);
    }

	g_count ++;
    delay(20);
}
