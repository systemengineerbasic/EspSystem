#define PIN_BUTTON  (0) // 

void setup() 
{
    Serial.begin(115200);

    pinMode(PIN_BUTTON, INPUT);
}

void loop() 
{
	int button_data = digitalRead(PIN_BUTTON); 
	if(button_data == LOW) {
        Serial.println("Button is pushed.");
    }
    else {
        Serial.println("Button is not pushed.");
    }
    
    delay(1000);
}

