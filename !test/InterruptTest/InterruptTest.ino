#define LEDpin1 2 // LED‚ğÚ‘±‚µ‚½GPIO”Ô†
#define LEDpin2 13 // LED‚ğÚ‘±‚µ‚½GPIO”Ô†
#define SWpin1 0 // SW‚ğÚ‘±‚µ‚½GPIO”Ô†
#define SWpin2 16 // SW‚ğÚ‘±‚µ‚½GPIO”Ô†

int SWdata;
int LedStatus = LOW;

void setup() 
{
	Serial.begin(9600);

	pinMode(LEDpin1, OUTPUT);
	pinMode(LEDpin2, OUTPUT);
	pinMode(SWpin1, INPUT);
	pinMode(SWpin2, INPUT);
	attachInterrupt( SWpin1, intFunc1, CHANGE);
	attachInterrupt( SWpin2, intFunc2, FALLING);
	
	digitalWrite(LEDpin1, LedStatus);
}

void loop() 
{
}

void intFunc1()
{
	Serial.println("Intr SW1");

	SWdata = digitalRead(SWpin1); 
	if(SWdata == 0){
		digitalWrite(LEDpin1, HIGH);
	} 
	else{
		digitalWrite(LEDpin1, LOW);
	}
}

void intFunc2()
{
	Serial.println("Intr SW2");

	if(LedStatus == LOW) {
		LedStatus = HIGH;
	}
	else {
		LedStatus = LOW;
	}

	digitalWrite(LEDpin2, LedStatus);
}

