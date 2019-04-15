// US Sendor
#define	IO_PIN_US_ECHO			(36)
#define	IO_PIN_US_TRIG			( 5)


float SR04_get_distance(unsigned long max_dist=100)   // return:[cm](timeoutî≠ê∂éûÇÕ10000.0)  max_dist:[cm]
{
	// Pulseî≠ê∂
	digitalWrite(IO_PIN_US_TRIG, LOW);   
	delayMicroseconds(2);
	digitalWrite(IO_PIN_US_TRIG, HIGH);  
	delayMicroseconds(20);
	digitalWrite(IO_PIN_US_TRIG, LOW);   
	
	// îΩéÀîgìûíBÇ‹Ç≈ÇÃéûä‘åvë™(timeoutî≠ê∂éûÇÕ0)
	unsigned long duration = pulseIn(IO_PIN_US_ECHO, HIGH, max_dist*58);  
	float dist;
	if(duration == 0) {
		dist = 10000.0;
	}
	else {
		dist = duration * 0.017; // [cm]
	}

	return dist;
}  



void setup() 
{

	pinMode(IO_PIN_US_ECHO, INPUT);    
	pinMode(IO_PIN_US_TRIG, OUTPUT);  

	Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() 
{
	float dist = SR04_get_distance();

	Serial.print("Distance = ");
	Serial.println(dist);

	delay(1000);                      // Wait for a second
}

