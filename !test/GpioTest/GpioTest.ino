#define	PIN_LED		(2)
#define	PIN_SW		(0)


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
// Initialize push switch
//=====================================
void init_sw()
{
    pinMode(PIN_SW, INPUT);
}

//=====================================
// Get push switch status(On or Off)
//=====================================
int get_sw_status()
{
	int sw_status = digitalRead(PIN_SW);
    if(sw_status == 0) { // If sw is pushed
        return  1;
    }
    else {
        return  0;
    }
}

//=====================================
// Initialization
//=====================================
void setup()
{
    // Initialize LED
    init_led();
    // Initialize push switch
    init_sw();
}

//=====================================
// Main loop
//=====================================
void loop() 
{
    int sw_status;
	sw_status = get_sw_status();
	
    if(sw_status) { // If sw is pushed, turn on LED.
        set_led_status(1);
    }
    else {
        set_led_status(0);
    }
        
}

