/*******************************************************************
    GpioTest.ino
       
    While you keep pushing button, LED is turned ON.

*******************************************************************/


#define	PIN_LED		(2)
#define	PIN_BUTTON	(0)


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
// Initialize button
//=====================================
void init_button()
{
    pinMode(PIN_BUTTON, INPUT);
}

//=====================================
// Get button status(On or Off)
//=====================================
int get_button_status()
{
	int button_status = digitalRead(PIN_BUTTON);
    if(button_status == 0) { // If button is pushed
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
    // Initialize button
    init_button();
}

//=====================================
// Main loop
//=====================================
void loop() 
{
    int button_status;
	button_status = get_button_status();
	
    if(button_status) { // If button is pushed, turn on LED.
        set_led_status(1);
    }
    else {
        set_led_status(0);
    }
        
}

