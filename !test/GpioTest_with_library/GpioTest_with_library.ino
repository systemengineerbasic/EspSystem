/*******************************************************************
    GpioTest.ino
       
    While you keep pushing button, LED is turned ON.

*******************************************************************/

#include <my_led.h>
#include <my_button.h>


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

