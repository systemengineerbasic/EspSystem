/*******************************************************************
    my_led_button.cpp
       
    LED/Button control library

*******************************************************************/
#include <Arduino.h>
#include "my_led_button.h"



//=====================================
// Constructor
//=====================================
my_led::my_led(int pin)
{
    io_pin = pin;
    pinMode(io_pin, OUTPUT);
    set_status(0);
}

//=====================================
// Setup LED status(On or Off)
//=====================================
void my_led::set_status(int status) // status : 0->Off, not zero->On 
{
    if(status) { 
        digitalWrite(io_pin, HIGH);    // turn on LED.
    }
    else {
        digitalWrite(io_pin, LOW);     // turn off LED.
    }
}

//=====================================
// Get LED status(On or Off)
//=====================================
int my_led::get_status() // return : 0->Off, not zero->On 
{
    return  digitalRead(io_pin);   
}



//=====================================
// Constructor
//=====================================
my_button::my_button(int pin)
{
    io_pin = pin;
    pinMode(io_pin, INPUT);
}

//=====================================
// Get button status(On or Off)
//=====================================
int my_button::get_status()
{
	int button_status = digitalRead(io_pin);
    if(button_status == 0) { // If button is pushed
        return  1;
    }
    else {
        return  0;
    }
}

