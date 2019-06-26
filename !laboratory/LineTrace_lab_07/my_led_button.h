/*******************************************************************
    my_led_button.h
       
    LED/Button control library

*******************************************************************/

#ifndef _MY_LED_BUTTONH_
#define _MY_LED_BUTTONH_


//=====================================
// LED class library
//=====================================
class my_led
{
private:
    int io_pin;
    
public:
    my_led(int pin);

    // Setup LED status(On or Off)
    void set_status(int status); // status : 0->Off, not zero->On 
    // Get LED status(On or Off)
    int get_status(); // return : 0->Off, not zero->On 
    
};


//=====================================
// Button class library
//=====================================
class my_button
{
private:
    int io_pin;
    
public:
    my_button(int pin);

    // Get button status(On or Off)
    int get_status(); // return: 1->Button Pushed  0->Button Released
    
};


#endif
