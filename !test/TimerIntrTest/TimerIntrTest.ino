#define	PIN_LED		(2)

// Timer Interrupt setting
hw_timer_t * timer = NULL;
// Mutex for critical section
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int g_led_status = 0;

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
// Timer interrupt service routine(ISR)
//=====================================
void IRAM_ATTR ISR_timer0()
{
    // Increment the counter and set the time of ISR
    portENTER_CRITICAL_ISR(&timerMux);

    set_led_status(g_led_status);
    if(g_led_status) {
        g_led_status = 0;
    }
    else {
        g_led_status = 1;
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() 
{
    // Initialze serial port
    Serial.begin(115200);
    
    // Initialze serial LED
    init_led();
    set_led_status(g_led_status);

    // Use 1st timer of 4 (counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
    timer = timerBegin(0, 80, true);

    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &ISR_timer0, true);

    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, 1000*1000, true);

    // Start an alarm
    timerAlarmEnable(timer);
}

void loop() 
{
    Serial.println("Hello World");
    
    delay(1500); // wait 1500[msec]
}

