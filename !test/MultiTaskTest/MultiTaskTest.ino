#define	PIN_LED		(2)
#define	PIN_BUTTON	(0)


QueueHandle_t g_xQueue_Command;
int g_cur_button_status;


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

void osTask_Button(void* param)
{
	for(;;) {
        // Wait 200[msec]. While this task is waiting, other task can execute. 
		vTaskDelay(200);

        int button_status;
    	button_status = get_button_status();

        if(button_status != g_cur_button_status) { // button status was changed
            // Send Q-message to command queue
            xQueueSend(g_xQueue_Command, &button_status, 0);
        }
        
        // Update global status 
        g_cur_button_status = button_status;
	}
}

void osTask_Led(void* param)
{
	for(;;) {
		int q_data = 0;
		BaseType_t	xStatus = xQueueReceive(g_xQueue_Command, &q_data, portMAX_DELAY);
		
		if(xStatus) { // if you receive some data from queue.
            int button_status = q_data;
            if(button_status) { // If button is pushed, turn on LED.
                set_led_status(1);
            }
            else {
                set_led_status(0);
            }
        }
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

    // Create queue for receive command from web.
    g_xQueue_Command = xQueueCreate(8, sizeof(int32_t));
    // Create tasks 
    xTaskCreatePinnedToCore(osTask_Button, "osTask_Button", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(osTask_Led, "osTask_Led", 4096, NULL, 3, NULL, 0);
}

void loop() {
}
