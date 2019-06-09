#define PIN_LED (2)
// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT 13
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) 
{
    // calculate duty, 8191 from 2 ^ 13 - 1
    uint32_t duty = (8191 / valueMax) * min(value, valueMax);

    // write duty to LEDC
    ledcWrite(channel, duty);
}

void setup() 
{
    // Setup timer and attach timer to a led pin
    Serial.begin(115200);
    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(PIN_LED, LEDC_CHANNEL_0);
}

void loop()
{
    Serial.println("LED On(light)");
    ledcAnalogWrite(LEDC_CHANNEL_0, 255);
    delay(2000);

    Serial.println("LED On(dark)");
    ledcAnalogWrite(LEDC_CHANNEL_0, 31);
    delay(2000);

    Serial.println("LED Off");
    ledcAnalogWrite(LEDC_CHANNEL_0, 0);
    delay(2000);
}


