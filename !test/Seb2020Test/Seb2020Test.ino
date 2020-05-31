/*****************************************************************************
  RPR-0521RS.ino
 Copyright (c) 2016 ROHM Co.,Ltd.
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/
#include <Wire.h>
#include <my_led_button.h>
#include <RPR-0521RS.h>

// BMX055　加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)


//---------------------- PIN ----------------------
#define PIN_BUTTON_01       ( 0) // 
#define PIN_BUTTON_RED      ( 4) // 
#define PIN_BUTTON_YELLOW   (19) // 
#define PIN_BUTTON_GREEN    (17) // 
#define PIN_LED_01          ( 2)
#define PIN_LED_RED         (13)
#define PIN_LED_YELLOW      (12)
#define PIN_LED_GREEN       (14)
#define PIN_LED_WHITE_01    (23)
#define PIN_LED_WHITE_02    (16)
#define PIN_LED_BLUE        ( 5)

#define PIN_SDA SDA
#define PIN_SCL SCL

//---------------------- D/A converter channel ----------------------
#define DAC_CH_LED_01           (3)
#define DAC_CH_LED_RED          (4)
#define DAC_CH_LED_YELLOW       (5)
#define DAC_CH_LED_GREEN        (6)
#define DAC_CH_LED_WHITE_01     (7)
#define DAC_CH_LED_WHITE_02     (8)
#define DAC_CH_LED_BLUE         (9)

// センサーの値を保存するグローバル関数
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;


my_led g_led_01(PIN_LED_01, DAC_CH_LED_01);
my_led g_led_red(PIN_LED_RED, DAC_CH_LED_RED);
my_led g_led_yellow(PIN_LED_YELLOW, DAC_CH_LED_YELLOW);
my_led g_led_green(PIN_LED_GREEN, DAC_CH_LED_GREEN);
my_led g_led_white_01(PIN_LED_WHITE_01, DAC_CH_LED_WHITE_01);
my_led g_led_white_02(PIN_LED_WHITE_02, DAC_CH_LED_WHITE_02);
my_led g_led_blue(PIN_LED_BLUE, DAC_CH_LED_BLUE);

my_button   g_button_01(PIN_BUTTON_01);
my_button   g_button_red(PIN_BUTTON_RED);
my_button   g_button_yellow(PIN_BUTTON_YELLOW);
my_button   g_button_green(PIN_BUTTON_GREEN);

RPR0521RS rpr0521rs;

QueueHandle_t g_xQueue_Serial;
SemaphoreHandle_t g_xMutex = NULL;


void osTask_sensor(void* param)
{
	int error;
	float	acc_x, acc_y, acc_z;
	float	gyro_x, gyro_y, gyro_z;
	BaseType_t xStatus;
	byte rc;
	float pre_abs_axl = 0;

	xSemaphoreGive(g_xMutex);
	for(;;) {
		vTaskDelay(50);

		// 照度・近接センサの値を取得
		unsigned short ps_val;
		float als_val;
		rc = rpr0521rs.get_psalsval(&ps_val, &als_val);
		uint32_t prox = (int)(ps_val/10); // Value is changed according to proximity.
		if(prox == 0) {
            g_led_white_02.turn(0);
        }
        else {
            g_led_white_02.turn(1);
            g_led_white_02.set_brightness(prox);
        }
        
        float temp = 100.0 - als_val;
        temp = (temp >= 0.0) ? temp : 0;
		uint32_t br = (int)(temp*2.55); // Value is changed according to brightness.
		if(br == 0) {
            g_led_white_01.turn(0);
        }
        else {
            g_led_white_01.turn(1);
            g_led_white_01.set_brightness(br);
        }

		// 加速度を取得
        BMX055_Accl();
		// 衝撃検出
		float abs_axl = (xAccl*xAccl + yAccl*yAccl + zAccl*zAccl);
		float diff_axl = abs_axl - pre_abs_axl;
		pre_abs_axl = abs_axl;
		float diff = abs(diff_axl);
		if(diff_axl < 10) {
            g_led_blue.turn(0);
		}
		else {
            g_led_blue.turn(1);
		}

        
		// ▼▼▼ [排他制御区間]開始 ▼▼▼
		xStatus = xSemaphoreTake(g_xMutex, 0);
		xSemaphoreGive(g_xMutex);
		// ▲▲▲ [排他制御区間]開始 ▲▲▲
	}

}

void isr_button_01()
{
    g_led_01.toggle();
}
void isr_button_red()
{
    g_led_red.turn(g_button_red.get_status());
}
void isr_button_yellow()
{
    g_led_yellow.turn(g_button_yellow.get_status());
}
void isr_button_green()
{
    g_led_green.turn(g_button_green.get_status());
}

void setup() 
{
	byte rc;

	Serial.begin(115200);
	while (!Serial);

	Wire.begin(PIN_SDA, PIN_SCL);
	
	g_button_01.attach_isr(isr_button_01, FALLING);
	g_button_red.attach_isr(isr_button_red, CHANGE);
	g_button_yellow.attach_isr(isr_button_yellow, CHANGE);
	g_button_green.attach_isr(isr_button_green, CHANGE);
	
	rc = rpr0521rs.init();

    //BMX055 初期化
    BMX055_Init();

	// Task生成(優先度は数が大きいほど優先度高)
	g_xMutex = xSemaphoreCreateMutex();
	xTaskCreatePinnedToCore(osTask_sensor, "osTask_sensor", 2048, NULL, 5, NULL, 0);
}

void loop() 
{
	delay(1000);
}

//=====================================================================================//
void BMX055_Init()
{
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03);   // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Accl()
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)  xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)  yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)  zAccl -= 4096;
  xAccl = xAccl * 0.0098; // renge +-2g
  yAccl = yAccl * 0.0098; // renge +-2g
  zAccl = zAccl * 0.0098; // renge +-2g
}
//=====================================================================================//
void BMX055_Gyro()
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)  xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)  yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag()
{
  int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] <<8) | (data[0]>>3));
  if (xMag > 4095)  xMag -= 8192;
  yMag = ((data[3] <<8) | (data[2]>>3));
  if (yMag > 4095)  yMag -= 8192;
  zMag = ((data[5] <<8) | (data[4]>>3));
  if (zMag > 16383)  zMag -= 32768;
}
