// MPU-6050 Accelerometer + Gyro

// I2CにアクセスするためにWireライブラリを使用
#include <Wire.h>

#include <MPU6050.h>


// I2C
#define IO_PIN_SDA 				SDA
#define IO_PIN_SCL 				SCL

// LED
#define	IO_PIN_LED				(18)


// デバイス初期化時に実行される
void setup() 
{
	// LED
	pinMode(IO_PIN_LED, OUTPUT);

	Serial.begin(9600);

	// I2C初期化	
	Wire.begin(IO_PIN_SDA, IO_PIN_SCL);

	MPU6050_init(&Wire);

}

float pre_axl = 0;
void loop() 
{
	int error;
	float	axl_x, axl_y, axl_z;
	float	gyro_x, gyro_y, gyro_z;
	float	temperature;

	// 加速度、角速度、温度を取得
	error = MPU6050_get_all(&axl_x, &axl_y, &axl_z, &gyro_x, &gyro_y, &gyro_z, &temperature);
	if(error != 0) {
		Serial.println("[Error] cannot get sensor data.");
	}
	else {
		Serial.print("Temp. =\t");
		// 温度
		Serial.print(temperature);
		Serial.print("\t||\t");
		// 加速度
		Serial.print("Acc. =\t");
		Serial.print(axl_x);
		Serial.print("\t");
		Serial.print(axl_y);
		Serial.print("\t");
		Serial.print(axl_z);
		Serial.print("\t||\t");
		// Gyro
		Serial.print("Gyro. = \t");
		Serial.print(gyro_x);
		Serial.print("\t");
		Serial.print(gyro_y);
		Serial.print("\t");
		Serial.print(gyro_z);

/*
		float abs_axl2 = (axl_x*axl_x + axl_y*axl_y + axl_z*axl_z);
		float diff_axl = abs_axl2 - pre_axl;
		if(diff_axl < 0) {
			diff_axl = -diff_axl;
		}
		pre_axl = abs_axl2;
		Serial.print(diff_axl, 2);
		Serial.print("\t");
		Serial.println("");

		if(diff_axl < 0.2) {
			digitalWrite(IO_PIN_LED,LOW);
		}
		else {
			digitalWrite(IO_PIN_LED,HIGH);
		}
*/

		Serial.println();
	}

	delay(1000);
//	delay(50);
}

