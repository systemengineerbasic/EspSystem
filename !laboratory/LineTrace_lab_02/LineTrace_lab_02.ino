////////////////////////////////////////////////////////////////////////////////
//
//		LineTrace_lab_02.ino
//			
//			LineTrace制御をTask上で動作させる
//
////////////////////////////////////////////////////////////////////////////////


// Motor Pin
#define	IO_PIN_MOTOR_1			(14)
#define	IO_PIN_MOTOR_2			(12)
#define	IO_PIN_MOTOR_3			(13)
#define	IO_PIN_MOTOR_4			(23)
#define	IO_PIN_MOTOR_ENA		(16)
#define	IO_PIN_MOTOR_ENB		(27)
// Line Tracking Sensor Pin
#define IO_PIN_LINETRACK_LEFT	(26)
#define IO_PIN_LINETRACK_CENTER	(17)
#define IO_PIN_LINETRACK_RIGHT	(39)

// Motor Speed
#define	MOTOR_SPEED				(150)


/////////////////////////////////////////////////////////////////////
// MODULE   : read_sensor_*()
// FUNCTION : センサーを読む(Left/Center/Right)
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
int read_sensor_L()
{
	return	!digitalRead(IO_PIN_LINETRACK_LEFT);
}
int read_sensor_C()
{
	return	!digitalRead(IO_PIN_LINETRACK_CENTER);
}
int read_sensor_R()
{
	return	!digitalRead(IO_PIN_LINETRACK_RIGHT);
}

/////////////////////////////////////////////////////////////////////
// MODULE   : forward()
// FUNCTION : 直進
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
void forward()
{
	ledcWrite(0, MOTOR_SPEED);  
	ledcWrite(1, MOTOR_SPEED);  
	digitalWrite(IO_PIN_MOTOR_1, HIGH);
	digitalWrite(IO_PIN_MOTOR_2, LOW);
	digitalWrite(IO_PIN_MOTOR_3, LOW);
	digitalWrite(IO_PIN_MOTOR_4, HIGH);
	Serial.println("go forward!");
}

/////////////////////////////////////////////////////////////////////
// MODULE   : back()
// FUNCTION : 後退
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
void back()
{
	ledcWrite(0, MOTOR_SPEED);  
	ledcWrite(1, MOTOR_SPEED);  
	digitalWrite(IO_PIN_MOTOR_1, LOW);
	digitalWrite(IO_PIN_MOTOR_2, HIGH);
	digitalWrite(IO_PIN_MOTOR_3, HIGH);
	digitalWrite(IO_PIN_MOTOR_4, LOW);
	Serial.println("go back!");
}

/////////////////////////////////////////////////////////////////////
// MODULE   : left()
// FUNCTION : 左回転
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
void left()
{
	ledcWrite(0, MOTOR_SPEED);  
	ledcWrite(1, MOTOR_SPEED);  
	digitalWrite(IO_PIN_MOTOR_1, LOW);
	digitalWrite(IO_PIN_MOTOR_2, HIGH);
	digitalWrite(IO_PIN_MOTOR_3, LOW);
	digitalWrite(IO_PIN_MOTOR_4, HIGH);
	Serial.println("go left!");
}

/////////////////////////////////////////////////////////////////////
// MODULE   : right()
// FUNCTION : 右回転
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
void right()
{
	ledcWrite(0, MOTOR_SPEED);  
	ledcWrite(1, MOTOR_SPEED);  
	digitalWrite(IO_PIN_MOTOR_1, HIGH);
	digitalWrite(IO_PIN_MOTOR_2, LOW);
	digitalWrite(IO_PIN_MOTOR_3, HIGH);
	digitalWrite(IO_PIN_MOTOR_4, LOW); 
	Serial.println("go right!");
} 

/////////////////////////////////////////////////////////////////////
// MODULE   : stop()
// FUNCTION : 停止
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
void stop()
{
	digitalWrite(IO_PIN_MOTOR_ENA, LOW);
	digitalWrite(IO_PIN_MOTOR_ENB, LOW);
	Serial.println("Stop!");
} 

/////////////////////////////////////////////////////////////////////
// MODULE   : Task_line_trace()
// FUNCTION : [Task] ライントレースカー制御
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
void Task_line_trace(void* param)
{
	for(;;) {
		vTaskDelay(10);

		if(read_sensor_C()){
			forward();
		}
		else if(read_sensor_R()) { 
			right();
			while(read_sensor_R());                             
		}   
		else if(read_sensor_L()) {
			left();
			while(read_sensor_L());  
		}
	}
	
}

/////////////////////////////////////////////////////////////////////
// MODULE   : setup()
// FUNCTION : 初期設定
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
void setup()
{
	// Serial通信の初期化
	Serial.begin(9600);
	
	// I/O Pin属性設定
	pinMode(IO_PIN_LINETRACK_LEFT, INPUT);
	pinMode(IO_PIN_LINETRACK_CENTER, INPUT);
	pinMode(IO_PIN_LINETRACK_RIGHT, INPUT);
	pinMode(IO_PIN_MOTOR_ENA, OUTPUT);
	pinMode(IO_PIN_MOTOR_ENB, OUTPUT);
	pinMode(IO_PIN_MOTOR_1, OUTPUT);
	pinMode(IO_PIN_MOTOR_2, OUTPUT);
	pinMode(IO_PIN_MOTOR_3, OUTPUT);
	pinMode(IO_PIN_MOTOR_4, OUTPUT);

	// DAC設定
	ledcSetup(0, 980, 8);
	ledcSetup(1, 980, 8);
	ledcAttachPin(IO_PIN_MOTOR_ENA, 0);
	ledcAttachPin(IO_PIN_MOTOR_ENB, 1);

	xTaskCreatePinnedToCore(Task_line_trace, "Task_line_trace", 2048, NULL, 1, NULL, 0);
}

/////////////////////////////////////////////////////////////////////
// MODULE   : loop()
// FUNCTION : メインループ
// RETURN   : なし
/////////////////////////////////////////////////////////////////////
void loop() 
{
	// 処理は全てTaskで行うのでメインループ内では何もしない
}

