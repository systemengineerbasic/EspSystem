
void setup() 
{
	// 転送レートを9600bpsに設定
	Serial.begin(9600);
}

void loop() 
{
	int i;
	int time;
	for(i = 0; i < 5; i ++) {
		// 経過時間取得
		time = millis();
		// シリアルモニタへ出力
		Serial.print(time); // シリアルへ時間出力（改行なし）
		Serial.print("[msec], ");
		// 1000msec待つ
		delay(1000);
	}
	Serial.println(); // 改行
	Serial.println("Hello World"); // 文字列出力（改行あり）
	Serial.println(); // 改行

	delay(2000);
}
