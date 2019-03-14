//www.elegoo.com

/*
//Line Tracking IO define
#define LT_R !digitalRead(10)
#define LT_M !digitalRead(4)
#define LT_L !digitalRead(2)

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
*/


#define LT_R (!digitalRead(39))
#define LT_M (!digitalRead(17))
#define LT_L (!digitalRead(26))

#define ENA 16
#define ENB 27
#define IN1 14
#define IN2 12
#define IN3 13
#define IN4 23

#define carSpeed 150

void forward(){
	ledcWrite(0, carSpeed);  
	ledcWrite(1, carSpeed);  
//  analogWrite(ENA, carSpeed);
//  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go forward!");
}

void back(){
	ledcWrite(0, carSpeed);  
	ledcWrite(1, carSpeed);  
//  analogWrite(ENA, carSpeed);
//  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go back!");
}

void left(){
	ledcWrite(0, carSpeed);  
	ledcWrite(1, carSpeed);  
//  analogWrite(ENA, carSpeed);
//  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go left!");
}

void right(){
	ledcWrite(0, carSpeed);  
	ledcWrite(1, carSpeed);  
//  analogWrite(ENA, carSpeed);
//  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  Serial.println("go right!");
} 

void stop(){
   digitalWrite(ENA, LOW);
   digitalWrite(ENB, LOW);
   Serial.println("Stop!");
} 

void setup(){
  Serial.begin(9600);
  pinMode(39,INPUT);
  pinMode(17,INPUT);
  pinMode(26,INPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

	ledcSetup(0, 980, 8);
	ledcSetup(1, 980, 8);
	ledcAttachPin(ENA, 0);
	ledcAttachPin(ENB, 1);
}

void loop() {
/*
	Serial.print("");
	Serial.print(LT_L);
	Serial.print("");
	Serial.print(LT_M);
	Serial.print("");
	Serial.print(LT_R);
	Serial.println();
	delay(500);	
*/

  if(LT_M){
   	forward();
  }
  else if(LT_R) { 
    right();
    while(LT_R);                             
  }   
  else if(LT_L) {
    left();
    while(LT_L);  
  }
}

