
void setup() 
{
	Serial.begin(115200);

    for(int i = 1; i <= 100; i ++) {
	    Serial.print(i);
        if(((i%3)==0) && ((i%5)==0)) {
	    	Serial.println(" : KonicaMinolta");
        }
        else if((i%3)==0) {
	    	Serial.println(" : Konica");
        }
        else if((i%5)==0) {
	    	Serial.println(" : Minolta");
        }
        else {
	    	Serial.println();
	    }
    }
}

void loop() 
{
}