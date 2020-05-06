#include "DHT.h"

DHT dht(33,DHT11);  //2î‘É|Å[Ég

void setup() {
  Serial.begin(115200);
  Serial.println("DHT11 trial");
  dht.begin();

}

void loop() {

  delay(3000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  Serial.print("Humidity : ");
  Serial.println(h);
  Serial.print("Temperature : ");
  Serial.println(t);
}
