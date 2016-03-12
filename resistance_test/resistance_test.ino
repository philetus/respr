// the value of the 'other' resistor
#define SERIESRESISTOR 4700
 
// What pin to connect the sensor to
#define SENSORPIN A0 
 
void setup(void) {
  Serial.begin(115200);
}
 
void loop(void) {
  float reading;
 
  reading = analogRead(SENSORPIN);
 
  //Serial.print("Analog reading "); 
  Serial.println(reading);
 
  // convert the value to resistance
  //reading = (1023 / reading)  - 1;
  //reading = SERIESRESISTOR / reading;
  //Serial.print("Thermistor resistance "); 
  //Serial.println(reading);
 
  delay(30);
}
