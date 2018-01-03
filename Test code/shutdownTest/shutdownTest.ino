// shutdown test code
#include "SwingForgeMotors.h"

const int sdPin = PROC_PIN_11, rLED = PROC_PIN_64, gLED = PROC_PIN_63, buttPin = PROC_PIN_1;

void setup(){
  pinMode(rLED, OUTPUT);
  digitalWrite(rLED, LOW);
  
	// begin serial. if button is pressed, wait for serial to connect
	Serial.begin(9600);
	pinMode(buttPin, INPUT);
	if(digitalRead(buttPin)) while(!Serial);

	// confirm awake
	Serial.println("WASSUP");

	// Blink LED 3 times
//	pinMode(gLED, OUTPUT);
//	digitalWrite(gLED, HIGH);
//	delay(500);
//	digitalWrite(gLED, LOW);
//	delay(500);
//	digitalWrite(gLED, HIGH);
//	delay(500);
//	digitalWrite(gLED, LOW);
//	delay(500);
//	digitalWrite(gLED, HIGH);
//	delay(500);
//	digitalWrite(gLED, LOW);

  flash(sdPin, 150);

	// confirm shutting down
	Serial.println("Shutting down, bro.");

	// shut down
	shutdown1();

	// if shutdown failed, report
	delay(500);
	Serial.println("Uh oh. I should have shut down by now.");
}

void loop(){
	
}

void flash(int pin, int times){
  pinMode(pin, OUTPUT);
  for(int i=0; i<times; i++){
    digitalWrite(pin, HIGH);
    delay(2000);
    digitalWrite(pin, LOW);
    delay(2000);
  }
}

void shutdown1(){
	pinMode(sdPin, OUTPUT);
	digitalWrite(sdPin, LOW);
}
