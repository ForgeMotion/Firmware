// Test all functions
// This is set up for PCB rev 1.1 (yellow)

const int motors[] = {4,5,6,20,21,22,23};
const int volUp = 15, volDown = 2, pwrCtrl = 1, battReadPin = A3;
const int rgbLed[] = {14,9,10};
const int csBLE = 16, csFlash = 8;

const int analogResolution = 10;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(volUp, INPUT_PULLUP);
  pinMode(volDown, INPUT_PULLUP);
  pinMode(battReadPin, INPUT);

  for(int i=0; i<7; i++){
    pinMode(motors[i],OUTPUT);
    digitalWrite(motors[i],LOW);
  }

  for(int i=0; i<3; i++){
    pinMode(rgbLed[i],OUTPUT);
    digitalWrite(rgbLed[i],HIGH);
  }

  Serial.println("Please press volume up");
  while(digitalRead(volUp)){ delay(1); }

  Serial.println("Volume up works. Please press volume down");
  while(digitalRead(volDown)){ delay(1); }


  Serial.print("Volume down works. Power supply/battery voltage: ");
  float voltage = 4.0*3.3*analogRead(battReadPin)/(1023.0);
  Serial.print(voltage);
  Serial.println(" volts");
  delay(2000);
  
  Serial.println();
  Serial.println("Testing RGB LED. Press volume up to continue");
  while(digitalRead(volUp)){ delay(1); }
  
  bool success = false;
  while(!success){
    Serial.println("Red");
    digitalWrite(rgbLed[0], LOW);
    delay(2000);
    digitalWrite(rgbLed[0], HIGH);
  
    Serial.println("Green");
    digitalWrite(rgbLed[1], LOW);
    delay(2000);
    digitalWrite(rgbLed[1], HIGH);
  
    Serial.println("Blue");
    digitalWrite(rgbLed[2], LOW);
    delay(2000);
    digitalWrite(rgbLed[2], HIGH);
    delay(1000);
    
    Serial.println("Test complete. Press volume up to continue or volume down to repeat the LED test");
    bool buttonPressed = false;
    while(!buttonPressed){ 
      success = !digitalRead(volUp);
      buttonPressed = !digitalRead(volDown) || success;
      if(buttonPressed && !success) Serial.println("Repeating LED test.");
      delay(1); 
    }
    
  }
  
  Serial.println("\n\rTesting motors and motor drivers.");
  delay(1000);
  for(int i=0; i<7; i++){
    Serial.print("Pulsing motor: ");
    Serial.println(i+1);
    digitalWrite(motors[i],HIGH);
    delay(1000);
    digitalWrite(motors[i],LOW);
    delay(300);
  } 


  Serial.println();
  Serial.println("Almost done; about to test shutdown function. ");
  Serial.println("To do: add audio testing and BLE and SPI Flash testing.");
  delay(1000);
  Serial.println("Shutting down.");

  pinMode(pwrCtrl, OUTPUT);
  digitalWrite(pwrCtrl, LOW);

  delay(5000);
  Serial.println();
  Serial.println("...");
  delay(10000);
  Serial.println("Crap, why am I still here? What will become of me now?");
}

void loop(){ 
}