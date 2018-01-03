// Test all functions
/* This is set up for PCB rev 4.2, 9/1/17 (unfinished)
Pins are updated.

List of tests:
x  2 Volume buttons
x  Battery sense
x  RGB Led
x  7 motor channels
x  Current monitor
  IMU
  Flash memory
    connect
    Empty flash
    Write needed audio files
  Audio
    Plays music
    Change amp gain
    Change volume with digipot
    Shutdown audio amp
  BLE
    Successful serial config
    Sleep BLE module
  Power switch
    Detect
    Switch voltage in range
x    Shutdown
*/

const int motors[] = {6,9,10,22,23,25,32};
const int volUp = 31, volDown = 27;
const int pwrSwSense = 15, iSense = A11, pwrCtrl = A12, battReadPin = A10;
const int rgbLed[] = {5,21,20};
const int csFlash = 1, imuInt = 28;
const int audioGain = 17, audioShutdown = 16;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(volUp, INPUT);
  pinMode(volDown, INPUT);
  pinMode(pwrSwSense, INPUT);
  pinMode(iSense, INPUT);
  pinMode(battReadPin, INPUT);
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
  float voltage = 4.3*3.3*analogRead(battReadPin)/(1023.0);
  Serial.print(voltage);
  Serial.println(" volts");
  Serial.print("Current consumption right now: ");
  float current = 3.3*analogRead(iSense)/(1023.0);
  Serial.print(current,3);
  Serial.println(" amps");
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

/*
R 64  5
g 63  21
b 62  20
1 61  6
2 46  9
3 49  10
4 44  22
5 45  23
6 42  25
7 41  32
pwr_ctrl  11  A12
pwr_sw  43  15
i_sense 10  A11
batt sense  9 A10
vol up  1 31
vol down  54  27
audio shdn  35  16
audio gain  36  17
imu int 53  28
flash cs  40  1
*/