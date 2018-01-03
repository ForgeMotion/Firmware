#define FREQUENCY_HZ  500
#define V_PEAK_TO_PEAK  2.0
#define DC_OFFSET_V   1.65

#define TWO_PI      6.28319
#define SCALE_CONST   (TWO_PI*FREQUENCY_HZ)/100000L

#include <Wire.h>

#define WIPER     0x00
#define SD_CLR    0x80
#define SD_H_WREG 0x90
#define SD_H_ZERO 0x91
#define SD_H_MID  0x92
#define SD_H_FULL 0x93
#define SD_L_WREG 0x88
#define SD_L_ZERO 0x89
#define SD_L_MID  0x8A
#define SD_L_FULL 0x8B
#define SD_W      0x84
#define QP_OFF    0xA0
#define QP_ON     0xA1
#define RST       0xC0

int delayTime = 5;
float phaseDelta = delayTime*SCALE_CONST; 

const float dcOffset = DC_OFFSET_V/3.3*4096, amplitude = V_PEAK_TO_PEAK/3.3*4096/2;

float phase = 0.0;
elapsedMicros usec = 0;

/* Address is determined by state of pin ADDR0
 * State  Addr
 *  LOW   0x28
 *   NC   0x29
 * HIGH   0x2B  */
const byte addr = 0x28;

byte pos = 0;

void setup(){
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  while(!Serial);
  analogWriteResolution(12);
  
  Serial.println("Reseting all configurations.");
  setConfig(addr,RST);
  Serial.println("Done.");
  delay(5);
  Serial.print("Charge pump: ");
  Serial.print(chargePumpEnabled(addr) ? "enabled" : "disabled");
  Serial.print(", Wiper position: ");
  Serial.println(wiperPosition(addr));

  pos = 127;
  Serial.print("Setting wiper to position: ");
  Serial.println(pos);

  bool success = setWiper(addr, pos, true);

  if(success){
    pinMode(21, OUTPUT);
    digitalWrite(21,HIGH);
    
    Serial.print("Position from register: ");
    Serial.println(wiperPosition(addr));
    Serial.println("Position set successfully!");
  }
  else {
    Serial.print("Failed to set position. Register set to: ");
    Serial.println(wiperPosition(addr));
  }
}

void loop(){
  float val = amplitude * sin(phase) + dcOffset;
  analogWrite(A14, (int)val);

  phase += phaseDelta;
  if (phase >= TWO_PI) phase = 0;
  
  while (usec < delayTime) ; // wait
  usec = usec - delayTime;
}


byte wiperPosition(byte addr){
  Wire.beginTransmission(addr);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.requestFrom(addr, 1);

  if(Wire.available()) return Wire.read();
  return 0xFF;
}


bool chargePumpEnabled(byte addr){
  return getConfig(addr) & 0x80;
}

/* returns 3 bits = 0bHWL. 0xFF for error
 * H: H terminal switch status, 0 is closed, 1 is open
 * W: W terminal switch status, 0 is closed, 1 is open
 * L: L terminal switch status, 0 is closed, 1 is open
 */
byte switchStatus(byte addr){
  byte c =  getConfig(addr);
  
  if(c == 0xFF) return 0xFF;

  c = (c >> 2) & 0b111;
  return c; 
}

/* returns 2 bits for tap select or 0xFF for error:
 * 00- wiper is at contents of wiper register
 * 01 – wiper is at 0x00
 * 10 – wiper is at 0x80
 * 11 – wiper is at 0xFF. 
 */
byte tapSelect(byte addr){
  byte c =  getConfig(addr);
  
  if(c == 0xFF) return 0xFF;

  c = c & 0b11;
  return c; 
}

byte getConfig(byte addr){
  Wire.beginTransmission(addr);
  Wire.write(0x80);
  Wire.endTransmission();
  
  Wire.requestFrom(addr, 1);

  if(Wire.available()){
    return Wire.read();
  }
  return 0xFF;
}


bool setWiper(byte addr, byte pos, bool confirm){
  Wire.beginTransmission(addr);
  Wire.write(WIPER);
  Wire.write(pos);
  Wire.endTransmission();

  if(confirm) return (wiperPosition(addr) == pos);

  return true;
}

void setConfig(byte addr, byte cmd){
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.write(0x00);
  Wire.endTransmission();
}

