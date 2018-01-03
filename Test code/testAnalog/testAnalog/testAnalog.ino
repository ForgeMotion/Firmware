const uint8_t teensyPinLUT[2][39] = {{1,2,9,10,11,12,18,26,27,28,29,35,36,37,38,39,40,41,42,43,44,45,46,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64},
{31,26,A10,A11,A12,A13,A14,33,24,3,4,16,17,19,18,0,1,32,25,15,22,23,9,10,13,11,12,28,27,29,30,2,14,7,8,6,20,21,5}};

uint8_t find(uint8_t val, const uint8_t *LUT, uint8_t len){
  for (int i = 0; i < len; i++){
    if(val == LUT[i]) return i;
  }
  return len+1;
}

int16_t teensyPinFromProcPin(uint8_t procPin){
  uint8_t index = find(procPin, teensyPinLUT[0], 39);
  if(index > 39) return -1;
  return teensyPinLUT[1][index];
}

int pwrSensePin = teensyPinFromProcPin(43);
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(pwrSensePin, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(pwrSensePin);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(1);        // delay in between reads for stability
}


