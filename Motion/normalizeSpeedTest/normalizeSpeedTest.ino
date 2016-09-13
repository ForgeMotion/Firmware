/*This is code to test a new function that casts type-long quaternions to floats
 * The original function takes the magnitude of the quaternion (type casting to float then root rum sqaure) and divides each component by it
 * The new function recognizes that the quaternions are normalized, but left shifted by 30 bits.
 * Speed improvment: 25200ns to 41ns (620x speed)
 * Accuracy improvement: 99.9953% to 99.999965% (128x more accurate)
 * 
 */
long test[] = {1073721088, 6588252, 1000846, 346229};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Running >>30 magnitude function 10k times");
  unsigned long t = micros();
  for(int i=0; i<10000; i++){
    for(int j=0; j<4; j++){
      float out = float(test[j])/0x40000000;
    }
  }
  unsigned long delta = micros()-t;
  Serial.print("Time (us): ");
  Serial.println(delta);

  
  Serial.println("Running normalize function 10k times");
  t = micros();
  for(int i=0; i<10000; i++){
    norm(test);
  }
  delta = micros()-t;
  Serial.print("Time (us): ");
  Serial.println(delta);

  Serial.println("Time to loop 10k times");
  t = micros();
  for(int i=0; i<10000; i++){
    float out = 0.0;
  }
  delta = micros()-t;
  Serial.print("Time (us): ");
  Serial.println(delta);
  delay(2000);
}

void norm(long *input){
  float quat[4];
    for(int i=0; i<4; i++)
        quat[i] = float(input[i]);
        
    float length = sqrt(quat[0] * quat[0] + quat[1] * quat[1] +  
        quat[1] * quat[1] + quat[1] * quat[1]);

    if (length == 0){
        Serial.println("mag is zero somehow");
        return;
    }

    for(int i=0; i<4; i++)
        quat[i] /= length;
}

