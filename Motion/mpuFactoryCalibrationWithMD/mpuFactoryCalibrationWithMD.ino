// This is meant to get the bias values for MPU-6050 from 
// the MD6.12 library and store them into eeprom locations
// First, we have to test that the 32 bit numbers will be written correctly


#include <EEPROM.h>

uint8_t gyroBiasStart = 10, accelBiasStart = 14;
int32_t gyroBias = -1000005, accelBias = -1000005;

void setup(){
	Serial.begin(1);
	while(!Serial);
	delay(500);
	
	Serial.print("gyrobias: ");
	Serial.print(gyroBias, BIN);
	Serial.print(", broken down: ");
	uint8_t temp[4];
	for(int i=0; i<4; i++) {
		temp[i] = uint8_t(gyroBias >> (8*i));
		Serial.print(temp[i],BIN);
	}
	Serial.println(" then reconstructed:");
	Serial.print("gyrobias: ");
	int32_t bias = 0;
	for(int i=0; i<4; i++) bias = (bias << 8) | temp[i];
	Serial.println(bias, BIN);
	Serial.println();

	// Serial.println("Press any key to continue");
	// while(!Serial.available());
}

void loop(){
	
}

/*
for(int i=0; i<4; i++){
    EEPROM.write(gyroBiasStart+i, uint8_t(gyroBias >> (8*i)));
    EEPROM.write(accelBiasStart+i, uint8_t(accelBias >> (8*i)));
}
*/