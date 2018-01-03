/*	MAX5395.cpp - Library for controlling MAX5395 digital potentiometer
	Written by Tyler McGahee, 2017
*/
#include "Arduino.h"
#include "MAX5395.h"
#include <Wire.h>

// Gets configuration data, returns 0xFF if failed
byte MAX5395::getConfig(){
	Wire.beginTransmission(addr);
	Wire.write(0x80);
	Wire.endTransmission();

	Wire.requestFrom(addr, 1);

	if(Wire.available()){
		return Wire.read();
	}
	return 0xFF;
}

// Function that can write all commands but wiper
void MAX5395::setConfig(byte cmd){
	Wire.beginTransmission(addr);
	Wire.write(cmd);
	Wire.write(0x00);
	Wire.endTransmission();
}

void MAX5395::setAddr(byte addrIn){
	addr = addrIn;
}

byte MAX5395::getAddr(){
	return addr;
}

void MAX5395::setAddrFromPinState(PinState state){
	switch (state) {
		case low:
			addr = 0x28;
			break;
		case high:
			addr = 0x29;
			break;
		default:
			addr = 0x2B;
			break;
	}
}

bool MAX5395::setWiperPosition(byte pos, bool confirm){
	Wire.beginTransmission(addr);
	Wire.write(0x00);
	Wire.write(pos);
	Wire.endTransmission();

	if(confirm) 
		return (getWiperPosition() == pos);

	return true;
}

byte MAX5395::getWiperPosition(){
	Wire.beginTransmission(addr);
	Wire.write(0x00);
	Wire.endTransmission();

	Wire.requestFrom(addr, 1);

	if(Wire.available()) 
		return Wire.read();

	return 0xFF;
}

bool MAX5395::setChargePumpStatus(bool enabled, bool confirm){
	setConfig(enabled?0xA1:0xA0);

	if(confirm) return getChargePumpStatus();

	return true;
}

bool MAX5395::getChargePumpStatus(){
	return getConfig() & 0x80;
}

void MAX5395::fullShutdown(Switch switchToOpen, WiperSetting setting){
	// Determine the command to send based on inputs
	byte cmd;

	if(switchToOpen == wiperSw)
		cmd = 0x84;
	else if(switchToOpen == lowSw)
		cmd = 0x88 + static_cast<uint8_t>(setting);
	else
		cmd = 0x90 + static_cast<uint8_t>(setting); 

	// Send shutdown command
	setConfig(cmd);
}

bool MAX5395::isShutdown(){
	byte c = getConfig();

	return (c & 0b11111) != 0;
}

/* returns 3 bits = 0bHWL. 0xFF for error
* H: H terminal switch status, 0 is closed, 1 is open
* W: W terminal switch status, 0 is closed, 1 is open
* L: L terminal switch status, 0 is closed, 1 is open
*/
byte MAX5395::getSwitchStatus(){
	byte c =  getConfig();

	if(c == 0xFF) return 0xFF;

	c = (c >> 2) & 0b111;
	return c; 
}

// Tap select can override the set wiper position
MAX5395::WiperSetting MAX5395::getWiperSetting(){
	byte c = getConfig();

	if(c == 0xFF) return wiperERR;

	if(c & 0b10)
		if(c & 0b01)
			return full;
		else
			return mid;
	else if(c & 0b01)
		return zero;

	return reg;
}

void MAX5395::clearShutdown(){
	setConfig(0x80);
}

void MAX5395::reset(){
	setConfig(0xC0);
}
