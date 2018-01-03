
#include "SwingForgeMotors.h"

SFMotor::SFMotor(int16_t pin, motor_position_e pos){
	_pin = pin;
	position = pos;
}

SFMotor::SFMotor(int16_t pin, motor_position_e pos, bool inverted){
	_inverted = inverted;
	SFMotor(pin, pos);
}

void SFMotor::init(){
	pinMode(_pin, OUTPUT);
	digitalWriteFast(_pin, _inverted?1:0);

	_enabled = true;
}

void SFMotor::disable(){
	digitalWriteFast(_pin, _inverted?1:0);
	pinDisable(_pin);

	_enabled = false;
}


bool SFMotor::setSpeed(int16_t speed){
	if(speed > 255 || speed < 0 || !_enabled) return false;
	
	analogWrite(_pin, _inverted?255-_speed:_speed);
	_speed = speed;

	return true;
}


bool SFMotor::digitalWrite(bool on){
	if(!_enabled) return false;

	digitalWriteFast(_pin, _inverted?!on:on);
	_speed = on?255:0;

	return true;
}


void pinDisable(const uint8_t pin) {
	volatile uint32_t *config;  
	if (pin >= CORE_NUM_TOTAL_PINS) return;
	config = portConfigRegister(pin);
	*config = 0;
}
