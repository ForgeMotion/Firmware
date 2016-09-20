
#include "SwingForgeMotors.h"

sfMotor::sfMotor(int16_t pin, int16_t pos){
	_pin = pin;
	position = pos;
}

void sfMotor::init(){
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, 0);
}

void sfMotor::disable(){
	_enable = false;
	digitalWrite(_pin, 0);
}

int16_t sfMotor::enable(int16_t speed){
	_enable = true;
	return setSpeed(speed);
}

int16_t sfMotor::setSpeed(int16_t speed){
	if(speed > 255 || speed < 0) return -1
	
	_speed = speed;
	analogWrite(_pin, _enabled ? _speed : 0);

	return 0;
}

int16_t sfMotor::getSpeed(){
	return _speed;
}

int16_t sfMotor::getPin(){
	return _pin;
}
