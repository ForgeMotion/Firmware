/*	MAX5395.h - Library for controlling MAX5395 digital potentiometer
	Written by Tyler McGahee, 2017
*/

#include "Arduino.h"
#ifndef MAX5395_h
#define MAX5395_h

class MAX5395 {
	byte addr = 0x2B;
	void setConfig(byte cmd);
	byte getConfig();

	public:
		MAX5395(){}
		MAX5395(byte addrIn) {addr = addrIn;}

		enum class PinState {low, high, nc};
		enum class Switch {high, low, wiper};
		enum class WiperSetting {reg, zero, mid, full};

		void setAddr(byte addrIn);
		void setAddrFromPinState(PinState state);
		byte getAddr();

		bool setWiperPosition(byte pos, bool confirm = false);
		byte getWiperPosition();

		bool setChargePumpStatus(bool enabled, bool confirm = false);
		bool getChargePumpStatus();

		void shutdown(Switch switchToOpen, WiperSetting setting);
		bool isShutdown();
		byte getSwitchStatus();
		WiperSetting getWiperSetting();
		void clearShutdown();

		void reset();
};

#endif