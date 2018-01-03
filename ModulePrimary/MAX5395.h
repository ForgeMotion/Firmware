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

		enum PinState {low, high, nc};
		enum Switch {swERR, highSw, lowSw, wiperSw};
		enum WiperSetting {wiperERR, reg, zero, mid, full};

		void setAddr(byte addrIn);
		void setAddrFromPinState(PinState state);
		byte getAddr();

		bool setWiperPosition(byte pos = 0, bool confirm = false);
		byte getWiperPosition();

		bool setChargePumpStatus(bool enabled = false, bool confirm = false);
		bool getChargePumpStatus();

		void fullShutdown(Switch switchToOpen, WiperSetting setting);
		bool isShutdown();
		byte getSwitchStatus();
		WiperSetting getWiperSetting();
		void clearShutdown();

		void reset();
};

#endif
