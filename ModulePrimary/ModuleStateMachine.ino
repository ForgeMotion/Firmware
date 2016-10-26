
#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// Declare variables 
// Motor control
	#include "SwingForgeMotors.h"
	sfMotor motor[7] = {
		{PROC_PIN_28, MOTOR_POS_FRONT},
		{PROC_PIN_41, MOTOR_POS_BACK_TOP},
		{PROC_PIN_42, MOTOR_POS_BACK_BOTTOM},
		{PROC_PIN_29, MOTOR_POS_SHOULDER_L},
		{PROC_PIN_46, MOTOR_POS_SHOULDER_R},
		{PROC_PIN_44, MOTOR_POS_SIDE_L},
		{PROC_PIN_45, MOTOR_POS_SIDE_R}
	}


// User settings
	float angleLimits[] = {6.0,-6.0,-7.0,7.0,25.0,25.0};   // [PitchBack,PitchFwd,YawR,YawL,RollBack,RollFwd]
	bool audioEnable[2] = {true,true}; // feedback enabled: {backswing, frontswing}
	bool leftHanded = false;
	uint32_t backSwingTimeout = 60000, frontSwingTimeout = 60000;


// IMU
	const uint8_t imuIntPin = PROC_PIN_64;

// GPIO
	const uint8_t battSensePin = PROC_PIN_11, killPin = PROC_PIN_26; // ==================================================
	const float battDivider = 4.3; // (33k+10k)/10k

	#include "RGBMood.h"
	RGBMood rgb(false, PROC_PIN_61, PROC_PIN_62, PROC_PIN_63);


	#include "Bounce2.h"
	const uint8_t volPins[] = {PROC_PIN_12, PROC_PIN_1}; // up, down
	Bounce volSw[2]; // vol up, vol down

// Power saving mode
	#include "Snooze.h"
	SnoozeBlock config;

// Audio
	const uint8_t audioShutdownPin = PROC_PIN_35;


void setup(){
// Initialize debug bus
	Serial.begin(115200);

// Check battery
	analogReference(INTERNAL);
	analogReadRes(16);
	analogReadAveraging(8);

	pinMode(battSensePin, INPUT);
	if(getBatteryLife(battSensePin, battDivider) <= 5){
		DEBUG_PRINTLN("Battery is critically low; shutting down");
		completeShutdown();
	}

// Initialize IMU


// Initialize serial flash


// Initialize audio
	pinMode(audioShutdownPin, OUTPUT);
	digitalWrite(audioShutdownPin, LOW);

	DEBUG_PRINTLN("Hello. Welcome to Swing Forge.");
	// Welcome audio file

// Prepare motors
	for(int i=0; i<7; i++) motor[i].init();

// Start motor intro


// Set up indicator led
	rgb.setMode(RGBMood::RAINBOW_HUE_MODE);  // Automatic random fade.
	rgb.setFadingSteps(200);   	// Fade with 200 steps.
	rgb.setFadingSpeed(25);    	// Each step last 25ms. A complete fade takes 25*200 = 5 seconds
	rgb.setHoldingTime(0);		// No need to stay red.
	rgb.fadeHSB(0, 255, 255); 	// Rainbow mode only change Hue so we first set the saturation and brightness.

	rgb.tick();
	
// Prepare button filters
	for(int i=0; i<2; i++){
		pinMode(volPins[i], INPUT_PULLUP);

		volSw[i] = Bounce();
		volSw[i].attach(volPins[i]);
		volSw[i].interval(5);
	}


// Configure sleep mode
    config.pinMode(imuIntPin, INPUT, RISING);
    config.setTimer(500);

// Initialize BLE


// Finish IMU calibration


// Audio/Visual cue for "ready"


// Begin state machine
	currentState = ST_IDLE;
}



void loop(){
	if(currentState != ST_BLE_PROGRAMMING){
		// Update pin readings
		for(int i=0; i<2; i++) volSw[i].update();


		// Check for BLE Data


		// Check battery
		x = getBatteryLife(battSensePin, battDivider);
		// if low, set led mode to red blink
		// if dying, audio cue + shutdown


		// Check volume - needs a timer so this doesn't occur too often
		if(volSw[0].fell()) changeVolume(1);
		else if(volSw[1].fell()) changeVolume(0);
	}


	currentState = runState(currentState);

	rgb.tick();
}




// ==================================== State machine definitions ====================================
// Enumerate the states of the state machine
enum state_t{
	ST_IDLE,
	ST_BACKSWING,
	ST_FRONTSWING,
	ST_BLE_PROGRAMMING,
	NUM_STATES
};

// Set up state functions - each must return the new currentState
typedef state_t state_func_t(); 


state_t doStIdle(){
	int who = Snooze.sleep(config); // 36 for timer, pin number for pin wakeup
	rgb.tick();

	if(who == imuIntPin) return ST_BACKSWING; // woken up by double tap on IMU!
}


state_t doStBswing(){

}


state_t doStFswing(){

}


state_t doStBLE(){

}


// Store state functions in lookup table
state_func_t* const stateTable[NUM_STATES] = {doStIdle, doStBswing, doStFswing, doStBLE};

// Function to run current state's function and transition if needed
state_t runState(state_t curState) {
    state_t newState = stateTable[curState]();

    transition_func_t *transition = transitionTable[newState][curState];
	if (transition) transition();

    return newState;
};

// Set up transition functions
typedef void transition_func_t();

// Store transition functions in lookup table (can be NULL even for valid transitions, can be duplicates)
transition_func_t * const transitionTable[NUM_STATES][NUM_STATES] = {
// Current State:	
// 	Idle 				Backswing 				Frontswing 			BLE				// New State (down)
	{NULL,				goBackswingToIdle,		goFrontswingToIdle,	NULL}, 	// Idle
	{goIdleToBackswing,	NULL,					NULL,				NULL},			// Backswing
	{NULL,				goBackswingToFrontswing,NULL,				NULL},			// Frontswing
	{NULL, 				NULL,					NULL,				NULL},			// BLE
};


// void goIdleToBLE(); 
void goIdleToBackswing();
void goBackswingToIdle();
// void goBackswingToBLE();
void goBackswingToFrontswing();
void goFrontswingToIdle();
// void goFrontswingToBLE();




uint8_t getBatteryLife(uint8_t pin, float divider) {
	uint16_t tempBatt = analogRead(pin);
	tempBatt = tempBatt * 1200 / 65535 * divider; // 1200mV/(2^16-1 counts)*divider constant

	DEBUG_PRINT("battery millivolts: ");
	DEBUG_PRINTLN(tempBatt);

	// convert millivolts to battery percentage (0-100)
	if (tempBatt <= 3700)
	tempBatt = 21 * tempBatt / 1000 - 67;
	else
	tempBatt = -244 * tempBatt * tempBatt / 1000000 + 2107 * tempBatt / 1000 - 4445;

	tempBatt = constrain(tempBatt, 0, 100);

	DEBUG_PRINT("battery percent: ");
	DEBUG_PRINTLN(tempBatt);

	return uint8_t(tempBatt);
}

void completeShutdown(){
	// disable motors
	for(int i=0; i<7; i++) motor[i].disable();

	// shut down BLE, IMU, audio
	SPI.end();
	Wire.end();
	digitalWrite(audioShutdownPin, LOW);

	pinMode(killPin, OUTPUT);
	digitalWrite(killPin, LOW);
}


void changeVolume(bool up){

}