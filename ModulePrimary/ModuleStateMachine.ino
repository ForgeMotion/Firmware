
#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#include "SwingForgeStateMachine.h"
#include "SwingForgeMotors.h"

// Declare variables 
// GPIO
	const uint8_t battSensePin = PROC_PIN_11, killPin = PROC_PIN_26; // ==================================================
	const float battDivider = 4.3; // (33k+10k)/10k

	#include "RGBMood.h"
	RGBMood rgb(false, PROC_PIN_61, PROC_PIN_62, PROC_PIN_63); // common cathode, rgb pins

	#include "Bounce2.h"
	const uint8_t volPins[] = {PROC_PIN_12, PROC_PIN_1}; // up, down
	Bounce volSw[2]; // vol up, vol down

// Motor control
	SFMotor motor[NUM_MOTORS] = {
		{PROC_PIN_28, MOTOR_POS_FRONT},
		{PROC_PIN_41, MOTOR_POS_BACK_TOP},
		{PROC_PIN_42, MOTOR_POS_BACK_BOTTOM},
		{PROC_PIN_29, MOTOR_POS_SHOULDER_L},
		{PROC_PIN_46, MOTOR_POS_SHOULDER_R},
		{PROC_PIN_44, MOTOR_POS_SIDE_L},
		{PROC_PIN_45, MOTOR_POS_SIDE_R}
	};
//	RGBMood 

// User settings
	float angleLimits[] = {6.0,-6.0,-7.0,7.0,25.0,25.0};	// [PitchFwd,PitchBack,YawL,YawR,RollBack,RollFwd]
	uint8_t feedbackEnable = 0b00111111;					// bitmask for feedback enable, indices match limits above

	uint32_t backswingTimeout = 60000, frontswingTimeout = 60000;
	elapsedMillis swingTimeout = 0;
	bool swingSuccess = false;

// IMU
	#include <Wire.h>
	#include <EEPROM.h>
	// #include "I2Cdev.h"
	#include "inv_mpu.h"
	#include "inv_mpu_dmp_motion_driver.h"
	#include "quaternionMath.h"

	const uint8_t imuIntPin = PROC_PIN_64, IMU_HZ = 50;
	volatile bool imuDataReady = false, tapDetected = false;
	Quaternion currentQuat, initQuat;
	float YPRdev[] = {0.0, 0.0, 0.0};
	uint8_t gyroBiasStart = 10, accelBiasStart = 14; // starting addresses of 4 byte bias values in eeprom

// Power saving mode
	#include <Snooze.h>
	SnoozeBlock config;

// BLE


// Audio and serial flash
	#include <Audio.h>
	// #include <Wire.h> // required, but already included
	#include <SPI.h>
	#include <SerialFlash.h>
	#include "play_sd_mp3.h"

	// Shutdown is active low, gain NC = 0db, HIGH = 6dB, LOW = 12dB
	const uint8_t audioShutdownPin = PROC_PIN_35, audioGainPin = PROC_PIN_36;
	const uint8_t flashCSPin = PROC_PIN_40;

	AudioPlaySdMp3		playMp31;
	AudioMixer4			mixer1;
	AudioOutputAnalog	dac1;
	AudioConnection		patchCord1(playMp31, 0, mixer1, 0);
	AudioConnection		patchCord2(playMp31, 1, mixer1, 1);
	AudioConnection		patchCord3(mixer1, dac1);
	
// Time between samples for BLE, battery, volume respectively (ms)
	uint16_t updateInterval[3] = {2000, 5000, 100};
	elapsedMillis elapsed[3];


void setup(){
// Initialize audio
	// Allocate memory
  	AudioMemory(8);

	//Set Volume to max - levels must add to <=1 
	mixer1.gain(0, 0.5);
	mixer1.gain(1, 0.5);

	// Start communication to flash memory
	if (!SerialFlash.begin(flashCSPin)) {
		DEBUG_PRINTLN("Cannot access SPI Flash chip");
		delay (1000);
		completeShutdown();
	}

// Initialize debug bus
	Serial.begin(115200);

// Check battery
	analogReference(INTERNAL); // 1.2V ref
	analogReadRes(16);
	analogReadAveraging(8);

	pinMode(battSensePin, INPUT);
	if(getBatteryLife(battSensePin, battDivider) <= 5){
		DEBUG_PRINTLN("Battery is critically low; shutting down");
		delay(1000);
		completeShutdown();
	}

// Initialize IMU
	Wire.begin();

	initIMU(0x68, imuIntPin);
	dmp_set_tap_count(2);

	// int32_t bias = 0;
	// for(int i=0; i<4; i++) bias = (bias << 8) | EEPROM.read(gyroBiasStart+i);
	// dmp_set_gyro_bias(bias);

	// bias = 0;
	// for(int i=0; i<4; i++) bias = (bias << 8) | EEPROM.read(accelBiasStart+i);
	// dmp_set_accel_bias(bias);

// Prepare motors
	for(int i=0; i<NUM_MOTORS; i++) motor[i].init();

// Start motor intro


// Set up indicator led
	rgb.setMode(RGBMood::RAINBOW_HUE_MODE);
	rgb.setFadingSteps(100);	// Fade with 100 steps between colors
	rgb.setFadingSpeed(20);		// Each step last 20ms. A complete fade takes 20*100 = 2 seconds
	rgb.setHoldingTime(0);
	rgb.fadeHSB(0, 255, 255); 	// Rainbow mode only change Hue so set saturation and brightness.

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


// Ready! Reset IMU vars and timers, and give Audio/Visual cue
	tapDetected = false;
	for(int i=0; i<3; i++) elapsed[i] = 0;

	DEBUG_PRINTLN("Hello. Welcome to Swing Forge.");
	playAudioFile("1welcome.mp3");
	rgb.setRGB(Color::WHITE);
}



void loop(){
	if(currentState != ST_BLE_PROGRAMMING){
		// Update pin readings
		for(int i=0; i<2; i++) volSw[i].update();


		// Check for BLE connection and data
		if(elapsed[0] > updateInterval[0]){
			elapsed[0] = 0;
			
			//  =================================================================================
		}

		// Check battery
		if(elapsed[1] > updateInterval[1]){
			elapsed[1] = 0;

			uint8_t battPercent = getBatteryLife(battSensePin, battDivider);
			if(battPercent < 20) rgb.setMode(RGBMood::FIRE_MODE);
			else if(battPercent < 10){
				playAudioFile("5batteryisdownto10p.mp3");
				completeShutdown();
			}
		}

		// Check volume
		if(elapsed[2] > updateInterval[2]){
			elapsed[2] = 0;

			if(volSw[0].fell()) changeVolume(1);
			else if(volSw[1].fell()) changeVolume(0);
		}
	}


	currentState = runState(currentState);

	if(imuDataReady) processIMU();

	rgb.tick();
}

// ==================================== State machine functions ====================================
state_t runIdleState(){
	// if a double tap happened in loop(), start swing!
	if(tapDetected){
		tapDetected = false;
		return ST_BACKSWING;
	}

	int who = Snooze.sleep(config); // 36 for timer, pin number for pin wakeup

	rgb.tick(); // (make sure we have a blinky light on wakeup)

	if(who == imuIntPin){ // or if a double tap woke us from our sleep!
		tapDetected = false;
		return ST_BACKSWING;
	}

	return ST_IDLE;
}

void goIdleToBackswing(){
	tone(motor[MOTOR_POS_FRONT].pin(), 488.28, 300);
	delay(600);
	tone(motor[MOTOR_POS_FRONT].pin(), 488.28, 400);

	if(rgb.mode_ != RGBMood::FIRE_MODE){
		rgb.setMode(RGBMood::GREEN_MODE);
		rgb.setFadingSteps(10);
		rgb.setFadingSpeed(40);
	}
	rgb.tick();

	if(feedbackEnable & (1<<4)) 
		playAudioFile("0chime1.mp3");

	dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_6X_LP_QUAT);
	// dmp_set_tap_count(1);

	swingTimeout = 0;
	while(swingTimeout < 100 && !imuDataReady) rgb.tick();

	if(imuDataReady) processIMU();
	else DEBUG_PRINTLN("ERROR: Quats enabled, but no FIFO interrupt received!");

	initQuat = currentQuat;
	swingTimeout = 0;
	swingSuccess = false;
}

state_t runBackswingState(){
	getYPRDev(&initQuat,&currentQuat,YPRdev);
	updateMotors(200);
	printStatus();

	// Auto detects handedness! If user rolled more than the limit, that's their backswing. 
	if (abs(YPRdev[2]) > abs(angleLimits[4])){
		angleLimits[5] = abs(angleLimits[5])*(YPRdev[2]<0?1:-1); // Adjust the frontswing limit to match handedness
		return ST_FRONTSWING;
	}

	if(swingTimeout > backswingTimeout) return ST_IDLE;

	if(tapDetected){
		tapDetected = false;
		return ST_IDLE;
	}

	return ST_BACKSWING;
}

void goBackswingToFrontswing(){
	if(feedbackEnable & (1<<4)) 
		playAudioFile("0chime1.mp3");

	if(rgb.mode_ != RGBMood::FIRE_MODE) {
		// speed up the light effects!
		rgb.setFadingSteps(5);
		rgb.setFadingSpeed(30);
	}

	swingTimeout = 0;
}

state_t runFrontswingState(){
	getYPRDev(&initQuat,&currentQuat,YPRdev);
	updateMotors(200);
	printStatus();

	if((angleLimits[5] > 0 && YPRdev[2] > angleLimits[5]) || (angleLimits[5] < 0 && YPRdev[2] < angleLimits[5])) {
		swingSuccess = true;
		return ST_IDLE;
	}

	if(swingTimeout > frontswingTimeout) return ST_IDLE;

	if(tapDetected){
		tapDetected = false;
		return ST_IDLE;
	}

	return ST_FRONTSWING;
}

void goSwingToIdle(){
	turnOffMotors();

	if(feedbackEnable & (1<<5)) 
		playAudioFile("0chime1.mp3");

	if(rgb.mode_ != RGBMood::FIRE_MODE){
		rgb.setMode(RGBMood::RAINBOW_HUE_MODE);
		rgb.setFadingSteps(100);
		rgb.setFadingSpeed(20);
		rgb.fadeHSB(0, 255, 255);
	}

	dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_GYRO_CAL);
	dmp_set_tap_count(2);
}

state_t runBLEState(){
	// Process serial data into new settings

	// Update light effects
	rgb.tick();

	// Audio: "Upload complete" || "upload failed, please try again"
	playAudioFile("4settingssuccessupdated.mp3");

	// Serial.flush()
	return ST_IDLE;
}

void goToBLE(){
	dmp_enable_feature(DMP_FEATURE_GYRO_CAL);
	dmp_set_tap_count(2);

	turnOffMotors();

	if(rgb.mode_ != RGBMood::FIRE_MODE){
		rgb.setMode(RGBMood::BLUE_MODE);
		rgb.setFadingSteps(10);
		rgb.setFadingSpeed(20);
	}
	rgb.tick();
}

void goBLEToIdle(){
	dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_GYRO_CAL);
	dmp_set_tap_count(2);

	if(rgb.mode_ != RGBMood::FIRE_MODE){
		rgb.setMode(RGBMood::RAINBOW_HUE_MODE);
		rgb.setFadingSteps(100);
		rgb.setFadingSpeed(20);
		rgb.fadeHSB(0, 255, 255);
	}
	rgb.tick();
}


// ==================================== General helper functions ====================================

uint8_t getBatteryLife(uint8_t pin, float divider) {
	uint16_t tempBatt = analogRead(pin);
	tempBatt = tempBatt * 1200 / 65535 * divider; // 1200mV/(2^16-1 counts)*divider constant

	DEBUG_PRINT("battery millivolts: ");
	DEBUG_PRINT(tempBatt);

	// convert millivolts to battery percentage (0-100)
	if (tempBatt <= 3700)
	tempBatt = 21 * tempBatt / 1000 - 67;
	else
	tempBatt = -244 * tempBatt * tempBatt / 1000000 + 2107 * tempBatt / 1000 - 4445;

	tempBatt = constrain(tempBatt, 0, 100);

	DEBUG_PRINT("; battery percent: ");
	DEBUG_PRINTLN(tempBatt);

	return uint8_t(tempBatt);
}

void completeShutdown(){
	// disable motors
	for(int i=0; i<7; i++) motor[i].disable();

	// shut down BLE, IMU, audio
	// SPI.end();
	Wire.end();
	digitalWriteFast(audioShutdownPin, LOW);

	pinMode(killPin, OUTPUT);
	digitalWriteFast(killPin, LOW);
}

void updateMotors(uint8_t motorSpeed){
	if(feedbackEnable & (1<<0)) motor[MOTOR_POS_FRONT].setSpeed(		(YPRdev[1] > angleLimits[0]) * motorSpeed);
	if(feedbackEnable & (1<<1)) motor[MOTOR_POS_BACK_TOP].setSpeed(		(YPRdev[1] < angleLimits[1]) * motorSpeed);
	if(feedbackEnable & (1<<1)) motor[MOTOR_POS_BACK_BOTTOM].setSpeed(	(YPRdev[1] < angleLimits[1]) * motorSpeed);
	if(feedbackEnable & (1<<2)) motor[MOTOR_POS_SHOULDER_L].setSpeed(	(YPRdev[0] > angleLimits[2]) * motorSpeed);
	if(feedbackEnable & (1<<3)) motor[MOTOR_POS_SHOULDER_R].setSpeed(	(YPRdev[0] > angleLimits[3]) * motorSpeed);
	if(feedbackEnable & (1<<2)) motor[MOTOR_POS_SIDE_L].setSpeed(		(YPRdev[0] > angleLimits[2]) * motorSpeed);
	if(feedbackEnable & (1<<3))	motor[MOTOR_POS_SIDE_R].setSpeed(		(YPRdev[0] > angleLimits[3]) * motorSpeed);
}

void turnOffMotors(){
	for(int i=0; i<NUM_MOTORS; i++) motor[i].digitalWrite(0);
}

void printStatus(){
	DEBUG_PRINT("Dev/lims: P: ");
	DEBUG_PRINT(YPRdev[1]); // pitch
	DEBUG_PRINT(" /+");
	DEBUG_PRINT(angleLimits[0]); // bwd lim
	DEBUG_PRINT("/");
	DEBUG_PRINT(angleLimits[1]); // fwd lim
	DEBUG_PRINT("  Y: ");
	DEBUG_PRINT(YPRdev[0]); // yaw
	DEBUG_PRINT(" /+");
	DEBUG_PRINT(angleLimits[2]); // right lim
	DEBUG_PRINT("/");
	DEBUG_PRINT(angleLimits[3]); // left lim
	DEBUG_PRINT("  R: ");
	DEBUG_PRINT(YPRdev[2]); // roll
	DEBUG_PRINT(" /+");
	DEBUG_PRINT(angleLimits[4]); // back lim
	DEBUG_PRINT("/");
	DEBUG_PRINT(-angleLimits[5]); // right lim
	DEBUG_PRINT("  ");
}

void playAudioFile(const char *filename, bool block = false){
  SerialFlashFile ff = SerialFlash.open(filename);
  DEBUG_PRINT("Playing file: ");
  DEBUG_PRINTLN(filename);

  uint32_t sz = ff.size();
  uint32_t pos = ff.getFlashAddress();

  // Start playing the file, non-blocking
  playMp31.play(pos,sz);

  // Simply wait for the file to finish playing.
  while (block && playMp31.isPlaying()) {}
}

void changeVolume(bool up){ // ====================================

	playAudioFile("volumebup.mp3");

}

// ==================================== IMU Helper functions ====================================
void initIMU(uint16_t addr, uint8_t interruptPin){
	struct int_param_s int_param;
	int_param.cb = imuInterrupt;
	int_param.pin = interruptPin;

//	st.hw->addr = addr;
	mpu_init(&int_param);
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	// mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(IMU_HZ);
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(0x88); // {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} orientation matrix
	dmp_enable_feature(DMP_FEATURE_TAP | DMP_FEATURE_GYRO_CAL);
	dmp_set_fifo_rate(IMU_HZ);
	mpu_set_dmp_state(1);

	dmp_register_tap_cb(tap_cb);
	dmp_set_tap_thresh(TAP_Z, 250);
	dmp_set_tap_axes(TAP_Z);
	// dmp_set_tap_time(100);
	// dmp_set_tap_time_multi(500);
}

uint8_t tapData[] = {0,0};
void tap_cb(unsigned char dir, unsigned char count){
	tapDetected = true;
	tapData[0] = dir; tapData[1] = count;

	DEBUG_PRINT("Tap detected! Direction: ");
	DEBUG_PRINT(dir);
	DEBUG_PRINT(" count: ");
	DEBUG_PRINTLN(count);
}

static void imuInterrupt(){
	imuDataReady = true;
}

void longArrayToQuat(long *input, Quaternion *quat){
	quat->w = float(input[0])/0x40000000;
	quat->x = float(input[1])/0x40000000;
	quat->y = float(input[2])/0x40000000;
	quat->z = float(input[3])/0x40000000;
}

void processIMU(){
	short gyro[3], accel[3], sensors;
	unsigned char more;
	int ret;
	unsigned long timestamp;

	long quat[4];

	if (0 == (ret = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))){
		if (!more) imuDataReady = false;

		longArrayToQuat(quat, &currentQuat);
	}
}
