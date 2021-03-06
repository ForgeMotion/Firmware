/*	Swing Forge V2 Firmware

*/

#define GO_TIMEOUT_PRE_MS 60000;  // for timeout function before swing starts
#define GO_TIMEOUT_POST_MS 60000; // for timeout function after swing starts
#define DEBOUNCE_MAX_MS 500;        // debouncing on go button



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
	int16_t angleLimits[] = {6,-6,-7,7,25,25};   // [PitchBack,PitchFwd,YawR,YawL,RollBack,RollFwd]
	bool audioEnable[2] = {true,true}; // feedback enabled: {backswing, frontswing}
	bool leftHanded = false;


// Analyzing Swing
	bool backSwingCompleted = 0, firstGo = 1, firstCalDone = 0;
	const int16_t goButtonPin = 3, goInterruptNum = 1;
	volatile unsigned long nowGo = 0, lastGo = 0;
	volatile bool goState = 0;
	unsigned long goStartTime = 0, firstCalTime = 0;
elapsedMillis 
// Battery and power
	const int16_t battVoltsPin = PROC_PIN_11, killPin = PROC_PIN_26;

// IMU


void setup() { 
// Initialize communication busses

	Serial.begin(115200);

// Set GPIOs
	pinMode(battVoltsPin, INPUT);
	
	for(int i=0; i<7; i++) motor[i].init();
	 
	// For GO button and LED
	pinMode(goButtonPin, INPUT_PULLUP);
	pinMode(goLedPin, OUTPUT);
	
	// For Bluetooth
	pinMode(bleButtonPin, INPUT_PULLUP);
	pinMode(bleLedPin, OUTPUT);

// Prepare attached devices
	// Initialize audio module
	AudioMan.reset();
	DEBUG_PRINTLN("Hello. Welcome to Swing Forge.");
	// checkLowBatt(); // If power is low, turn off system, must be after AudioMan.reset, when using lipo
	AudioMan.playVoice(0); // "Welcome to Swing Forge!"

	// Initialize RTC
		// rtc.begin(); // Must be BEFORE SD datalogger
	
	// Initialize SD datalogger
		// initializeSD(CSPin); 
		// getLimitsFromSD(); // Open the default profile from SD card

// Begin IMU datastream 
	// Initialize IMU
	DEBUG_PRINTLN(F("Initializing sensor..."));
	mpu.initialize();
	DEBUG_PRINTLN(F("Testing sensor connections..."));
	DEBUG_PRINTLN(mpu.testConnection() ? F("Sensor connection successful!") : F("Sensor connection failed"));

	// load and configure the DMP
	DEBUG_PRINTLN(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	digitalWrite(goLedPin, HIGH);

	// mpu.setXAccelOffset(-3841);
	// mpu.setYAccelOffset(-1025);
	// mpu.setZAccelOffset(512);
	// mpu.setXGyroOffset(-33);
	// mpu.setYGyroOffset(31);
	// mpu.setZGyroOffset(95);
	
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		DEBUG_PRINTLN(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		DEBUG_PRINTLN(F("Enabling interrupt detection..."));
		attachInterrupt(IMUInterruptNum, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set DMP Ready flag so the main loop() function knows it's okay to use it
		DEBUG_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		firstCalTime = millis();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		DEBUG_PRINT(F("DMP Initialization failed (code "));
		DEBUG_PRINT(devStatus);
		DEBUG_PRINTLN(F(")"));
	}

// Attach interrupts
	attachInterrupt(goInterruptNum, processGoInterrupt, FALLING);
	attachInterrupt(bleInterruptNum, processBleInterrupt, FALLING); 
}

void loop() {
// If IMU won't start, can't do anything
if (!dmpReady) {
	DEBUG_PRINTLN("Sensor Error!");
	AudioMan.playVoice(2); // IMU error audio
	digitalWrite(killPin,HIGH); // kill power
	while(1);
}

// IMU time to calibrate
if(!firstCalDone && ((millis() - firstCalTime) > firstCalPeriod)){
	AudioMan.asyncPlayVoice(9); // "Ready"  
	firstCalDone = 1; 
	goState = 0;
	DEBUG_PRINTLN("IMU cal'd");
}

if(bleState){ // If user wants to program settings
	bleProgram();
}
else{ // Analyze the swing
	while (!mpuInterrupt && fifoCount < packetSize) {
		if(firstCalDone){
		if (!goState){
			if (!firstGo){ // if user pressed button to end swing
			endSwing();
			}

			goLedTime = millis();
			if(goLedTime - lastLedTime > readyBlinkTime) {
			lastLedTime = goLedTime;
			digitalWrite(goLedPin, goLedState);
			goLedState = !goLedState;
			}
		}
		else {
			if (firstGo){
			DEBUG_PRINTLN("FIRST GO");
			digitalWrite(goLedPin, HIGH);
			goStartTime = millis(); // initialize timer for timeout
			AudioMan.asyncPlayVoice(10); // "Running"
			
			qInitPos = qCurrent.getConjugate();
			
			firstGo = 0;
			}

			// Calculate change in angles from start
			if(!qDevCalcd){
			calcYPRDev();
			qDevCalcd = 1;
			}
			
			// Turn on motors if appropriate limits were broken
			FrMotTrig = (yprDev[1] > angleLimits[0]); // if leaned fwd
			ReMotTrig = (yprDev[1] < angleLimits[1]); // if leaned back
			RiMotTrig = (yprDev[0] > angleLimits[3]); // if yawed right
			LeMotTrig = (yprDev[0] < angleLimits[2]); // if yawed left
			
			triggerMotors();
			printStatus();
			
			if(!backSwingCompleted){
			if (!leftHanded && yprDev[2] <= -angleLimits[4]){ // if rolled back far enough, the backSwing completed
				DEBUG_PRINTLN("=======Back swing completed!=========");
				if(bkSwiEnable){
				AudioMan.asyncPlayVoice(11); // "beep"
				}
				goStartTime = millis(); // re-initialize timer for timeout
				backSwingCompleted = 1;
			}
			else if (leftHanded && yprDev[2] >= angleLimits[4]){ // if rolled back far enough, the backSwing completed
				DEBUG_PRINTLN("=======Back swing completed!=========");
				if(bkSwiEnable){
				AudioMan.asyncPlayVoice(11); // "beep"
				}
				goStartTime = millis(); // re-initialize timer for timeout
				backSwingCompleted = 1;
			}
			else if ((millis() - goStartTime) > GO_TIMEOUT_PRE_MS){ // haven't completed backswing before timeout
				DEBUG_PRINTLN("Timed out on backswing");
				endSwing();
			}
			} 
			else {
			if (!leftHanded && yprDev[2] >= angleLimits[5]){ // if rolled forward enough, swing has ended
				DEBUG_PRINTLN("Swing completed!");
				endSwing();
			}
			else if (leftHanded && yprDev[2] <= -angleLimits[5]){ // if rolled forward enough, swing has ended
				DEBUG_PRINTLN("Swing completed!");
				endSwing();
			}
			else if ((millis() - goStartTime) > GO_TIMEOUT_POST_MS){ // check for swing timeout
				DEBUG_PRINTLN("Timed out on forward swing");
				endSwing();
			}
			}
		}
		}
	}
	processIMU();
	qDevCalcd = 0;
}
}


void dmpDataReady() {
mpuInterrupt = true;
}

void processIMU(){
// If the MPU interrupt triggers or fifocount gets too big, get IMU data
mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();
fifoCount = mpu.getFIFOCount();
if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check for overflow (this should never happen unless our code is too slow)
		mpu.resetFIFO();
		DEBUG_PRINTLN(F("FIFO overflow!"));
} 
else if (mpuIntStatus & 0x02) {
	while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	mpu.getFIFOBytes(fifoBuffer, packetSize);
	fifoCount -= packetSize;

	mpu.dmpGetQuaternion(&qCurrent, fifoBuffer);
	qNorm = qCurrent.getNorm();
	if(qNorm >= 1.01 || qNorm < 0.99 ){
		qCurrent.normalize();
		DEBUG_PRINTLN("Had to normalize quat from sensor");
	}
}
}

void calcYPRDev(){
// qInitPos = conjugate of initial pos quat
// qDev = qInitPos * qCurrent
qDev = qInitPos.getProduct(qCurrent);
mpu.dmpGetEuler(yprDev, &qDev);
for(int16_t i=0; i<3; i++){
	yprDev[i] *= 180.0/M_PI;
}
}

void endSwing(){
DEBUG_PRINTLN("END SWING===============================");
FrMotTrig = 0;
ReMotTrig = 0;
LeMotTrig = 0;
RiMotTrig = 0;
triggerMotors();
 
if(frSwiEnable){
	AudioMan.asyncPlayVoice(11); // Beep
}
 
goState = 0;
backSwingCompleted = 0;
firstGo = 1;

// Turn off Go led
digitalWrite(goLedPin, LOW);
goLedState = 1; // sets state so next blink is off
}

bool initializeSD(int16_t CSPin){
bool success = 0;
DEBUG_PRINT("Initializing SD card...");
pinMode(CSPin, OUTPUT); // Chip select pin
if (!SD.begin(CSPin)) {
	DEBUG_PRINTLN("Initialization failed!");
	return success;
}
DEBUG_PRINTLN("Initialization OK!");
success = 1;
return success;
}

void getLimitsFromSD(){
/*Settings on SD card in file Default.txt
Numbers should be positive, each number on a new line, in the following order:
PitchBack
PitchFwd
YawR
YawL
RollBack
RollFwd
FrMotEnable
ReMotEnable
LeMotEnable
RiMotEnable
bkSwiEnable
frSwiEnable
leftHanded
*/

uCard = SD.open("Default.txt");
if (uCard) {
	DEBUG_PRINTLN("From Default.txt:");
	
	// Read the file
	while (uCard.available()) {

		// Read in the 6 limits
		int16_t i = 0;
		while(i < 6) {
		inChar = uCard.read(); // returns byte
		if (isDigit(inChar)) {
			inString += (char)inChar;
		}
		else if (inChar == '\n') {
			angleLimits[i] = inString.toInt();
			DEBUG_PRINTLN(angleLimits[i]);
			inString = ""; 
			i++;
		}
		}

		// Read in the 6 enables + leftHanded
		i = 0;
		while(i < 6) {
		inChar = uCard.read(); // returns byte
		if (isDigit(inChar)) { 
			inString += (char)inChar;
		}
		else if (inChar == '\n') {
			switch (i) {
			case 0:
				FrMotEnable = inString.toInt();
				break;
			case 1:
				ReMotEnable = inString.toInt();
				break;
			case 2:
				LeMotEnable = inString.toInt();
				break;
			case 3:
				RiMotEnable = inString.toInt();
				break;
			case 4:
				bkSwiEnable = inString.toInt();
				break;
			case 5:
				frSwiEnable = inString.toInt();
				break;
			case 6:
				leftHanded = inString.toInt();
				break;
			default:
				// ignore data if not number or new line
				break;
			}
			inString = "";
			i++;
		}
		}
		uCard.close();
	}
}
else {
	DEBUG_PRINTLN("Settings file could not be read! Shutting down!");
	digitalWrite(killPin,HIGH); // kill power
	while(1);
}

// Need to correct the direction of some limits, since not stored
angleLimits[1] = -angleLimits[1];
angleLimits[2] = -angleLimits[2];
}

/* bool newDatalogFile(){
bool success = 0;

inString = "";
DateTime now = rtc.now(); // get date/time
inChar = now.month();
if (inChar <10){
	inString += "0";
}
inString += String(inChar);
inChar = now.day();
if (inChar <10){
	inString += "0";
}
inString += String(inChar);
inChar = now.hour();
if (inChar <10){
	inString += "0";
}
inString += String(inChar);
inChar = now.minute();
if (inChar <10){
	inString += "0";
}
inString += String(inChar);
inString += ".txt";
char filename[13];
inString.toCharArray(filename, 13);
uCard = SD.open(filename, FILE_WRITE);

// Debug check to see if the file exists:
DEBUG_PRINTLN(inString);
DEBUG_PRINTLN(filename);
DEBUG_PRINTLN("Creating datalog");
if (SD.exists(filename)) {
	success = 1;
	DEBUG_PRINT(filename);
	DEBUG_PRINTLN(" exists!");
}
else {
	DEBUG_PRINT(filename);
	DEBUG_PRINTLN(" doesn't exist."); 
}

return success;
} */

int16_t checkLowBatt(){
battVolts = analogRead(battVoltsPin)/1023.0*5000.0; // in mV from 0 to 5000
if (battVolts > 4150){ battPct = 100; }
else if (battVolts > 3760) { battPct = map(battVolts,3760,4150,50,100); digitalWrite(battLedPin, HIGH); }
else if (battVolts > 3650) { battPct = map(battVolts,3650,3760,25,50); digitalWrite(battLedPin, HIGH); }
else { 
	DEBUG_PRINTLN("Battery critically low!");
	AudioMan.playVoice(1); // Audio for low batt
	digitalWrite(killPin,HIGH); // kill power
}
DEBUG_PRINT("Battery percent: ");
DEBUG_PRINTLN(battPct);
return battPct;
}

void triggerMotors(){
DEBUG_PRINT("Fr: ");
DEBUG_PRINT(FrMotTrig);
DEBUG_PRINT(" Re: ");
DEBUG_PRINT(ReMotTrig);
DEBUG_PRINT(" Le: ");
DEBUG_PRINT(LeMotTrig);
DEBUG_PRINT(" Ri: ");
DEBUG_PRINTLN(RiMotTrig);


if(FrMotEnable){
	analogWrite(motorFront, FrMotTrig*scaleFactor);
}
if(ReMotEnable){
	analogWrite(motorRear, ReMotTrig*scaleFactor);
}
if(LeMotEnable){
	analogWrite(motorLeft, LeMotTrig*scaleFactor);
}
if(RiMotEnable){
	analogWrite(motorRight, RiMotTrig*scaleFactor);
}
}

void printStatus(){
DEBUG_PRINT("Dev/lims: P: ");
DEBUG_PRINT(yprDev[1]); // pitch
DEBUG_PRINT(" /+");
DEBUG_PRINT(angleLimits[0]); // bwd lim
DEBUG_PRINT("/");
DEBUG_PRINT(angleLimits[1]); // fwd lim
DEBUG_PRINT("  Y: ");
DEBUG_PRINT(yprDev[0]); // yaw
DEBUG_PRINT(" /+");
DEBUG_PRINT(angleLimits[2]); // right lim
DEBUG_PRINT("/");
DEBUG_PRINT(angleLimits[3]); // left lim
DEBUG_PRINT("  R: ");
DEBUG_PRINT(yprDev[2]); // roll
DEBUG_PRINT(" /+");
DEBUG_PRINT(angleLimits[4]); // back lim
DEBUG_PRINT("/");
DEBUG_PRINT(-angleLimits[5]); // right lim
DEBUG_PRINT("  ");
}

void processGoInterrupt(){
nowGo = millis();
// If interrupts come faster than DEBOUNCE_MAX_MS, assume it's a bounce and ignore
if (nowGo - lastGo > DEBOUNCE_MAX_MS) {
	goState = !goState;
	lastGo = nowGo;
}
}

void processBleInterrupt(){
nowBLE = millis();
if (nowBLE - lastBLE > bleBounceTime) {
	bleState = !bleState;
	lastBLE = nowBLE;
}
}

void bleProgram(){
if(!(mpu.getSleepEnabled())){
	mpu.setSleepEnabled(1);
}

digitalWrite(bleLedPin, HIGH);
digitalWrite(goLedPin, LOW);

BLEMini_begin(57600); // connect to phone

int16_t settingsBuffer[28]; // only using first 14, but larger in case extra data sent
int16_t i, temp;

for(i = 0; i < 28; i++){
	settingsBuffer[i] = 0;
}

i = 0;
while ( BLEMini_available() ) {
	temp = BLEMini_parseInt();
	if(!(temp > -180 && temp < 180) ){
		temp = 360;
	}
	settingsBuffer[i] = temp;
	i++;
}

if(settingsBuffer[13] == 180){ // if the stop int16_t is correct, load the limits.
	for(i = 0; i < 6; i++){
		if(settingsBuffer[i] < 180){
		angleLimits[i] = settingsBuffer[i];
		}
	}
	FrMotEnable = ((settingsBuffer[6] < 360) && (settingsBuffer[6] > 0));
	ReMotEnable = ((settingsBuffer[7] < 360) && (settingsBuffer[7] > 0));
	LeMotEnable = ((settingsBuffer[8] < 360) && (settingsBuffer[8] > 0));
	RiMotEnable = ((settingsBuffer[9] < 360) && (settingsBuffer[9] > 0));
	bkSwiEnable = ((settingsBuffer[10] < 360) && (settingsBuffer[10] > 0));
	frSwiEnable = ((settingsBuffer[11] < 360) && (settingsBuffer[11] > 0));
	leftHanded = ((settingsBuffer[12] < 360) && (settingsBuffer[11] > 0));
}
// else{
//  AudioMan.playVoice(3); // Bluetooth error
//  delay(100)0
// }

if(!bleState){
	BLEMini_end();
	mpu.setSleepEnabled(0);
	digitalWrite(bleLedPin, LOW);
	// return;
}
}