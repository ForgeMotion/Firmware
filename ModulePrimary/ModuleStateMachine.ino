// New state machine pseudo code



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

state_t doStIdle();
state_t doStBswing();
state_t doStFswing();
state_t doStBLE();

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

void goIdleToBLE();
void goIdleToBackswing();
void goBackswingToIdle();
void goBackswingToBLE();
void goBackswingToFrontswing();
void goFrontswingToIdle();
void goFrontswingToBLE();
void goBLEToIdle();

// Store transition functions in lookup table
transition_func_t * const transitionTable[NUM_STATES][NUM_STATES] = {
// Current State:	
// 	Idle 				Backswing 				Frontswing 			BLE				// New State (down)
	{NULL,				goBackswingToIdle,		goFrontswingToIdle,	goBLEToIdle}, 	// Idle
	{goIdleToBackswing,	NULL,					NULL,				NULL},			// Backswing
	{NULL,				goBackswingToFrontswing,NULL,				NULL},			// Frontswing
	{goIdleToBLE, 		goBackswingToBLE,		goFrontswingToBLE,	NULL},			// BLE
};



void setup(){
	// lots of stuff here



	currentState = ST_IDLE;
}



void loop(){
	currentState = runState(currentState);
}