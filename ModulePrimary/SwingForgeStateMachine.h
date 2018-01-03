#ifndef _SWINGFORGE_STATE_MACHINE_
#define _SWINGFORGE_STATE_MACHINE_


// ==================================== State machine structure ====================================
// Enumerate the states of the state machine
enum state_t{
  ST_IDLE,
  ST_BACKSWING,
  ST_FRONTSWING,
  ST_BLE_PROGRAMMING,
  NUM_STATES
};

// declare and initialize state to idle
state_t currentState = ST_IDLE;

// Set up state functions - each must return the new currentState
typedef state_t state_func_t();


state_t runIdleState();
state_t runBackswingState();
state_t runFrontswingState();
state_t runBLEState();

void goBackswingToFrontswing();
void goIdleToBackswing();
void goSwingToIdle();
void goToBLE();
void goBLEToIdle();

// Store state functions in lookup table
state_func_t* const stateTable[NUM_STATES] = {runIdleState, runBackswingState, runFrontswingState, runBLEState};


// Set up transition functions
typedef void transition_func_t();

// Store transition functions in lookup table (can be NULL even for valid transitions, can be duplicates)
transition_func_t * const transitionTable[NUM_STATES][NUM_STATES] = {
// Current State: 
// Idle					Backswing				Frontswing		BLE				// New State (down)
  {NULL,				goSwingToIdle,			goSwingToIdle,	goBLEToIdle	},	// Idle
  {goIdleToBackswing, 	NULL,					NULL,			NULL		},	// Backswing
  {NULL,				goBackswingToFrontswing,NULL,			NULL		},	// Frontswing
  {goToBLE,				goToBLE,				goToBLE,		NULL		},	// BLE
};


// Function to run current state's function and transition if needed
state_t runState(state_t curState) {
  state_t newState = stateTable[curState]();

  transition_func_t *transition = transitionTable[newState][curState];
  if(transition) transition();

	return newState;
}
#endif
