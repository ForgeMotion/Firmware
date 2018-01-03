/* Goal: Write C/Cpp code to parse the message from BLE.
Now, presumably we're going to modify the BLE code so it doesn't send over so much bullshit.

Here's what BLE sent out after the iPhone sent the settings over:
WriteHandler 
Length: 
Data: ÿSPPP<(~
WriteHandler 
Length: 
Data: ÿSPPP<(~

ÿSPPP<(~
255 083 080 080 080 060 040 126

These are the values of:
0xff, fwd*10, bwd*10, right*10, left*10, backswing, end swing, bools for limits on/off and handedness	
decimal 126 = 0b1111110


what does the receiving array look like?
	float angleLimits[] = {6.0,-6.0,-7.0,7.0,25.0,25.0};	// [PitchFwd,PitchBack,YawL,YawR,RollBack,RollFwd]
	uint8_t feedbackEnable = 0b00111111;					// bitmask for feedback enable, indices match limits above

*/

// serialEvent occurs whenever a new data comes in the hardware serial RX and at the end of each loop.
void serialEvent() {
	// Wait for all data to arrive
	Serial.flush();
	
	// Delete data until we get to a header byte
	while(Serial.available()){
		if(Serial.peek() != 255) {
			Serial.read();
		} else {
			// Remove header byte
			Serial.read();

			// Make sure there's enough data
			if(Serial.available() >= 7){
				// Grab the full message. If we find another message header, start over.
				uint8_t temp[7];
				for (int i = 0; i < 7; i++)	{
					if(Serial.peek() == 255) 
						serialEvent();
						return;
					else
						temp[i] = Serial.read();
				}

				// Check that the values are valid
				for (int i = 0; i < 6; i++){
					if(!(temp[6] & (1<<i)) || temp[i] > 200){
						// a value is not valid, start over
						serialEvent();
						return;					
					}
				}
				
				// Message is valid. Clear the buffer
				while(Serial.available()) Serial.read();

				// Write the data into settings
				for (int i = 0; i < 4; i++)
					angleLimits[i] = float(temp[i])*0.1;
				angleLimits[5] = float(temp[4]);
				angleLimits[6] = float(temp[5]);
				feedbackEnable = temp[6];
			
				// make sure signs are correct =========================================================

			} else { // If not enough data for a message, clear the buffer
				while(Serial.available()) Serial.read();
			}
		}
	}
	return;
}
