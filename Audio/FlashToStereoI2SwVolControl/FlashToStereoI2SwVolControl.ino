/* Teensy Audio Library volume control:
Teensy AudioUse playSdWav or AudioPlaySdMp3 to get two channel audio (Channel 0 is left, 1 is right).
Each channel goes into a mixer (which has up to 4 input channels, 1 output channel).
Set the gain of each mixer with AudioMixer4.gain(channel, gain); e.g. mix1.gain(0, 0.4);

Gain is +/-32767.0f, where 1.0 is 0dB gain
Algorithm:
	multiplier[channel] = gain * 65536.0f; // gain * 2^16 makes it a int32_t, same as <<16
	mixer.cpp applyGain(*data, multiplier[channel])
		grabs the data (two 16 bit samples stored in a uint32_t)
		calls these clever assembly things to unpack and multiply in one step:
			signed_multiply_32x16b(uint32_t a, uint32_t b)
				int32_t out;
				asm volatile("smulwb %0, %1, %2" : "=r" (out) : "r" (a), "r" (b));
				
				(smulwb out, a, b) is the same as:
				b = b & 0xff; // use only bottom bits, that's the "b" in "smulwb" and "signed_multiply_32x16b"
				int_48 temp;
				temp = a*b;
				out = temp >> 16;
			signed_multiply_32x16t(uint32_t a, uint32_t b)
				int32_t out;
				asm volatile("smulwt %0, %1, %2" : "=r" (out) : "r" (a), "r" (b));
				
				(smulwt out, a, b) is the same as:
				b = b >> 16; // use only top bits, that's the "t" in "smulwt" and "signed_multiply_32x16t"
				int_48 temp;
				temp = a*b;
				out = temp >> 16;
		Then a weird kind of bit shift: 
			val1 = signed_saturate_rshift(val1, 16, 0);
			signed_saturate_rshift(int32_t val, int bits, int rshift)
				int32_t out;
				asm volatile("ssat %0, %1, %2, asr %3" : "=r" (out) : "I" (bits), "r" (val), "I" (rshift));

				argument 3 specifies a shift. asr "I" (rshift) is an optional "Arithmetic Shift Right".
				 	
				ssat does the shift (here, rshift=0 and no shift happens.) then limits the value to –2^(bits–1) ≤ x ≤ 2^(bits–1) –1

				so overall it's the same as:
					if(out > 2^(bits-1)-1) out = 2^(bits-1)-1;
					else if(out < -2^(bits-1)) out = -2^(bits-1);
				

		so, in total:
			int16_t data = sample;
			int32_t mult = gain * 2^16;
			int32_t scaledData = (mult*data) / 2^16;

			if(scaledData > 2^15-1) scaledData = 2^15-1;
			else if(scaledData < -2^15) scaledData = -2^15;
			
			int16_t output = (int16_t)scaledData;

		trying to condense it:
			int16_t output = constrain(gain * sample, -2^15, 2^15-1);
			therefore, audio will start saturating if gain > (2^15-1)/sample_max

	More importantly, it's a linear voltage control. to get linear power control, gain = sqrt(desiredGain) where desiredGain is linear
*/
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=168.09091186523438,242.09091186523438
AudioMixer4              mixer1;         //xy=339.0909118652344,212.09091186523438
AudioMixer4              mixer2;         //xy=340.0909118652344,274.0909118652344
AudioOutputI2S           i2s1;           //xy=502.0909423828125,241.09091186523438

AudioConnection          patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection          patchCord2(playSdWav1, 1, mixer2, 0);
AudioConnection          patchCord3(mixer1, 0, i2s1, 0);
AudioConnection          patchCord4(mixer2, 0, i2s1, 1);
// GUItool: end automatically generated code
