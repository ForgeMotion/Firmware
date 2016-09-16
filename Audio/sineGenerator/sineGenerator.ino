#define FREQUENCY_HZ 	500
#define V_PEAK_TO_PEAK	2.0
#define DC_OFFSET_V 	1.65

#define TWO_PI			6.28319
#define SCALE_CONST		(TWO_PI*FREQUENCY_HZ)/100000L

int delayTime = 5;
float phaseDelta = delayTime*SCALE_CONST; 

const float dcOffset = DC_OFFSET_V/3.3*4096, amplitude = V_PEAK_TO_PEAK/3.3*4096/2;

float phase = 0.0;
elapsedMicros usec = 0;

void setup() {
  analogWriteResolution(12);
}

void loop() {
  float val = amplitude * sin(phase) + dcOffset;
  analogWrite(A14, (int)val);

  phase += phaseDelta;
  if (phase >= TWO_PI) phase = 0;
  
  while (usec < delayTime) ; // wait
  usec = usec - delayTime;
}

/*
every delayTime phase increases by 0.02
1 phase = 2*pi = 1 cycle
(2*pi/0.02) loops = (100*pi) loops = 1 cycle
1 loop = 500 micros
1 cycle = 2*pi/.02 loops * 500 micros/loop
f = cycles/sec = 1/(500*2*pi/.02) = .02/(500*2*pi)
f = phaseDelta/(delayTime * 2*pi)
delayTime = phaseDelta/(2*pi*f)
*/