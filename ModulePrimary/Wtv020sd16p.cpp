/*
 Wtv020sd16p.cpp - Library to control a WTV020-SD-16P module to play voices from an Arduino board.
 Created by Diego J. Arevalo, August 6th, 2012.
 Released into the public domain.
 
Modifications by Tyler McGahee 8/20/2014
 This is a modified version. Changes as follows:
 -Reduce sendCommand time from 55-57ms to less than 10ms
   -Down to <2.7ms, 8/20/2014
   -Have to add delay(100) to end of unmute to keep it working (may be able to reduce this time)
 -Change playVoice so it waits for low then high. Makes it more reliable
 -Reduced delay() in playVoice from 30 to 10, still stable
 */

#include "Arduino.h"
#include "Wtv020sd16p.h"

const unsigned int PLAY_PAUSE = 0xFFFE;
const unsigned int STOP = 0xFFFF;
const unsigned int VOLUME_MIN = 0xFFF0;
const unsigned int VOLUME_MAX = 0xFFF7;


Wtv020sd16p::Wtv020sd16p(int resetPin,int clockPin,int dataPin,int busyPin)
{
  _resetPin=resetPin;
  _clockPin=clockPin;
  _dataPin=dataPin;
  _busyPin=busyPin;
  _busyPinState=HIGH;

  pinMode(_resetPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  pinMode(_dataPin, OUTPUT);
  pinMode(_busyPin, INPUT);
}


void Wtv020sd16p::reset(){
  digitalWrite(_clockPin, LOW);
  digitalWrite(_resetPin, HIGH);

  //Reset pulse.
  digitalWrite(_resetPin, LOW);
  delay(5);
  digitalWrite(_resetPin, HIGH);

  //Reset idle to start bit.
  digitalWrite(_clockPin, HIGH);
  delay(2000); //Could probably be a lot smaller if needed --orig300--------------------
}


void Wtv020sd16p::playVoice(int voiceNumber){
  
  sendCommand(voiceNumber); 
  _busyPinState=digitalRead(_busyPin);

  while(_busyPinState==LOW){
    _busyPinState=digitalRead(_busyPin);
  }
  while(_busyPinState==HIGH){
    _busyPinState=digitalRead(_busyPin);
  }

  delay(10); //was 30ms, 5 not stable, 10 stable, 7 NOT, 8 NOT
}


void Wtv020sd16p::asyncPlayVoice(int voiceNumber){
  sendCommand(voiceNumber);
}


void Wtv020sd16p::stopVoice(){
  sendCommand(STOP);
}


void Wtv020sd16p::pauseVoice(){
  sendCommand(PLAY_PAUSE);
}


void Wtv020sd16p::mute(){
  sendCommand(VOLUME_MIN);
}


void Wtv020sd16p::unmute(){
  sendCommand(VOLUME_MAX);
  delay(100);
}


void Wtv020sd16p::sendCommand(unsigned int command) {

  //Start bit Low level pulse.
  digitalWrite(_clockPin, LOW);
  delayMicroseconds(1700); //WAS delay(2); 1200 micros NOT working, 1500 NOT, 1800 OK, 1600 NOT, 1700 OK

  for (unsigned int mask = 0x8000; mask > 0; mask >>= 1) {

    //Clock low level pulse.
    digitalWrite(_clockPin, LOW);
    delayMicroseconds(10); //WAS 50 micros, 10 stable

    //Write data setup.
    digitalWrite(_dataPin, command & mask); //more efficient than next 6 lines, saves 2-3 micros
//    if (command & mask) {
//      digitalWrite(_dataPin, HIGH);
//    }
//    else {
//      digitalWrite(_dataPin, LOW);
//    }

    //Write data hold.
    delayMicroseconds(10); //WAS 50 micros, 30 stable, 10 stable, 2764

    //Clock high level pulse.
    digitalWrite(_clockPin, HIGH);
    delayMicroseconds(10); //WAS 100 micros, 70 stable, 30 stable, 10 stable

    if (mask>0x0001){
      //Stop bit high level pulse.
      delayMicroseconds(10); //WAS delay(2);, 10 micros stable
    }
  }

  //Busy active high from last data bit latch.
//  delay(20); //Removed!
}
