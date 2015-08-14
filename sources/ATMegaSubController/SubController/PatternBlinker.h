/*
 * PatternBlinker.h
 *
 * Created: 21.11.2014 22:44:41
 *  Author: JochenAlt
 */ 


#ifndef PATTERNBLINKER_H_
#define PATTERNBLINKER_H_

#include "TimePassedBy.h"

// Class that implements a pretty pattern blinker without use of delay().
// Has to run in a loop.
// use:
//      // define blink pattern, 1=on, 0=off
//		static uint8_t BotIsBalancing[3] = { 0b11001000,0b00001100,0b10000000}; // nice pattern. Each digit takes 50ms
//																				// define blink pattern array as long
//																				// as you like
//		PatternBlinker blinker;													// initiate pattern blinker
//		blinker.set(LED_PIN_BLUE,BotIsBalancing,sizeof(BotIsBalancing));		// assign pattern
//		while (true) {
//			blinker.loop();														// switch on off LED when necessary
//			<do something else>
//		}
//
class PatternBlinker {
	public:

	PatternBlinker() {
		setup(0,0);
	}

	PatternBlinker(uint8_t pPin, uint8_t ms) {
		setup(pPin,ms);
	}

	 void setup(uint8_t pPin, uint8_t ms) {
		mPin = pPin;
		mDuration = ms;
		mPattern = NULL;
		pinMode(pPin, OUTPUT);
		digitalWrite(pPin, LOW);
	}
	
	// switch blinker off
	void off() {
		mPattern = NULL;
		if (mPin != -1)
		digitalWrite(mPin,HIGH);
	}
	
	// set the blink pattern on the passed pin
	void set(uint8_t* pPattern, uint8_t pPatternLength) {
		mPattern = pPattern;
		mPatternLen = pPatternLength;
		mSeq = 0;
		oneTime = false;
	}
	void setOneTime(uint8_t* pPattern, uint8_t pPatternLength) {
		set(pPattern,pPatternLength);
		oneTime = true;
	}

	void loop() {
		uint16_t passed_ms;
		if ((timer.isDue_ms(mDuration,passed_ms))) {
			if (mPattern != NULL) {
				uint8_t pos,bitpos;
				pos = mSeq / 8;
				bitpos = 7- (mSeq & 0x07);
				if ((mPattern[pos] & _BV(bitpos)) > 0) {
					digitalWrite(mPin,LOW);
				}
				else {
					digitalWrite(mPin,HIGH);
				}

				mSeq++;
				if ((mSeq >= (mPatternLen)*8))
					mSeq = 0;
					if (oneTime)
						mPattern = NULL;
			} else {
				digitalWrite(mPin,HIGH);
			}
		}
	}
	
	
	uint8_t mSeq;			// current position within the blink pattern
	int8_t mPin;			// pin to be used
	uint8_t* mPattern;		// blink pattern, passed in set()
	uint8_t  mPatternLen;	// length of the pattern  = sizeof(*mPattern)
	TimePassedBy timer;		// timer for checking passed time
	boolean oneTime;
	uint8_t mDuration;
};





#endif /* PATTERNBLINKER_H_ */