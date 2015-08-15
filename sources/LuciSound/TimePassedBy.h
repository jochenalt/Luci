/*
 * TimePassedBy.h
 *
 *  Created on: 26.02.2015
 *      Author: JochenAlt
 */

#ifndef TIMEPASSEDBY_H_
#define TIMEPASSEDBY_H_

#include "util.h"



// small helper class to measure the passed time since an event and to check
// whether something that is supposed to run at a specific time (due_ms/due_us)
// use:
//     TimePassedBy timer(MS);						// initialize timer that is supposed to execute something periodically
//     int16_t passed_ms;
//     while (true) {								// to be used in a loop
//			<do what you like>
//			if (timer.due_ms(200, passed_ms))		// check whether 200ms have been passed and
//				<do something that has to be called every 200ms>
//		}
//
class TimePassedBy {
	public:
	// initialize this timer to work
	TimePassedBy () {
		mLastCall_ms = millis();
		now = mLastCall_ms;
	}
	// true, if at least <ms> milliseconds passed since last invocation that returned true.
	// returns the actual passed time in addition
	bool isDue_ms(int ms, int &passed_ms) {
		unsigned long now = millis();
		passed_ms = now-mLastCall_ms;
		if (passed_ms>=ms) {
			mLastCall_ms = now;
			return true;
		}
		return false;
	}
	void setDueTime (int due_ms) {
		mLastCall_ms = millis()-due_ms;
	}

	bool isDue_ms(int  ms) {
		int passed_ms;
		return isDue_ms(ms, passed_ms);
	}

	unsigned long mLastCall_ms;	// last due time in milliseconds
	unsigned long now;		// current time
};


#endif /* TIMEPASSEDBY_H_ */
