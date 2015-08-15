/*
 * LEDController.h
 *
 *  Created on: 06.06.2015
 *      Author: JochenAlt
 */

#ifndef SRC_LEDCONTROLLER_H_
#define SRC_LEDCONTROLLER_H_

#include <iostream>
#include <string>
#include "setup.h"
#include "Util.h"

class LEDPattern {
	public:
     	friend ostream& operator<<(ostream&, const LEDPattern&);

		enum BrightnessType { OFF=0, DARK=30,  LIGHT=80, FULL=100 };

		LEDPattern();
		LEDPattern(const LEDPattern& pattern );
		void operator=(const LEDPattern& pattern);
		void setFadeInOut(unsigned long endTime, int msIn, int msOut, int dutyOff, int dutyOn, bool repeat);
		void setFadeInOut(unsigned long endTime, int msIn, int msOut, int dutyOff, int dutyOn);

		void setFadeOut(unsigned long startTime, int time, int duty);
		void setFadeIn(unsigned long startTime, int time, int duty);
		void setConstantDuty(unsigned long startTime, int time, int duty);


		void setDark();
		void clear();
		int getCurrentDuty(unsigned long now);

	private:
		unsigned long lampFadeInOutStartTime;
		unsigned long lampFadeInOutMiddleTime;
		unsigned long lampFadeInOutEndTime;
		int lampFadeInOutDutyOn;
		int lampFadeInOutDutyOff;
		bool repeat;

};
class LEDController {
public:
	LEDController();
	void loop();
	void setup();

	static LEDController& getInstance() {
		static LEDController instance;
		return instance;
	}

	void setPattern(const LEDPattern& pattern);
private:
	LEDPattern pattern;
};

#endif /* SRC_LEDCONTROLLER_H_ */
