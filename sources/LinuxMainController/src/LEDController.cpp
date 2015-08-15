/*
 * LEDController.cpp
 *
 *  Created on: 06.06.2015
 *      Author: JochenAlt
 */
#include <iostream>
#include "LEDController.h"
#include "MicroControllerInterface.h"
#include "Util.h"
#include <string>

bool debugLED = false;
ostream& operator<<(ostream& os, const LEDPattern& p)
{
	cout 	<< "[" << p.lampFadeInOutStartTime << ":" << p.lampFadeInOutDutyOff
			<< ".." << p.lampFadeInOutMiddleTime << ":" << p.lampFadeInOutDutyOn << ".."
			<< p.lampFadeInOutEndTime << ":" << p.lampFadeInOutDutyOff << "]";
	if (p.repeat)
		cout << "(R)";
    return os;
}

LEDPattern::LEDPattern() {
	clear();
};
LEDPattern::LEDPattern(const LEDPattern& pattern ){
	lampFadeInOutDutyOn 	= pattern.lampFadeInOutDutyOn;
	lampFadeInOutDutyOff 	= pattern.lampFadeInOutDutyOff;

	lampFadeInOutStartTime 	= pattern.lampFadeInOutStartTime;
	lampFadeInOutMiddleTime = pattern.lampFadeInOutMiddleTime;
	lampFadeInOutEndTime 	= pattern.lampFadeInOutEndTime;
	repeat 	                = pattern.repeat;
};

void LEDPattern::operator=(const LEDPattern& pattern ){
	lampFadeInOutDutyOn 	= pattern.lampFadeInOutDutyOn;
	lampFadeInOutDutyOff 	= pattern.lampFadeInOutDutyOff;

	lampFadeInOutStartTime 	= pattern.lampFadeInOutStartTime;
	lampFadeInOutMiddleTime = pattern.lampFadeInOutMiddleTime;
	lampFadeInOutEndTime 	= pattern.lampFadeInOutEndTime;
	repeat 	                = pattern.repeat;

};


void LEDPattern::clear() {
	lampFadeInOutStartTime = 0;
	lampFadeInOutMiddleTime = 0;
	lampFadeInOutEndTime = 0;
	lampFadeInOutDutyOff = 0;
	lampFadeInOutDutyOn = 0;
	repeat = false;

}

void LEDPattern::setFadeInOut(unsigned long endTime, int msIn, int msOut, int dutyOff, int dutyOn, bool pRepeat) {
	if (((dutyOn >= 0) || (dutyOff >= 0))  && ((msIn >= 0) || (msOut >= 0))) {
		lampFadeInOutStartTime 	= endTime-msIn-msOut;
		lampFadeInOutMiddleTime = endTime -msOut;
		lampFadeInOutEndTime 	= endTime;

		lampFadeInOutDutyOn = dutyOn;
		lampFadeInOutDutyOff = dutyOff;
		repeat = pRepeat;
	}
}

void LEDPattern::setFadeInOut(unsigned long endTime, int msIn, int msOut, int dutyOff, int dutyOn) {
	setFadeInOut(endTime,msIn, msOut, dutyOff, dutyOn,false);
}

void LEDPattern::setFadeOut(unsigned long endTime, int time, int duty) {
	setFadeInOut(endTime,0,time,0, duty);
}
void LEDPattern::setFadeIn(unsigned long endTime, int time, int duty) {
	setFadeInOut(endTime, time,0, 0,duty);
}

void LEDPattern::setConstantDuty(unsigned long endTime, int time, int duty) {
	setFadeInOut(endTime, time,0, duty,duty);
}

int LEDPattern::getCurrentDuty(unsigned long now) {
	int duty = 0;
	if (lampFadeInOutStartTime > 0) {
		if (now < lampFadeInOutMiddleTime) {
			int duration = lampFadeInOutMiddleTime-lampFadeInOutStartTime;
			int elapsedTime = now-lampFadeInOutStartTime;
			duty = lampFadeInOutDutyOff + ((lampFadeInOutDutyOn-lampFadeInOutDutyOff)*elapsedTime)/duration;
			duty = constrain(duty,0,100);
		}
		else {
			if (now < lampFadeInOutEndTime) {
				int duration = lampFadeInOutEndTime-lampFadeInOutMiddleTime;
				int elapsedTime = now-lampFadeInOutMiddleTime;

				duty = lampFadeInOutDutyOff + ((lampFadeInOutDutyOn-lampFadeInOutDutyOff)*elapsedTime)/duration;
				duty = constrain(duty,0,100);
			}
		}

		if (now > lampFadeInOutEndTime) {
			if (repeat) {
				int diff = now - lampFadeInOutStartTime;
				lampFadeInOutStartTime += diff;
				lampFadeInOutMiddleTime += diff;
				lampFadeInOutEndTime += diff;
			}
			clear(); // pattern done
		}
	}
	return duty;
}

LEDController::LEDController()
{
	pattern.clear();
}

void LEDController::setup(){
	pattern.clear();
}

void LEDController::loop() {
	unsigned long now = millis();
	int duty = 0;
	duty = pattern.getCurrentDuty(now);
	duty = duty*duty/100; // adapt linear PWM to human perception of brightness (parabolic curve)
	if ((duty > 0) && (debugLED))
		cout << "lamp " << pattern << " (" << now << ")=" << duty << endl;

	MicroControllerInterface::getInstance().setLampDuty((duty*255)/100);
}

void LEDController::setPattern(const LEDPattern& pPattern) {
	if (debugLED)
		cout << "set LED pattern" << pPattern << endl;
	pattern = pPattern;
}

