/*
 * MicroControllerInterface.h
 *
 *  Created on: 02.03.2015
 *      Author: JochenAlt
 */

#ifndef MICROCONTROLLERINTERFACE_H_
#define MICROCONTROLLERINTERFACE_H_

#include "setup.h"

class MicroControllerInterface {
public:
	MicroControllerInterface() {
		lampDuty = 0;
		dataIsPending = false;
		powerOn = false;
		for (int i = 0;i<SERVO_NUMBER;i++)
			servoPWMValue_us[0] = DEFAULT_PULSE_WIDTH;


	}
	static MicroControllerInterface& getInstance() {
		static MicroControllerInterface instance;
		return instance;
	}

	void setup();
	bool getPowerOn() { return powerOn; };
	void setServoData(int servo1_us, int servo2_us, int servo3_us,int servo4_us,int servo5_us);
	void setLampDuty(int pLampDuty);

	bool isServoDataPending() {
		return dataIsPending;
	}

	void sendIfValueHasChanged() {
		if (dataIsPending)
			send();
	}
	void setLampFadeOut(int fadeOutTime_ms);

	void send();
	void power(bool onOff);

	bool dataIsPending;
	int servoPWMValue_us[SERVO_NUMBER];
	int lampDuty;
	bool powerOn;

};

#endif /* MICROCONTROLLERINTERFACE_H_ */
