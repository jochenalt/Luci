/*
 * ServoControler.h
 *
 *  Created on: 24.02.2015
 *      Author: JochenAlt
 */

#ifndef SERVOCONTROLLER_H_
#define SERVOCONTROLLER_H_

#include "setup.h"
#include <iostream>
#include <string>
#include "Configuration.h"

using namespace std;

enum ServoType { SERVO_BASE_TURN = 0, SERVO_LEG=1, SERVO_ARM=2, SERVO_HEAD_NICK=3, SERVO_HEAD_TURN=4 };

// Hitec HS-77BB 400us per 45°
// Hitec HS-7954HS 400us per 45°
// Hitec HS-8775MB 400us per 45°
// Hitec HS-5070MH 400us per 45°

class Servo {
	friend class ServoController;
public:
	Servo() {
		servoType = SERVO_BASE_TURN;
		currentAngle = 0;
		servoMiddleAngle = 0;
		servoMinAngle = 0;
		servoMaxAngle = 0;
		setPointInTime = 0;
		usPer45Degree = 450;
		reverse = false;
	};
	void setup(ServoType pServoType, bool reverse);
	void writeConfig();
	string getName();
	bool setAngle(float angleDegree);
	bool checkAngle(float& angleDegree);

	int setUnlimitedAngle(float angleDegree);

	float getAngle();
	int getPWMus();
	void setMaxAngle(float pMaxAngle);
	void setMinAngle(float pMinAngle);
	void setMiddleAngle(float pMiddleAngle);

	float getMinAngle();
	float getMaxAngle();
	float getMiddleAngle();
	void print();

private:

	ServoType servoType;
	float currentAngle;

	float servoMiddleAngle;
	float servoMinAngle;
	float servoMaxAngle;
	float setPointInTime;
	bool reverse;
	int usPer45Degree;
};


class ServoController {
	friend class Servo;
public:

	ServoController() {
		currentServo = SERVO_BASE_TURN;
	};
	virtual ~ServoController() {};

	static ServoController& getInstance();
	void setup();

	void printMenu();
	void callMenu();
	Servo& getServo(ServoType servoType);
	void print();
	void setServoData();

	Servo baseTurnServo;
	Servo legServo;
	Servo armServo;
	Servo headNickServo;
	Servo headTurnServo;

	ServoType currentServo;
};


#endif /* SERVOCONTROLLER_H_ */
