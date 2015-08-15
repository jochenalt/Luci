/*
 * ServoControler.cpp
 *
 *  Created on: 24.02.2015
 *      Author: JochenAlt
 */

#include "ServoController.h"
#include "MicroControllerInterface.h"
#include "Util.h"


bool debugServo = true;
bool amendedPulseWidth = true;

void Servo::setup(ServoType pServoType,bool pReverse) {
	servoType = pServoType;
	currentAngle = 0;
	reverse = pReverse;
	Configuration& config = Configuration::getInstance();
	servoMiddleAngle = config.getOrCreateFloat("Servo." + getName() + "-middle", 0.0);
	servoMinAngle    = config.getOrCreateFloat("Servo." + getName() + "-min",-90.0);
	servoMaxAngle    = config.getOrCreateFloat("Servo." + getName() + "-max",90.0);
	usPer45Degree    = config.getOrCreateFloat("Servo." + getName() + "-us-per-45-degree",450);

}

void Servo::writeConfig() {
	Configuration& config = Configuration::getInstance();
	config.putFloat("Servo." + getName() + "-middle", servoMiddleAngle);
    config.putFloat("Servo." + getName() + "-min",servoMinAngle);
    config.putFloat("Servo." + getName() + "-max",servoMaxAngle);
    config.putFloat("Servo." + getName() + "-us-per-45-degree",usPer45Degree);

}

string Servo::getName() {
	switch (servoType) {
		case SERVO_BASE_TURN:
			return "base-turn";
		case SERVO_LEG:
			return "leg";
		case SERVO_ARM:
			return "arm";
		case SERVO_HEAD_NICK:
			return "head-nick";
		case SERVO_HEAD_TURN:
			return "head-turn";
		default:
			return "";
	}
}


int Servo::setUnlimitedAngle(float angleDegree) {

	currentAngle = angleDegree;
	setPointInTime = millis();
	return getPWMus();
}

int Servo::getPWMus() {
	float motorDegree = currentAngle + servoMiddleAngle;
	int motor_us =  mapLong((motorDegree*(reverse?-1000.0:1000.0)),-90L*1000, +90L*1000, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	if (amendedPulseWidth)
		motor_us =  mapLong((motorDegree*(reverse?-1000.0:1000.0)),-90L*1000, +90L*1000, DEFAULT_PULSE_WIDTH-2*usPer45Degree, DEFAULT_PULSE_WIDTH+2*usPer45Degree);

	return motor_us;
}

bool Servo::checkAngle(float &angleDegree) {
	if (angleDegree > servoMaxAngle) {
		cerr << "Servo " << getName() << "(" << angleDegree << ") limited to " << servoMaxAngle << endl;
		angleDegree = servoMaxAngle;
		return false;
	}
	if (angleDegree < servoMinAngle) {
		cerr << "Servo " << getName() << "(" << angleDegree << ") limited to " << servoMinAngle << endl;
		angleDegree = servoMinAngle;
		return false;
	}
	return true;
}


bool Servo::setAngle(float angleDegree) {
	bool ok = checkAngle(angleDegree);
	setUnlimitedAngle(angleDegree);
	return ok;
}

void Servo::print() {
	int motor_us = getPWMus();

	if (debugServo)
		cout << "servo " << getName()
		 << "[" << servoMinAngle << ".." << servoMiddleAngle << ".." << servoMaxAngle << "] := "
		 << getAngle() << "°" << " = " << motor_us << "us" << endl;
}


float Servo::getAngle() {
	return currentAngle;
}

void Servo::setMaxAngle(float pMaxAngle) {
	writeConfig();
}

void Servo::setMinAngle(float pMinAngle) {
	servoMinAngle = pMinAngle;
	writeConfig();
}

float Servo::getMinAngle() {
	return servoMinAngle;
}

float Servo::getMaxAngle() {
	return servoMaxAngle;
}

void Servo::setMiddleAngle(float pMiddleAngle) {
	float diff = pMiddleAngle + servoMiddleAngle;
	servoMiddleAngle = diff;
	servoMinAngle -= diff;
	servoMaxAngle -= diff;
	writeConfig();
}

float Servo::getMiddleAngle() {
	return servoMiddleAngle;
}


ServoController& ServoController::getInstance() {
	static ServoController instance;
	return instance;
}

void ServoController::setServoData() {
	MicroControllerInterface::getInstance().setServoData(
			baseTurnServo.getPWMus(), legServo.getPWMus(),armServo.getPWMus(),
			headNickServo.getPWMus(),headTurnServo.getPWMus());
}

void ServoController::setup() {

	baseTurnServo.setup(SERVO_BASE_TURN, true /* reverse direction */);
	legServo.setup(SERVO_LEG, 			 false);
	armServo.setup(SERVO_ARM, 			 false);
	headNickServo.setup(SERVO_HEAD_NICK, false);
	headTurnServo.setup(SERVO_HEAD_TURN, true/* reverse direction */);

	baseTurnServo.setAngle(0);
	legServo.setAngle(0);
	armServo.setAngle(0);
	headNickServo.setAngle(0);
	headTurnServo.setAngle(0);

	currentServo = SERVO_BASE_TURN;
}


void ServoController::printMenu() {
	cout << "Servo Menu" << endl
 		 << "p    - power " << string(MicroControllerInterface::getInstance().getPowerOn()?"off":"on") << endl
 		 << "1-5 - select servo" << endl
		 << "+/- - position" << endl
		 << "*/_ - position" << endl
		 << "<   - define position as min" << endl
		 << ">   - define poition as max" << endl
		 << ".   - define position as middle position" << endl << endl;

	cout << "current Servo:";
	getServo(currentServo).print();
}


Servo& ServoController::getServo(ServoType servoType) {
	switch (servoType) {
		case SERVO_BASE_TURN:
			return baseTurnServo;
		case SERVO_LEG:
			return legServo;
		case SERVO_ARM:
			return armServo;
		case SERVO_HEAD_NICK:
			return headNickServo;
		case SERVO_HEAD_TURN:
			return headTurnServo;
		default:
			return baseTurnServo;
	}
}

void ServoController::print() {
	baseTurnServo.print();
	legServo.print();
	armServo.print();
	headNickServo.print();
	headTurnServo.print();
}



void ServoController::callMenu() {
	while (true){
		char c = keyPressed();
		Configuration::getInstance().loop(); // check if something has to be written to the config file


		if ((c != 0) && (c != EOF) && (c != '\n')){
			switch (c) {
			case 'p': {
				bool powerOn = MicroControllerInterface::getInstance().getPowerOn();
				cout << "switch servo power " << (powerOn?"off":"on")<< endl;
				setServoData();
				MicroControllerInterface::getInstance().power(!powerOn);
				break;
			}
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
				currentServo = static_cast<ServoType>(c-'1');
				cout << "consider servo " << getServo(currentServo).getName() << endl;
				break;
			case '+':
			case '-': {
				Servo& servo = getServo(currentServo);
				float currentAngle = servo.getAngle();
				currentAngle += ((c=='+')?0.5:-0.5);
				servo.setUnlimitedAngle(currentAngle);
				setServoData();
				MicroControllerInterface::getInstance().sendIfValueHasChanged();
				servo.print();
				}
				break;
			case '*':
			case '_': {
				Servo& servo = getServo(currentServo);
				float currentAngle = servo.getAngle();
				currentAngle += ((c=='*')?2.0:-2.0);
				servo.setUnlimitedAngle(currentAngle);
				setServoData();
				MicroControllerInterface::getInstance().sendIfValueHasChanged();
				servo.print();
				}
				break;

			case '<': {
				Servo& servo = getServo(currentServo);
				servo.setMinAngle(servo.getAngle());
				servo.print();
			}
				break;
			case '>': {
				Servo& servo = getServo(currentServo);
				float currentAngle = servo.getAngle();
				servo.setMaxAngle(currentAngle);
				servo.print();
				}
				break;
			case '.': {
				Servo& servo = getServo(currentServo);
				servo.setMiddleAngle(servo.getAngle());
				servo.currentAngle = 0;
				servo.print();
				}
				break;
			case 'h':
				printMenu();
				break;
			case '#':
			case '\e':
				return;
				break;
			default:
				break;
			}
		}
	}
}
