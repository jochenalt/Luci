//============================================================================
// Name        : LucyController.cpp
// Author      : Jochen Alt
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <iomanip>
#include <iostream>
#include "LuciController.h"
#include "ServoController.h"
#include "Kinematic.h"
#include "Configuration.h"
#include "Kinematic.h"
#include "TrajectoryController.h"
#include "MicroControllerInterface.h"
#include "TrajectoryPattern.h"
#include "TrajectoryPatternController.h"
#include "LEDController.h"
#include "Util.h"
#include "Sight.h"

using namespace std;

bool production = false;

void setupLuci(bool productiveOn) {
	cout << "I'm cute Luci." << endl;
	Configuration::getInstance().loadConfigFile();
	ServoController::getInstance().setup();
	Kinematics::getInstance().setup();
	TrajectoryController::getInstance().setup();
	MicroControllerInterface::getInstance().setup();
	TrajectoryPatternController::getInstance().setup();
	LEDController::getInstance().setup();

	// setup is done, now switch off everything properly
	// set null postion, switch lamp off, switch off all servos
	Kinematics::getInstance().moveServosTo(Kinematics::getInstance().getNullPosition());
	MicroControllerInterface::getInstance().setLampDuty(0);
	MicroControllerInterface::getInstance().power(false);

	std::setprecision(1);
	prepareReadKey();
	production = productiveOn;
}

void printMenu() {
	cout << "Lucy " << millis() << "ms" << endl
		<< "K   - kinematic controller" << endl
		<< "S   - servo controller" << endl
		<< "T   - trajectory controller" << endl
		<< "P   - trajectory pattern controller" << endl
		<< "d   - debug on/off" << endl
		<< "m   - fast math on/off" << endl
		<< "p   - production mode on/off" << endl

		<< "ESC  - quit" << endl;
}


bool loopLuci() {
	Configuration::getInstance().loop(); // check if something has to be written to the config file
	LEDController::getInstance().loop(); // update LED

	if (production) {
		TrajectoryPatternController::getInstance().loop();
	}

	loopSight(); 		// check if background thread has found a face

	char c = keyPressed();

	if ((c != 0) && (c != EOF) && (c != '\n')){
		production = false;
		switch (c) {
			case 'h':
				printMenu();
				break;
			case 'S':
				ServoController::getInstance().printMenu();
				ServoController::getInstance().callMenu();
				break;
			case 'K':
				Kinematics::getInstance().printMenu();
				Kinematics::getInstance().callMenu();
				break;
			case 'T':
				TrajectoryController::getInstance().printMenu();
				TrajectoryController::getInstance().callMenu();
				break;
			case 'P':
				TrajectoryPatternController::getInstance().printMenu();
				TrajectoryPatternController::getInstance().callMenu();
				break;

			case 'd':
				debug = !debug;
				cout << "debug is " << (debug?"on":"off") << endl;
				break;
			case 'm':
				useFastMath = !useFastMath;
				cout << "fast-math is " << (useFastMath?"on":"off") << endl;
				break;
			case 'p':
				production = !production;
				cout << "production is " << (production?"on":"off") << endl;
				break;

			case '#':
			case '\e':
				return false;
				break;
			default:
				break;
		}
	}

	return true;
};

void tearDownLuci() {
	shutdownKey();
}


void setFaceDetection(const TimedPosition& face) {
	TrajectoryController::getInstance().setDetectedFace(face);
	// LEDController::getInstance().setLampFadeOutLight();
}

void setNoFaceDetection() {
	// LEDController::getInstance().setLampFadeOutSlowDark();
}

