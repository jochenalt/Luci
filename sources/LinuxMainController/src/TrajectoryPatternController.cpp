/*
 * TrajectoryPatternController.cpp
 *
 *  Created on: 06.06.2015
 *      Author: JochenAlt
 */

#include "TrajectoryPatternController.h"

#include "TrajectoryPattern.h"
#include "TrajectoryController.h"

#include "MicroControllerInterface.h"
#include "ServoController.h"
#include "LEDController.h"

#include "Kinematic.h"
#include "math.h"
#include "Util.h"
#include "Sight.h"


bool changePattern = false;


TimedPosition TrajectoryPatternController::fetchNextPosition(TimedPosition lastPosition) {
		if (currentPattern == NULL)
			return TimedPosition();
		else {
			unsigned long now = millis();
			if (startPatternWithNextCall) {
				patternStartTime = now;
				lastPosition.atTime = patternStartTime;
				startPatternWithNextCall = false;
			}
			if (lastPosition.atTime< now)
				lastPosition.atTime = now;

			if (debugTrajectory) {
				cout << currentPattern->getPatternName()
					 << ".fetchNextPosition("
					 << currentPattern->getPhaseName() << ","
					 << currentPattern->getSubPhase() << "," << lastPosition << ") ";
			}
			TimedPosition nextPos = currentPattern->fetchNextPosition(lastPosition);
			if (!nextPos.isNull()) {
				currentPattern->lastPosition = currentPattern->currentPosition;
				currentPattern->currentPosition = nextPos;

			if (debugTrajectory) {
				nextPos.println(" next");
			}
			}
			return nextPos;
		}
	}
TrajectoryPatternType TrajectoryPatternController::getType() {
	if (currentPattern == NULL)
		return NO_PATTERN;
	else
		return currentPattern->getType();
}


void TrajectoryPatternController::loop() {
	TrajectoryController::getInstance().loop();

	// if kinematics does not have at least 3 points (good for bezier curves), fetch a new one
	if (TrajectoryController::getInstance().getTrajectorySize()< 4) {
		unsigned long now = millis();
		TimedPosition setNextPoint = fetchNextPosition(TrajectoryController::getInstance().getLastTrajectoryEntry());
		if (!setNextPoint.isNull()) {
			TrajectoryController::getInstance().addPoint(now,setNextPoint);
		} else {
			setPattern(NO_PATTERN);
			// if next Point is null, there is no active pattern. Hopefully in the next loop there will be one
		}
	}

	changePatternAsAppropriate();
}

void TrajectoryPatternController::setPattern(TrajectoryPatternType pNewPattern) {
	if (getType() != pNewPattern) {
		switch(pNewPattern) {
			case SO_BORED:
				currentPattern = &soBoredPattern;
				break;
			case FACE_INTERACTION:
				currentPattern = &faceInteractionPattern;
				break;
			case NO_PATTERN:
				currentPattern = NULL;
				break;
			case DEPRESSION_PATTERN:
				currentPattern = &depressionPattern;
				break;
			case WAKEUP_PATTERN:
				currentPattern = &wakeupPattern;
				break;
			case NAUGHTYLAMP_PATTERN:
				currentPattern = &naughtyLampPattern;
				break;

			default:
				cerr << "unknown pattern in setPattern" << endl;
				currentPattern = NULL;
			break;
		}

		if (currentPattern != NULL) {
			currentPattern->init(); 			// start with first phase
			startPatternWithNextCall = true; 	// as soon as this pattern is asked the first time for a point, start time is initialized
		}
	}
}


void TrajectoryPatternController::printMenu() {
	cout << "Trajectory Pattern " << endl
	 		<< "p     - power " << string(MicroControllerInterface::getInstance().getPowerOn()?"off":"on") << endl
			<< "1     - test no.1" << endl
			<< "2     - test no.2" << endl
			<< "3     - so bored pattern" << endl
			<< "4     - depression pattern" << endl
			<< "5     - face interaction " << endl
			<< "6     - wake up pattern" << endl
			<< "7     - watch naughty stuff" << endl

			<< "c     - switch change pattern " << (changePattern?"off":"on") << endl;
}

void TrajectoryPatternController::callMenu() {
	unsigned long powerOffTime = 0;
	while (true)  {

		TrajectoryPatternController::getInstance().loop();	// new point for trajectory to be fetched?
		Configuration::getInstance().loop(); 				// check if something has to be written to the config file
		loopSight(); 										// Any Face around?


		if ((powerOffTime > 0) && (millis()>powerOffTime)) {
			cout << "switch servo power off" << endl;
			ServoController::getInstance().setServoData();
			MicroControllerInterface::getInstance().power(false);
			powerOffTime = 0;
		}

		char c = keyPressed();
		switch (c) {
			case 'p': {
				bool powerOn = MicroControllerInterface::getInstance().getPowerOn();
				cout << "switch servo power " << (powerOn?"off":"on")<< endl;
				unsigned long now = millis();
				if (powerOn) {
					TrajectoryPatternController::getInstance().setPattern(NO_PATTERN);
					TimedPosition nullPosition(now+2000,Kinematics::getNullPosition());
					TrajectoryController::getInstance().addPoint(now, nullPosition, now);
					powerOffTime = millis() + 2500;
				} else {
					ServoController::getInstance().setServoData();
					MicroControllerInterface::getInstance().power(!powerOn); // takes 3 seconds (asynchronously) to power up
					TimedPosition sleepPosition(now+3000,Kinematics::getSleepPosition());
					TrajectoryController::getInstance().addPoint(now, sleepPosition);
				}
				break;
			}
			case '1':
				test1();
				break;
			case '2':
				test2();
				break;
			case '3': {
				changePattern = false;
				TrajectoryPatternController::getInstance().setPattern(SO_BORED);
				break;
			}
			case '4': {
				changePattern = false;
				TrajectoryPatternController::getInstance().setPattern(DEPRESSION_PATTERN);
				break;
			}
			case '5': {
				changePattern = false;

				TimedPosition face (millis(), Point(-1000,390,0), Rotation(0,0,0));
				TrajectoryController::getInstance().setDetectedFace(face);
				TrajectoryPatternController::getInstance().setPattern(FACE_INTERACTION);
				break;
			}
			case '6': {
				changePattern = false;
				TrajectoryPatternController::getInstance().setPattern(WAKEUP_PATTERN);
				break;
			}
			case '7': {
				changePattern = false;
				TrajectoryPatternController::getInstance().setPattern(NAUGHTYLAMP_PATTERN);
				break;
			}

			case 'c': {
				changePattern = !changePattern;
				cout << "change pattern = " << (changePattern?"on":"off") << endl;
				break;
			}
#ifdef INCLUDE_LUCI_SIGHT
			case '+': {
				Sight::getInstance().setCamLatency(Sight::getInstance().getCamLatency() + 5);
				cout << "CAM Latency compensation is now " <<  Sight::getInstance().getCamLatency() << endl;;
				break;
			}
			case '-': {
				Sight::getInstance().setCamLatency(Sight::getInstance().getCamLatency() - 5);
				cout << "CAM Latency compensation is now " <<  Sight::getInstance().getCamLatency() << endl;;
				break;
			}
#endif
			case 'h':
				printMenu();
				break;
			case '#':
			case '\e':
				return;
			default:
				break;
		} // switch
	}
}


void TrajectoryPatternController::test1() {
	// basic test: move head in x direction +50mm, -100mm,+100mm,-50mm
	// interrupt in the middle of first movement and move in y axis
	//
	unsigned long now = millis();
	Position p = Kinematics::getInstance().getCurrentPosition();
	TimedPosition tensor;
	tensor.pos = p;
	tensor.atTime = now+1000;
	tensor.pos.point += Point(100,0,0);
	TrajectoryController::getInstance().addPoint(now,tensor);
	tensor.pos.point += Point(0,-100,0);
	tensor.atTime += +1000;
	TrajectoryController::getInstance().addPoint(now,tensor);
	tensor.pos.point += Point(-100,0,0);
	tensor.atTime += +1000;

	TrajectoryController::getInstance().addPoint(now,tensor);
	tensor.pos.point += Point(0,100,0);
	tensor.atTime += +1000;
	TrajectoryController::getInstance().addPoint(now, tensor);
	runPattern(5000UL);
}

void TrajectoryPatternController::test2() {

	unsigned long now = millis();

	// basic test: move head in x direction +50mm, -100mm,+100mm,-50mm
	TimedPosition tensor(millis(),Kinematics::getInstance().getCurrentPosition());

	tensor.pos.point.x += 50;
	tensor.pos.point.y += 0;
	tensor.atTime = now + 1000;

	TrajectoryController::getInstance().addPoint(now, tensor);
	tensor.pos.point.x += 0;
	tensor.pos.point.y += -50;
	tensor.atTime += 1000;

	TrajectoryController::getInstance().addPoint(now, tensor);
	tensor.pos.point.x += -50;
	tensor.pos.point.y += 0;
	tensor.atTime += 1000;

	TrajectoryController::getInstance().addPoint(now, tensor);
	tensor.pos.point.x += 0;
	tensor.pos.point.y += +50;
	tensor.atTime += 1000;

	TrajectoryController::getInstance().addPoint(now, tensor);

	startTime();
	unsigned long startTime = now;
	bool interruptDone = false;
	while (millis() - startTime < 6000UL) {
			loop();
			now = millis();
			if (!interruptDone && (now-startTime >= 700UL)) {
				interruptDone= true;
				cout << "redefine curve at t=" << millis()-startTime << endl;
				TimedPosition current(now, Kinematics::getInstance().getCurrentPosition());
				// interrupt in the middle of first movement
				current.println("current");
				current.pos.point.x += 50*300/1000+50;
				current.pos.point.y += 0;
				current.atTime += 1300;
				TrajectoryController::getInstance().addPoint(now, current,now);
				current.pos.point.x += 0;
				current.pos.point.y += -100;
				current.atTime += 1000;

				TrajectoryController::getInstance().addPoint(now, current);
				current.pos.point.x += -100;
				current.pos.point.y += 0;
				current.atTime += 1000;

				TrajectoryController::getInstance().addPoint(now, current);
				current.pos.point.x += 0;
				current.pos.point.y += +100;
				current.atTime += 1000;

				TrajectoryController::getInstance().addPoint(now, current);
			}
	}
}




void TrajectoryPatternController::runPattern(unsigned long duration_ms) {
	unsigned long startTime = millis();

	// act with a couple of preloaded points
	while (millis() - startTime < duration_ms) {
		loop();
	}
}

void TrajectoryPatternController::changePatternAsAppropriate() {
	if (!changePattern)
		return;

	if (getType() == NO_PATTERN)
		setPattern(SO_BORED);

	// after 10 seconds of no face, give up face interaction
	if (getType() == FACE_INTERACTION && TrajectoryController::getInstance().timeSinceFaceDetected() >  10000) {
		setPattern(SO_BORED);
	}

	// if a new face comes up, interact immediately
	if (getType()== SO_BORED && TrajectoryController::getInstance().timeSinceFaceDetected() < 100) {
		setPattern(FACE_INTERACTION);
	}

	// if Luci is bored, continue with depression after 20s
	if (getType() == SO_BORED  && randomInt(0,millis()-getStartTime())> 20000)
		setPattern(DEPRESSION_PATTERN);

	// if Luci has finished being depressed, quit (which triggers everything once more)
	if ((getType() == DEPRESSION_PATTERN)  && isDone())
		setPattern(NO_PATTERN);
}
