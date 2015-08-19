/*
 * TrajectoryPattern.cpp
 *
 *  Created on: 09.04.2015
 *      Author: JochenAlt
 */

#include "TrajectoryPattern.h"
#include "MicroControllerInterface.h"
#include "ServoController.h"

#include "TrajectoryController.h"
#include "TrajectoryPatternController.h"

#include "Kinematic.h"
#include "math.h"
#include "Util.h"

bool debugPattern = false;



Point TrajectoryPatternImpl::lookAt;

void TrajectoryPatternImpl::init() {
	setPhase(startPhase);
	lastPosition.pos = Kinematics::getInstance().getCurrentPosition();
	lastPosition.atTime = millis();
	currentPosition = lastPosition;
};

// generate a sample to look at such that the entire space in front of Luci is sampled
Point& SoBoredPattern::getScanForFacePosition() {

// Luci is supposed to check a random pattern that consists of ROWS*LINES windows for a face
#define ROWS  3
#define LINES 2
#define SAMPLE_NO (ROWS*LINES)
#define X_INTERVAL (-1500-ARM_LENGTH) 	// expect a face in 100cm distance from eye
#define Y_INTERVAL 250   				// constant height
#define Z_INTERVAL 1000  				// interval goes from -500..+500,

	static Position allTensors[ROWS][LINES];
	static bool isSampled[ROWS][LINES];
	static bool initialized = false;
	static int samples = 0;
	if (!initialized) {
		if (debugPattern)
			cout << "SoBoredPattern: Windows Definition " << endl;

		for (int i = 0;i<ROWS;i++) {
			for (int j = 0;j<LINES;j++) {
				isSampled[i][j] = false;
				allTensors[i][j].point = Point(
						X_INTERVAL,
						Kinematics::getInstance().getNullPosition().point.y+(Y_INTERVAL/2)-j*Y_INTERVAL/(LINES-1),
						Z_INTERVAL/2 - i*Z_INTERVAL/(ROWS-1));
				allTensors[i][j].rot = Rotation(0,0,0);
				if (debugPattern)
					cout << "scan window[" << i << "," << j << "]=";
				allTensors[i][j].println();
			}
		}
		if (debugPattern)
			cout << "SoBored Pattern Window Definition done" << endl;

		initialized = true;
	}

	// find scan window not yet scanned in this period
	int rowIndex = randomInt(0,ROWS-1);
	int lineIndex = randomInt(0,LINES-1);
	int safetyCounter = 0; // to ensure termination of the following loop
	while ((safetyCounter++ < ROWS*LINES) && isSampled[rowIndex][lineIndex]) {
		rowIndex++;
		if (rowIndex>= ROWS) {
			rowIndex = 0;lineIndex++;
		}
		if (lineIndex>= LINES) {
			lineIndex = 0;
		}
	}
	samples++;
	// check if everything has been sampled, then reset sample window
	if (samples == LINES*ROWS) { // did we cover all sample windows?
		samples = 0;
		// yes, reset all scan windows and start once again
		for (int i = 0;i<ROWS;i++)
			for (int j = 0;j<LINES;j++)
				isSampled[i][j] = false;
	}
	// return the window not yet scanned

	scanPosition = allTensors[rowIndex][lineIndex].point;
	return scanPosition;
}



TimedPosition SoBoredPattern::fetchNextPosition(const TimedPosition& lastPosition) {

	// initialize with last position, just in case
	TimedPosition nextTensor(lastPosition.atTime,currentPosition.pos);
	Point& lookAt = getLookAt();


	// this pattern runs in two phases
	// 1. moving to a main position with full body movement
	// 2a. look around with small body movement but bigger head turns
	// 2b. short break with very small head movement
	// repeat 2a and 2b multiple times

	switch (getPhase()) {
	case MOVE_TO_NEW_POSITION: {
		Position newPos = Kinematics::getNullPosition();
		Point addPoint(0  + 40*signFloat(newPos.point.x - nextTensor.pos.point.x +0.123*randomPosNeg(),5),
					   10 + 30*signFloat(newPos.point.y - nextTensor.pos.point.y +0.123*randomPosNeg(),5),
					   0  + 80*signFloat(newPos.point.z - nextTensor.pos.point.z +0.123*randomPosNeg(),5));

		nextTensor.pos = newPos + Position(addPoint,Rotation());
		nextTensor.atTime  += 100+addPoint.distance(nextTensor.pos.point)*2;

		mainPosition = nextTensor.pos.point;

		lookAt = getScanForFacePosition();
		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		setPhase(LOOKING_AT);
		break;
	}

	case LOOKING_AT: { // short break with small movement couple of times
		if (getSubPhase() == 0)
			setSubPhase(randomInt(2,4));
		Point addPoint(0,
					   30*signFloat(mainPosition.y - nextTensor.pos.point.y+0.123*randomPosNeg(),5),
					   30*signFloat(mainPosition.z - nextTensor.pos.point.z+0.123*randomPosNeg(),5));

		if (getSubPhase() % 2 == 0) {
			lookAt = getScanForFacePosition(); // fetch new scan position
		} else {
			lookAt = scanPosition;
		}
		nextTensor.atTime  += nextTensor.pos.point.distance(addPoint)*4;
		nextTensor.pos.point = mainPosition + addPoint;

		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		decSubPhase();
		if (getSubPhase() == 0) {
			setPhase(MOVE_TO_NEW_POSITION);
		};

		break;

	}
	default:
		cerr << "SoBord Pattern: invalid phase";
	}
	return nextTensor;
}


TimedPosition DepressionPattern::fetchNextPosition(const TimedPosition& lastPosition) {

	// go slowly to sorrow position
	TimedPosition nextTensor(lastPosition.atTime,currentPosition.pos);
	int duration = 0;
	Point& lookAt = getLookAt();


	// this pattern runs in two phases
	// 1. move to sorrow position which is a bit lower an backward than null position
	// 2a. look around with small body movement but bigger head turns
	// 2b. short break with very small head movement
	// repeat 2a and 2b multiple times

	switch (getPhase()) {
	case MOVE_TO_SORROW_POSITION: {
		nextTensor.pos = Kinematics::getNullPosition();
		Point addPoint(randomInt(40,60),randomInt(-60,-40),0);
		duration = addPoint.length()*50;
		nextTensor.atTime  += duration;
		nextTensor.pos.point += addPoint;

		lookAt = Point(-500,0,randomPosNeg()*100);
		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration,0,LEDPattern::OFF,LEDPattern::LIGHT);

		setPhase(BREATH_IN);
		break;
	}
	case BREATH_IN : {
		Point addPoint(0,0,0);
		nextTensor.pos.point += addPoint;
		duration = 600;
		nextTensor.atTime  += duration;
		lookAt += Point(-300,0,0);
		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration,0,LEDPattern::OFF,LEDPattern::LIGHT);

		setPhase(SORROW_BREAK);
		break;
	}
	case SORROW_BREAK: {
		Point addPoint(0,0,0);
		nextTensor.pos.point += addPoint;
		duration = 600;
		nextTensor.atTime  += duration;
		lookAt += Point(+300,0,0);
		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,0,duration,LEDPattern::OFF,LEDPattern::LIGHT);

		setPhase(DEPRESSION_POSITION);
		break;
	}
	case DEPRESSION_POSITION: {
		Point addPoint(0,-50, 0);
		duration = 1000;
		nextTensor.atTime  += duration;
		nextTensor.pos.point += addPoint;
		lookAt += Point(-1000,550,randomPosNeg()*200);
		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration/2,duration/2,LEDPattern::OFF,LEDPattern::LIGHT);

		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);

		setPhase(DEEP_DEPRESSION_POSITION);
		break;
	}
	case DEPRESSION_HIGH:{
		Point addPoint(0,randomInt(-40,-20),0);
		duration = 1000;
		nextTensor.atTime  += duration;
		nextTensor.pos.point += addPoint;
		lookAt = Point(-500,700,0);

		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration/2,duration/2,LEDPattern::DARK,LEDPattern::LIGHT);

		setPhase(DEEP_DEPRESSION_POSITION);
		break;
	}
	case DEEP_DEPRESSION_POSITION: {
		if (getSubPhase() == 0) {
			setSubPhase(8*3+1);
			Point addPoint(0,randomInt(-40,-20),0);
			duration = 1000;
			nextTensor.atTime  += duration;
			nextTensor.pos.point += addPoint;
			lookAt = Point(-400,0,0);

			Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		} else {
			decSubPhase();
			duration = 200;
			nextTensor.atTime  += duration;
			if (getSubPhase() > 0) {
				float amplitude = 20+30*(getSubPhase()/(8.0*3.0));
				float z = amplitude*sinFast(getSubPhase()*(2.0*M_PI/(8.0)));
				lookAt = Point(-400,0,z);
				Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
			} else {
				lookAt = Point(-400,0,0);
				Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
				setPhase(DONE);
			}
		}
		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration/2,duration/2,LEDPattern::OFF,LEDPattern::DARK);

		break;
	}
	case DONE:
		nextTensor.null();
		return nextTensor;
	default:
		cerr << "Depression Pattern: invalid phase" << endl;
	}
	if (debugPattern) {
		nextTensor.println("   pos");
		lookAt.println("   look at");
	}
	return nextTensor;
}

TimedPosition FaceInteractionPattern::fetchNextPosition(const TimedPosition & lastPosition) {
	TimedPosition nextTensor(lastPosition.atTime,currentPosition.pos);
	int duration = 0;
	Point facePos = TrajectoryController::getInstance().getDetectedFace().pos.point;
	facePos.print("facepos");
	int sgnY = signFloat(nextTensor.pos.point.y > Kinematics::getNullPosition().point.y,5);
	int sgnZ = signFloat(nextTensor.pos.point.z > Kinematics::getNullPosition().point.z,5);
	// int sgnFaceY = signFloat(facePos.y > Kinematics::getNullPosition().point.y,5);
	int sgnFaceZ = signFloat(facePos.z > Kinematics::getNullPosition().point.z,5);

	switch (getPhase()) {
		case RECOGNIZE_FACE: {

			nextTensor.pos = Kinematics::getNullPosition();
			duration = 3000;
			nextTensor.atTime  += duration;
			float z = 0;
			if (facePos.x < 50)
				z = (facePos.z * nextTensor.pos.point.x)/facePos.x;
			z = constrain(z,-70,70);
			nextTensor.pos.point += Point(-40,-50,z);
			if (!facePos.isNull())
				Kinematics::lookAt(nextTensor.pos, facePos);
			nextTensor.setStareAtFace(true);
			nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration,0,LEDPattern::DARK,LEDPattern::LIGHT);

			setPhase(SURPRISED_LOOK);

			break;
		}
		case SURPRISED_LOOK:
			duration = 1000;
			nextTensor.atTime  += duration;
			nextTensor.pos = Kinematics::getNullPosition();
			nextTensor.pos.point += Point(60,30,-sgnFaceZ*40);
			if (!facePos.isNull())
				Kinematics::lookAt(nextTensor.pos, facePos);
			nextTensor.setLookAtFace(facePos);
			nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration,0,LEDPattern::DARK,LEDPattern::FULL);

			setPhase(SLOW_APPROACH);

			break;
		case SLOW_APPROACH: {
			duration = 3000;
			nextTensor.atTime  += duration;
			nextTensor.pos = Kinematics::getNullPosition();
			float z = 0,y = 0;
;
			if (facePos.x < 50) {
				z = (facePos.z * nextTensor.pos.point.x)/facePos.x;
				z = constrain(z,-70,70);
				y = (facePos.y * nextTensor.pos.point.x)/facePos.x;
				y = constrain(y,-50,50);
			}

			nextTensor.pos.point += Point(-80,y,z);
			if (!facePos.isNull())
				Kinematics::lookAt(nextTensor.pos, facePos);
			nextTensor.setStareAtFace(true);
			nextTensor.getLamp().setFadeIn(nextTensor.atTime,duration, LEDPattern::LIGHT);

			setPhase(QUICK_MOVE_TO_CORNER);
			break;
		}
		case QUICK_MOVE_TO_CORNER: {
			duration = 1000;
			nextTensor.atTime  += duration;
			nextTensor.pos = Kinematics::getNullPosition();
			nextTensor.pos.point += Point(+20,-sgnY*60,sgnFaceZ*60);

			nextTensor.setLookAtFace(facePos);
			nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration,0,LEDPattern::DARK,LEDPattern::LIGHT);

			if (!facePos.isNull())
				Kinematics::lookAt(nextTensor.pos, facePos);
			setPhase(SLOW_MOVE_AROUND_FACE);
			// setPhase(RECOGNIZE_FACE);

			break;
		}
		case SLOW_MOVE_AROUND_FACE: {
			if (getSubPhase() == 0) {
				setSubPhase(4, nextTensor.pos.point);
			}
			bool moveOrBreak = getSubPhase() % 2 == 0;
			if (moveOrBreak) { // move it
				duration = 800;
				nextTensor.atTime  += 800;
				nextTensor.pos.point = getSubPhasePoint();
				nextTensor.pos.point += Point(0,-sgnY*30 ,-sgnZ*25);
			} else { // short break and follow the face
				duration = 2500;
				nextTensor.pos.point += Point(0,20*sgnY, 20*sgnZ);
			}
			nextTensor.atTime  += duration;
			nextTensor.setStareAtFace(true);
			nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration,duration,LEDPattern::DARK,LEDPattern::LIGHT);

			decSubPhase();
			if (getSubPhase() == 0)
				setPhase(randomBool()?SLOW_APPROACH:NODDING_MOVE);
			if (!facePos.isNull())
				Kinematics::lookAt(nextTensor.pos, facePos);

			break;
		}
		case NODDING_MOVE: {
			duration = 250;
			if (getSubPhase() == 0)
				setSubPhase(9);
			if ((getSubPhase() % 2) == 0) {
				nextTensor.pos.point += Point(0,+20,0);
				nextTensor.pos.rot.z += +15;
				nextTensor.getLamp().setFadeInOut(nextTensor.atTime,0,duration,LEDPattern::OFF,LEDPattern::LIGHT);

			} else {
				nextTensor.pos.point += Point(0,-20,0);
				nextTensor.pos.rot.z += -15;
				nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration,0,LEDPattern::OFF,LEDPattern::LIGHT);
			}
			nextTensor.atTime  += duration;

			decSubPhase();
			if (getSubPhase() == 0) {
				setPhase(RECOGNIZE_FACE);
			}
			break;
		}
	default:
		cerr << "face interaction pattern: invalid phase" << endl;
	}

	return nextTensor;
}


TimedPosition WakeUpPattern::fetchNextPosition(const TimedPosition& lastPosition) {
	TimedPosition nextTensor(lastPosition);
	int duration = 0;
	Point& lookAt = getLookAt();

	switch (getPhase()) {
	case SLEEP: {
		nextTensor.pos = Kinematics::getSleepPosition();
		duration = 2000;
		nextTensor.atTime  += duration;
		nextTensor.getLamp().setFadeIn(nextTensor.atTime,duration,LEDPattern::DARK);

		setPhase(HEAD_UP);
		break;
	}
	case HEAD_UP: {
		nextTensor.pos = Kinematics::getNullPosition();
		nextTensor.pos.rot.z -= 15;
		duration = 2000;
		nextTensor.atTime  += duration;
		nextTensor.getLamp().setFadeIn(nextTensor.atTime,duration,LEDPattern::DARK);
		setPhase(BLINK);
		break;
	}
	case BLINK: {
		setOrDecSubPhase(1,LOOK_AROUND);

		duration = 800;
		nextTensor.pos = Kinematics::getNullPosition();
		nextTensor.pos.rot.z -= 15;
		nextTensor.atTime  += duration;
		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration/2,duration/2,LEDPattern::LIGHT,LEDPattern::OFF);
		break;
	}
	case LOOK_AROUND: {
		setOrDecSubPhase(6,DONE);

		bool shortBreak = (getSubPhase() % 2) == 1;
		if (shortBreak) {
			duration = 800;
			nextTensor.atTime  += duration;
			nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration/2,duration/2,LEDPattern::FULL,LEDPattern::OFF, true);
		} else {
			nextTensor.pos = Kinematics::getNullPosition();

			duration = 350;
			nextTensor.atTime  += duration;
			int subsubphase = (getSubPhase()) / 2;

			if ((subsubphase == 3) || (subsubphase == 2)) {
				lookAt = Point(-1000,nextTensor.pos.point.y, ((subsubphase % 2)*2 -1) * 500);
			} else {
				lookAt = Point(-1000,nextTensor.pos.point.y + (subsubphase % 2) * 300,0);
			}
			nextTensor.getLamp().setConstantDuty(nextTensor.atTime, duration, LEDPattern::FULL);
			Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		}


		break;
	}
	case DONE: {
		nextTensor.null();
		break;
	}
	default:
		cerr << "WakeUpPattern: invalid phase" << endl;
	}
	return nextTensor;
}


TimedPosition NaughtyLampPattern::fetchNextPosition(const TimedPosition& lastPosition) {

	TimedPosition nextTensor (lastPosition);
	int duration = 0;

	Point& lookAt = getLookAt();

	switch (getPhase()) {
	case SLEEP: {
		nextTensor.pos = Kinematics::getSleepPosition();
		duration = 3000;
		nextTensor.atTime  += duration;
		nextTensor.getLamp().setFadeOut(nextTensor.atTime,duration,LEDPattern::FULL);

		setPhase(WATCH_CAREFULLY);

		break;
	}
	case WATCH_CAREFULLY: {
		setOrDecSubPhase(5,BLINK);

		if ( (getSubPhase() == 5) || (getSubPhase() == 3) || (getSubPhase() == 1)) {
			duration = (getSubPhase() == 5)?600:300;
			nextTensor.pos  = Kinematics::getInstance().getNullPosition();
			nextTensor.pos.point  += Point(0,0,0);
			nextTensor.atTime  += duration;
			lookAt = Point(-1000,(getSubPhase()==1)?500:300,(getSubPhase() == 5)?1000:((getSubPhase() == 3)?-1000:-200));
			Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
			nextTensor.getLamp().setConstantDuty(nextTensor.atTime,duration,LEDPattern::DARK);
		} else {
			duration = 1000;
			nextTensor.atTime  += duration;
			nextTensor.pos.point += Point(0,0,(getSubPhase() == 4)?50:((getSubPhase() == 2)?-50:-20));

			nextTensor.getLamp().setConstantDuty(nextTensor.atTime,duration,LEDPattern::FULL);
		}

		break;
	}
	case BLINK: {
		// setOrDecSubPhase(2,GO_DOWN);
		setOrDecSubPhase(2,PUSH_BOX);

		if (getSubPhase() == 2) {
			duration = 700;
			nextTensor.pos  = Kinematics::getInstance().getNullPosition();
			nextTensor.pos.point += Point(0,50,0);
			lookAt = Point(-1000,300,-300);
			nextTensor.atTime  += duration;
			Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
			nextTensor.getLamp().setConstantDuty(nextTensor.atTime,duration,LEDPattern::FULL);
		} else {
			duration = 400;
			nextTensor.atTime  += duration;
			nextTensor.getLamp().setFadeIn(nextTensor.atTime,duration/4,LEDPattern::FULL);
		}
		break;
	}
	case PUSH_BOX: {
		// push the box two times. Start with position right in front of the box
		// with a small move forward with each step
		setOrDecSubPhase(4,CHECK);

		if (getSubPhase() % 2 == 0) { // start position
			duration = 800;
			nextTensor.pos  = Kinematics::getInstance().getNullPosition();
			nextTensor.pos.point.y = 50;
			nextTensor.pos.point.x += 180 - (4-getSubPhase())*10;
			nextTensor.atTime  += duration;
			lookAt = Point(nextTensor.pos.point.x,0,nextTensor.pos.point.z);
			Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
			nextTensor.getLamp().setConstantDuty(nextTensor.atTime,duration,LEDPattern::DARK);
		} else { // push position
			duration = 500;
			nextTensor.pos  = Kinematics::getInstance().getNullPosition();
			nextTensor.pos.point.y = 50;
			nextTensor.pos.point.x += 180 -0- (4-getSubPhase())*10;
			nextTensor.atTime  += duration;
			lookAt = Point(nextTensor.pos.point.x-50,0,nextTensor.pos.point.z);
			Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
			nextTensor.getLamp().setConstantDuty(nextTensor.atTime,duration,LEDPattern::FULL);
		}

		break;
	}
	case CHECK: {
		// go up again and check carefully anyone watching
		setOrDecSubPhase(1,WATCH);
		if (getSubPhase() == 1) {
			duration = 600;
			nextTensor.pos  = Kinematics::getInstance().getNullPosition();
			nextTensor.pos.point  += Point(0,50,0);
			nextTensor.atTime  += duration;
			lookAt = Point(-1000,300,-1000);
			Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
			nextTensor.getLamp().setConstantDuty(nextTensor.atTime,duration,LEDPattern::DARK);
		} else {
			duration = 1000;
			nextTensor.atTime  += duration;
			nextTensor.pos.point += Point(0,0,-50);
			nextTensor.getLamp().setConstantDuty(nextTensor.atTime,duration,LEDPattern::FULL);
		}
		break;
	}
	case WATCH: {
		// now go down and watch naughty pictures
		setOrDecSubPhase(3,WATCH_INTENSIVELY);
		duration = 1000;
		nextTensor.pos = Kinematics::getInstance().getNullPosition();
		int leftRight = (getSubPhase()/2 % 2)*2-1; // -1, +1
		int upDown = (getSubPhase() % 2); // +1,0

		nextTensor.pos.point += Point(100,-80,0);
		nextTensor.atTime  += duration;

		lookAt = Point(nextTensor.pos.point.x - upDown*200,
							 0,
							 -leftRight*120);
		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);

		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration/2,duration/2,LEDPattern::DARK,LEDPattern::LIGHT, true);
		break;
	}
	case WATCH_INTENSIVELY: {
		setOrDecSubPhase(6,WATCH);
		static int leftRight, upDown;

		if (getSubPhase() == 6) {
			leftRight = randomPosNeg();
			upDown 	  = randomPosNeg();

		} else {
			if ((getSubPhase() % 2) == 1)
				leftRight = randomPosNeg();
			upDown = -upDown;
		}
		if (nextTensor.pos.point.z*leftRight >= 0 )
			duration = 1500;
		else
			duration = 800;

		nextTensor.pos = Kinematics::getInstance().getNullPosition();
		int upDownX = randomInt(25,35);
		nextTensor.pos.point += Point(60+upDown*upDownX, -200,leftRight*60);
		nextTensor.atTime  += duration;
		lookAt = Point(nextTensor.pos.point.x - ((upDown==1)?50:0), 0, leftRight*140);
		Kinematics::lookAt(nextTensor.pos.point, lookAt, nextTensor.pos.rot);
		nextTensor.getLamp().setFadeInOut(nextTensor.atTime,duration/2,duration/2,LEDPattern::LIGHT,LEDPattern::FULL, true);

		break;
	}
	case DONE:
		nextTensor.null();
		return nextTensor;
	default:
		cerr << getPatternName() << ":invalid phase" << endl;
	}
	if (debugPattern) {
		nextTensor.println("   pos");
		lookAt.println("   look at");
	}
	return nextTensor;
}
