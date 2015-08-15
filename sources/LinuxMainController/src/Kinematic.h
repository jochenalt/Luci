/*
 * Kinematic.h
 *
 *  Created on: 25.02.2015
 *      Author: JochenAlt
 */

#ifndef KINEMATIC_H_
#define KINEMATIC_H_

#include "setup.h"
#include "Spatial.h"
enum PositionAxisType { NO_POSITION=-1, POSITION_X = 0, POSITION_Y = 1, POSITION_Z = 2, ORIENTATION_Z = 4, ORIENTATION_Y= 5} ;
#define POSITION_HISTORY_SIZE (1000/SERVO_LOOP_TIME_MS)

class Kinematics {
public:
	Kinematics() {
		manualPosition = getNullPosition();
		positionAxis = NO_POSITION;
		servoPosition.pos = getNullPosition();
		for (int i = 0;i<POSITION_HISTORY_SIZE;i++) {
			history[i].null();
		}
		historyIdx = 0;
		historyLen = 0;
	}

	static Kinematics& getInstance() {
		static Kinematics instance;
		return instance;
	}

	Position getCurrentPosition();

	static Position getNullPosition();
	static Position getSleepPosition();

	void setup();
	void init();
	void callMenu();
	void printMenu();

	bool moveServosTo(const Position& eye);

	static void lookAt(const Point& eyePosition, const Point& lookAt, Rotation& headOrientation);
	static void lookAt(Position& eyePosition, const Point& lookAt);
	Point getLookAtPoint(const Position &position, float distance);


	Point transformWebcamFaceToPoint(const Point &faceRelToWebcam, const Position& positionWhenFaceWasDetected);
	Point getPointFromBallCoord(const Rotation& rot, float len);
	void limitPosition(Position &pos);
	void limitAcceleration(TimedPosition& nextPos);

	void getHistoricalPosition(Position& pos, unsigned long t);

	void printManualPosition();
	Position getManualPosition();
	void increaseManualPosition(PositionAxisType pPositionAxis, float inc);
	void moveToManualPosition();

private:
	Position getAcceleration(const TimedPosition& nextPos);
	Position getSpeed(const TimedPosition& pos1, const TimedPosition& pos2);
	void addToHistory(const TimedPosition& tensor);
	TimedPosition& getHistory(int IdxDelta);


	Position manualPosition;
	TimedPosition servoPosition;
	TimedPosition lastServoPosition;
	Position speedVector;
	PositionAxisType positionAxis;
	TimedPosition history[POSITION_HISTORY_SIZE];
	int historyIdx;
	int historyLen;
};


#endif /* KINEMATIC_H_ */
