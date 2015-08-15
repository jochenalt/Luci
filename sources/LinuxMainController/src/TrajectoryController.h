/*
 * Trajectory.h
 *
 *  Created on: 26.02.2015
 *      Author: JochenAlt
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "setup.h"
#include <iostream>
#include <string>
#include "Configuration.h"
#include "BezierCurve.h"
#include "Kinematic.h"
#include "Spatial.h"
#include "Util.h"

extern bool debug;

class TrajectoryController {
public:
	TrajectoryController() {
		facePosition.null();
		positionAxis = NO_POSITION;
	}

	void printMenu();
	void callMenu();
	void setup();
	void loop();

	void addPoint(unsigned long now,TimedPosition& pEye, unsigned long startApproachingTime = 0);

	Position      getPointByTime(unsigned long now);
	TimedPosition getPointByIndex(int idx);
	TimedPosition getLastTrajectoryEntry() { return trajectory.get(trajectory.size()-1); };

	int getTrajectorySize() { return trajectory.size(); };

	static TrajectoryController& getInstance() {
		static TrajectoryController instance;
		return instance;
	}

	void halt();
	void print();

	void setDetectedFace(TimedPosition face);
	unsigned long timeSinceFaceDetected();
	TimedPosition getDetectedFace();
private:
	void runBezierCurve();
	void initMovement(unsigned long now, unsigned long startTime);
	void startNextTrajectoryPiece();
	void adaptCurrentTrajectoryPiece(unsigned long now);
	TimedPositionList trajectory;
	Position manualInput;
	BezierCurve bezierCurve;
	PositionAxisType positionAxis;
	TimePassedBy loopTime;
	TimedPosition facePosition;
};

#endif /* TRAJECTORY_H_ */
