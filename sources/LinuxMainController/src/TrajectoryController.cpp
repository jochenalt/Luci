/*
 * Trajectory.cpp
 *
 *  Created on: 26.02.2015
 *      Author: JochenAlt
 */

#include "TrajectoryController.h"
#include "Kinematic.h"
#include <boost/lexical_cast.hpp>
#include "setup.h"
#include "MicroControllerInterface.h"
#include "TrajectoryPattern.h"
#include "Util.h"

bool debugTrajectory = false;
extern bool debugBezier;
extern bool debugKinematic;
void TrajectoryController::setup() {
	Position initialPosition;
	// assume the initial position as start
	trajectory.add(TimedPosition(0,Kinematics::getInstance().getNullPosition()));
	bezierCurve.reset();
	facePosition.null();
}

void TrajectoryController::initMovement(unsigned long now, unsigned long approachingTime) {
	if (debugTrajectory)
		 cout << "initTensor"<< endl;
	trajectory.get(0).atTime = now;// start point is current position, start from now
	TimedPosition startPoint = trajectory.get(0);
	TimedPosition endPoint	= trajectory.get(1);
	if (endPoint.atTime < startPoint.atTime) {
		endPoint.atTime = startPoint.atTime + 1000;
		cerr << "ERR:TrajectoryController::initTensor.endPoint="<< endPoint.atTime << "startPoint="<< startPoint.atTime << endl;
	}
	TimedPosition afterPoint; // null

	if (trajectory.size() >= 3) {
		afterPoint = trajectory.get(2);
	}
	bezierCurve.set(startPoint, startPoint,endPoint, afterPoint);

	// set the next lamp pattern
	LEDController::getInstance().setPattern(endPoint.getLamp());

	print();
}


void TrajectoryController::print() {
	if (debugTrajectory) {
		cout << "Trajectory" << endl;
		trajectory.print();
		if (debugTrajectory)
			bezierCurve.println("bezier");
	}
}


TimedPosition TrajectoryController::getPointByIndex(int idx) {
	return trajectory.get(idx);
}

Position TrajectoryController::getPointByTime(unsigned long time) {
	Position eye;

	if (time < trajectory.get(0).atTime) {
		cerr << "BUG:getPoint does not work in the very past";
		return eye;
	}

	// if time is after defined trajectory, assume the last point
	if (getLastTrajectoryEntry().atTime<time) {
		eye = getLastTrajectoryEntry().pos;
		return eye;
	}

	// find the right end, i.e. the point which time is after passed time
	int endIndex = 0;
	while (trajectory.get(endIndex).atTime < time)
		endIndex++;
	int startIndex = endIndex-1;
	if (startIndex < 0) {
		cerr << "BUG: startIndex < 0";
		return eye;
	}

	// current curve? Then take bezier, otherwise linear interpolation
	if (startIndex == 0) {
		eye = bezierCurve.getPointOfLine(time);
	}
	else {
		eye = trajectory.get(startIndex).getPointOfLine(time, trajectory.get(endIndex));
	}
	return eye;
}

void TrajectoryController::addPoint(unsigned long now, 				/* current time of caller */
									TimedPosition& pEye 			/* included time is used */,
									unsigned long startApproachingTime /* = 0 */) {
	if ((startApproachingTime  == 0) || (startApproachingTime  > pEye.atTime))
		startApproachingTime = pEye.atTime;

	TimedPosition eye(pEye);
	if (debugTrajectory) {
		eye.println("addPoint");
		/*
		bool saveDebugKinematic = debugKinematic;
		debugKinematic = true;
		Kinematics::getInstance().moveServosTo(pEye.pos);
		debugKinematic = saveDebugKinematic;
		*/
	}

	// check for boundaries first
	Kinematics::getInstance().limitPosition(eye.pos);

	// compute the point where we are at starting point.
	// Returns null point if current trajectory ends before startApproachingTime
	Position startPoint = getPointByTime(startApproachingTime);

	// is new point within the first interval that is currently running?
	bool pointIsInRunningCurve = (trajectory.size() >= 2) && (getPointByIndex(1).laterThan(startApproachingTime));

	// is new point within the next interval that is currently running?
	bool pointIsRightAfterRunningCurve = ((trajectory.size() >= 3) &&
			getPointByIndex(1).earlierThan(startApproachingTime) &&
			getPointByIndex(2).laterThan(startApproachingTime)) ||
		((trajectory.size() == 2) && getPointByIndex(1).earlierThan(startApproachingTime));

	// if movement start before last point, we have to change the current trajectory
	if (startApproachingTime < getLastTrajectoryEntry().atTime) {

		// remove all trajectory points happening after the starting time
		trajectory.removeLaterThan(startApproachingTime);

		// if the starting point happens earlier than the target position
		// add the predicted position at the starting time
		// (the new point is added anyhow later on)
		// then add an intermediate point the movement starts from
		if (startApproachingTime < eye.atTime) {
			// add starting point of new movement
			TimedPosition intermediatePoint(startApproachingTime,startPoint);
			trajectory.add(intermediatePoint);
			intermediatePoint.println("intermediate point");
		}
	}

	// add new position
	trajectory.add(eye);

	// we need at least two positions to move
	if (trajectory.size() >= 2) {
		if (!bezierCurve.isActive())
			initMovement(now,eye.atTime); // we start to move
		else {
			if (pointIsInRunningCurve || pointIsRightAfterRunningCurve) {
				adaptCurrentTrajectoryPiece(now); // we adapt current curve
			}
		}
	}
}

void TrajectoryController::startNextTrajectoryPiece() {
	if (trajectory.size() > 1) {
		if (debugTrajectory)
			trajectory.get(1).println("takeNextTensor");

		if (trajectory.size() == 2) {
			// after deleting the first point, just one is left and we are already there, not enough for a trajectory.
			bezierCurve.reset();
		} else { // getListSize > 2

			// new end support point is the next point mirrored at the new end point
			// if available, otherwise use the endPoint (which slows down the trajectory speed nicely)
			TimedPosition afterPoint;
			if (trajectory.size() >= 4)
				afterPoint = trajectory.get(3);

			TimedPosition& startPoint = trajectory.get(1);
			TimedPosition& endPoint = trajectory.get(2);

			// if we are supposed to update lucis direction towards the face, do it
			if (startPoint.isLookAtFace() || startPoint.isStareAtFace()) {
				TimedPosition currentFace = TrajectoryController::getInstance().getDetectedFace();
				// dont touch the start point, since we are currently at this point
				// modification would lead to  a jump
				Kinematics::getInstance().lookAt(endPoint.pos.point,
						currentFace.pos.point,
						endPoint.pos.rot);
				cout << "next trajecory to look at face curr=" << Kinematics::getInstance().getCurrentPosition().rot
					 << "start=" << startPoint.pos.rot << " end=" << endPoint.pos.rot << endl;

			}
			bezierCurve.set(trajectory.get(0), startPoint, endPoint,afterPoint);

			// set the next lamp pattern
			LEDController::getInstance().setPattern(endPoint.getLamp());

		}

		// delete first tensor, not used anymore
		trajectory.remove(0);

		if (debugTrajectory) {
			trajectory.println("trajectory");
			bezierCurve.println("bezier");
		}
	}
}

void TrajectoryController::adaptCurrentTrajectoryPiece(unsigned long now) {
	if (debugTrajectory)
		cout << "redefine CurrentCurve" << endl;
	if (trajectory.size() > 1) {
		TimedPosition endPoint = trajectory.get(1);
		if (trajectory.size() > 2) {
			TimedPosition nextPoint = trajectory.get(2);
			unsigned long startTime = bezierCurve.getStart().atTime;
			unsigned long endTime = bezierCurve.getEnd().atTime;
			float dT = endTime-startTime;
			if (startTime != endTime) {
				// re-compute the start support point and adapt the end point.
				// Take care that the new support point leaves the current direction the same and slowly goes towards the new endpoint.
				float t = ((float)now -(float)startTime) / dT;
				bezierCurve.amend(t, endPoint, nextPoint);

				// start from now, ignore the curve that has been done already
				// trajectory.get(0).atTime= now;
			} else {
				cerr << "adaptCurrentTrajectoryPiece: dT = 0" << endl;
			}
		} else {
			bezierCurve.reset(); // stop
		}
		print();
	}
}

void TrajectoryController::runBezierCurve() {
	if (bezierCurve.isActive()) {
		unsigned long now = millis();
		TimedPosition startPos = trajectory.get(0);
		TimedPosition& endPos = trajectory.get(1);
		TimedPosition afterPos;
		if (trajectory.size() > 2)
			afterPos = trajectory.get(2);

		// compute ratio 0..1 where we are in current part of the curve
		float t = ((float)now-(float)startPos.atTime) / ((float)endPos.atTime- (float)startPos.atTime);
		if (t <= 1.0) {
			TimedPosition tensor = bezierCurve.getCurrent(t);


			// limit the acceleration for a smooth movement
			Kinematics::getInstance().limitAcceleration(tensor);

			// check if we need to adapt the curve online by looking at a stupid face
			// do this by adapting the end point of the curve in order to smooth the movement
			static Point filteredFacePosition;
			if (startPos.isStareAtFace() || startPos.isLookAtFace()) {
				// current face is either the just detected face (stareAtFace),
				// or the defined face from start point (lookAtFace)
				TimedPosition currentFace =
						startPos.isStareAtFace()?
								TrajectoryController::getInstance().getDetectedFace():
								TimedPosition(startPos.atTime,Position(startPos.getLookAtFacePoint(), Rotation()));

				if (!currentFace.isNull()) {
					if (filteredFacePosition.isNull()) {
						// define an artificial point Luci is currently looking
						// at in the same distance like the found face
						Position curr = Kinematics::getInstance().getCurrentPosition();
						Point lookAt = Kinematics::getInstance().getLookAtPoint(curr, currentFace.pos.point.distance(curr.point));
						filteredFacePosition = lookAt;
					}
					else {
						float dist = filteredFacePosition.distance(currentFace.pos.point);
						float f = (1.0-dist/(150+dist))/10 - 0.02;
						filteredFacePosition = filteredFacePosition*(1.0-f) + currentFace.pos.point*f;
						// cout << "d=" << dist << "f" << f*100 << "%" << endl;
					}
				}

				if (!filteredFacePosition.isNull()) {
					Rotation orientationTowardsFace;
					Kinematics::getInstance().lookAt(tensor.pos.point,
							filteredFacePosition,
							orientationTowardsFace);
					tensor.pos.rot = orientationTowardsFace;
				}

				static unsigned long lastFaceDetection = 0;
				if (currentFace.atTime != lastFaceDetection) {
					lastFaceDetection = currentFace.atTime;
					// new face has been detected
					// patch the end of the trajctory, to ensure that in case the next piece is supposed to
					// not stare at the face, there is no sudden movement.

					// patch the end position of the trajectory
					// (if possible patch the next trajectory as well)
					// dont use amend, since it is anyhow patched with every iteration
					Rotation endPointRot;
					Kinematics::getInstance().lookAt(endPos.pos.point,filteredFacePosition,endPointRot);
					endPos.pos.rot = endPointRot;
					bezierCurve.patchB(endPos,endPos.pos);
				}
				if (debugBezier) {
					tensor.println("faced-bezier-pnt");
				}
			} else {
				// reset the latest face position to not have a jump when stare/lookAtFace-mode starts again
				filteredFacePosition.null();

				if (debugBezier) {
					tensor.println("bezier-pnt");
					cout << "t=" << t;
					// bezierCurve.print();
				}
			}
			Kinematics::getInstance().moveServosTo(tensor.pos);
		} else {
			// we are later than approaching time, so this piece of the curve has been done already, take next piece
			startNextTrajectoryPiece();
			runBezierCurve();
		}
	}
}

void TrajectoryController::loop() {
	if (loopTime.isDue_ms(SERVO_LOOP_TIME_MS) ) {

		// set the current position by using a bezier curve
		runBezierCurve();

		// set the LED's duty (pattern has been defined in startNextTrajectoryPiece)
		LEDController::getInstance().loop();

		// if a pwm value of any servo has changed, send values via I2C to the ATmega
		// dont send this too often to give the uC a chance to set the pwm value between a duty cycle.
		// ideally exactly every 20ms (that is the servo refresh rate)
		MicroControllerInterface::getInstance().sendIfValueHasChanged();
	}
}


void TrajectoryController::setDetectedFace(TimedPosition face) {
	if (!face.isNull()) {
		face.pos.point.x = constrain(face.pos.point.x, -1500, -600);
		face.pos.point.y = constrain(face.pos.point.y, 0, 3000);
		face.pos.point.z = constrain(face.pos.point.z, -3000,+3000);

		facePosition = face;
	}
}


TimedPosition TrajectoryController::getDetectedFace() {
	return facePosition;
}

unsigned long TrajectoryController::timeSinceFaceDetected() {
	return timediff(millis(),facePosition.atTime);
}



void TrajectoryController::halt() {
	TimedPosition current = bezierCurve.getCurrent(millis());
	while (trajectory.size()>0)
		trajectory.remove(0);
	addPoint(millis(), current);
	bezierCurve.reset();

	print();
}

void TrajectoryController::printMenu() {
	cout << "Trajectory" << endl
			<< "p     - power on/off" << endl
			<< "s     - store position" << endl
			<< "d     - delete last" << endl
			<< "r     - run" << endl
			<< "x/y/z - set eye's position" << endl
			<< "Y/Z   - set eye's orientation" << endl
			<< "+/-   - position" << endl;

	print();
}



void TrajectoryController::callMenu() {
	while (true)  {
		loop();

		char c = keyPressed();
		switch (c) {
			case 'p': {
				bool powerOn = MicroControllerInterface::getInstance().getPowerOn();
				cout << "switch servo power " << (powerOn?"off":"on")<< endl;
				MicroControllerInterface::getInstance().power(!powerOn);
				break;
			}
			case 's': {
					Position p = Kinematics::getInstance().getManualPosition();
					TimedPosition tensor(millis()+5000,p);
					addPoint(millis(),tensor);
					break;
				}
			case 'd':
				if (trajectory.size()> 0)
					trajectory.remove(trajectory.size()-1);
				break;
			case 'r': {
					unsigned long time = millis();
					for (int i = 0;i<trajectory.size();i++) {
						trajectory.get(i).atTime = time;
						time += 10000;
					};
				}
				break;
			case 'x':
				positionAxis = POSITION_X;
				break;
			case 'y':
				positionAxis = POSITION_Y;
				break;
			case 'z':
				positionAxis = POSITION_Z;
				break;
			case 'Y':
				positionAxis = ORIENTATION_Y;
				break;
			case 'Z':
				positionAxis = ORIENTATION_Z;
				break;
			case '+':
				Kinematics::getInstance().increaseManualPosition(positionAxis,+1.0);
				Kinematics::getInstance().moveToManualPosition();
				break;
			case '-':
				Kinematics::getInstance().increaseManualPosition(positionAxis,-1.0);
				Kinematics::getInstance().moveToManualPosition();
				break;
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
