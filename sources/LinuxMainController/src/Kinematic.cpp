/*
 * Kinematic.cpp
 *
 *  Created on: 25.02.2015
 *      Author: JochenAlt
 */

#include <iostream>
#include <iomanip>
#include "Util.h"
#include "Spatial.h"

#include "Kinematic.h"
#include "ServoController.h"
#include "MicroControllerInterface.h"
#include "TrajectoryController.h"
#include "math.h"

bool debugKinematic = false;
bool moveDirectly = true;


Position Kinematics::getNullPosition() {
	Position nullP;
	nullP.point.set(- FOOT_LENGTH - ARM_LENGTH - SHOULDER_LENGTH - NECK_LENGTH - EYE_LENGTH ,BASE_HEIGHT + LEG_LENGTH + NECK_HEIGHT,0);
	nullP.rot.set (0,0,0);
	return nullP;
}

Position Kinematics::getSleepPosition() {
	Position nullP;
	nullP.point.set(- FOOT_LENGTH - ARM_LENGTH - SHOULDER_LENGTH - NECK_HEIGHT,BASE_HEIGHT + LEG_LENGTH -NECK_LENGTH-EYE_LENGTH,0);

	Point lookAt(nullP.point.x,0,0);
	Kinematics::getInstance().lookAt(nullP.point,lookAt,nullP.rot);
	return nullP;
}
void Kinematics::moveToManualPosition() {
	moveServosTo(manualPosition);
}

void Kinematics::increaseManualPosition(PositionAxisType pPositionAxis,float inc) {
	switch (pPositionAxis) {
		case POSITION_X:
			manualPosition.point.x += inc;
			break;
		case POSITION_Y:
			manualPosition.point.y += inc;
			break;
		case POSITION_Z:
			manualPosition.point.z += inc;
			break;
		case ORIENTATION_Y:
			manualPosition.rot.y += inc;
			break;
		case ORIENTATION_Z:
			manualPosition.rot.z += inc;
			break;
		default:
		break;
	}

	printManualPosition();
}
void Kinematics::init() {

}

void Kinematics::setup() {
	positionAxis = NO_POSITION;
}

void  Kinematics::getHistoricalPosition(Position& pos, unsigned long t) {
	if (historyLen == 0) {
		// we are at the very beginning and do not have a history yet, so take the current position, it's the best we have
		pos = Kinematics::getInstance().getCurrentPosition();
	} else {

		int idxDelta = 0;
		while ((abs(idxDelta) < POSITION_HISTORY_SIZE) && (getHistory(idxDelta).atTime > t)) {
			idxDelta--;
		}
		pos = getHistory(idxDelta).pos;
		// cout << "getHistoricalPosition(" << t << ")=" << pos << endl;
	}
}


TimedPosition& Kinematics::getHistory(int IdxDelta) {
	int idx = historyIdx+IdxDelta;
	while (idx <0)
		idx += POSITION_HISTORY_SIZE;
	idx = idx % POSITION_HISTORY_SIZE;
	return history[idx];
}

void Kinematics::addToHistory(const TimedPosition& tensor) {
	// store a history of positions for 2 seconds with a resolution of 20ms
	// (that's a bit more precise than the update frequency of the face recognition)
	historyIdx = (historyIdx +1) % POSITION_HISTORY_SIZE;
	history[historyIdx] = tensor;
	historyLen = max(historyLen+1,POSITION_HISTORY_SIZE);
}


/**
 *  set the orientation of the head such that the eye is looking at a passed point
 */
void Kinematics::lookAt(const Point& eyePosition, const Point& lookAt, Rotation& headOrientation) {

	float dx = eyePosition.x-lookAt.x;
	float dz = lookAt.z-eyePosition.z;
	float dy = lookAt.y-eyePosition.y;
	if (dx >= 0) {
		headOrientation.x = 0;
		headOrientation.z = degrees(atan2Fast(dy,dx));
		headOrientation.y = -degrees(atan2Fast(dz,sqrt(dx*dx+dy*dy)));
		// headOrientation.z = degrees(atan2Fast(dy,sqrt(dx*dx+dz*dz)));
		// headOrientation.y = -degrees(atan2Fast(dz,dx));

		// cout << "lookat eye" << eyePosition << "la=" << lookAt << "(" << dx << "," << dy << "," << dz << ") s=" << sqrt(dx*dx+dz*dz) << " ho" << headOrientation << endl;
	} else {
		headOrientation.x = 0;
		headOrientation.z = degrees(atan2Fast(dy,-dx));
		headOrientation.y = -degrees(atan2Fast(dz,sqrt(dx*dx+dy*dy)));
		// headOrientation.z = degrees(atan2Fast(dy,sqrt(dx*dx+dz*dz)));
		// headOrientation.y = -degrees(atan2Fast(dz,-dx));

		// cout << "lookat eye" << eyePosition << "la=" << lookAt << "(" << dx << "," << dy << "," << dz << ") s=" << sqrt(dx*dx+dz*dz) << " ho" << headOrientation << endl;
	}
}

void Kinematics::lookAt(Position& eyePosition, const Point& lookAt) {
	Kinematics::lookAt(eyePosition.point,lookAt,eyePosition.rot);
}

Point Kinematics::transformWebcamFaceToPoint(const Point &faceRelToWebcam, const Position &positionWhenFaceWasDetected) {
	// given coordinates are relative to eye, so find out the position of the eye at the passed time

	// Rotate the face coord relative to the webcam by the head orientation to compute the face coord
	// relatively to the head's non-rotated coord system. Translate the face by the head's position
	// to get the coord in luci's absolute coord system
	Matrix3x3 rotationMatrixHead;
	rotationMatrixHead.computeRotationMatrix(positionWhenFaceWasDetected.rot);
	Point face(faceRelToWebcam);
	rotationMatrixHead.rotatePoint(face);

	// take face and translate by luci's eye position to get absolute coord of face
	face.translate(positionWhenFaceWasDetected.point);

	Point faceDetected;
	faceDetected = face;
	return faceDetected;
}

Point Kinematics::getLookAtPoint(const Position &position, float distance) {
	Matrix3x3 rotationMatrixHead;
	rotationMatrixHead.computeRotationMatrix(position.rot);
	Point sight(-distance,0,0); // straight sight
	rotationMatrixHead.rotatePoint(sight);

	Point lookAt(position.point);
	lookAt.translate(sight);
	return lookAt;
}

Point Kinematics::getPointFromBallCoord(const Rotation& rot, float len) {
	Matrix3x3 rotationMatrixHead;
	Point eye = Point(-len,0,0);
	// eye = Point(-433,0,250);
	rotationMatrixHead.computeRotationMatrix(rot /*Rotation(0,0,60) */);
	rotationMatrixHead.rotatePoint(eye);
	return eye;
}

void Kinematics::limitPosition(Position &pos) {
	Position savePos(pos);
	// bounding box around the null position
	pos.point.x = constrain(pos.point.x,-420,-200);
	pos.point.y = constrain(pos.point.y,0,450);
	pos.point.z = constrain(pos.point.z,-420,+400);
	if (pos != savePos) {
		pos.println("WARN: abs position limited");
	}

	// check if coordinates are within a segment of a ball of
	// radius LEG_LENGTH+ARM_LENGTH+NECK_LENGTH
	// if not, limit the angles and reduce the distance between base and eye accordingly
	float yAngle  = -degrees(atan2Fast(pos.point.z,-pos.point.x));
	float zAngle = degrees(atan2Fast(pos.point.y,sqrt(pos.point.x*pos.point.x + pos.point.z*pos.point.z)));
	float yAngleNew  = yAngle;
	float zAngleNew = zAngle;

	bool computeBallCoord = false;
	if ((yAngle<-50) || (yAngle>50)) {
		yAngleNew = constrain(yAngle,-50,50);
		computeBallCoord = true;
	}
	if ((zAngle<5) || (zAngle>85)) {
		zAngleNew = constrain(zAngle,5,85);
		computeBallCoord = true;
	}
	if (computeBallCoord) {
		Matrix3x3 rotationMatrixHead;
		rotationMatrixHead.computeRotationMatrix(Rotation(0,yAngleNew,zAngleNew));
		float len = pos.point.distance(Point(0,0,0));
		len = constrain(len,150, LEG_LENGTH+ARM_LENGTH+NECK_LENGTH);
		Point rotatePoint (-len,0,0);
		Point rotatePointBak (rotatePoint);

		rotationMatrixHead.rotatePoint(rotatePoint);
		cerr << "WARN: angle(y,z)=(" << yAngle << "," << zAngle << ") limited to (" << yAngleNew << "," << zAngleNew << ") "
			 << "pos=" << pos.point << " limited to " << rotatePoint << endl;
		pos.point = rotatePoint;

	}
}

Position Kinematics::getSpeed(const TimedPosition& pos1, const TimedPosition& pos2) {
	static int i = 0;
	i++;
	if (i == 3) {
		cout << "x";
	}
	float timeDiff = timediff(pos2.atTime,pos1.atTime);
	Position speedVector = (pos2.pos - pos1.pos) / timeDiff;

	// apply a low pass filter over 30ms
	static Position speedVectorFilter;
	static float lowPassTimeConstant = SERVO_LOOP_TIME_MS; // [ms]
	static float lowPassFilterFactor = (lowPassTimeConstant/(lowPassTimeConstant+((float)SERVO_LOOP_TIME_MS)));
	speedVectorFilter = speedVectorFilter*lowPassFilterFactor + speedVector*(1.0-lowPassFilterFactor);

	return speedVectorFilter;
}

Position Kinematics::getAcceleration(const TimedPosition& nextPos) {

	// predict the position along the current speed vector assuming if we did not change anything
	Position predictPosition(servoPosition.pos);
	Position predictedTranslation(speedVector);
	predictedTranslation *= (nextPos.atTime-servoPosition.atTime);
	predictPosition += predictedTranslation;

	// compute the acceleration to the Position we actually want to reach [mm/ms^2]
	int dT = nextPos.atTime - servoPosition.atTime;
	Position acceleration = (nextPos.pos - predictPosition) * (2.0 / ( dT*dT ));
	return acceleration;
}

// limits the acceleration in caartesic coord
void Kinematics::limitAcceleration(TimedPosition& nextPos) {
	// predict the position along the current speed vector assuming if we did not change anything
	// this predicted position happens with no acceleration
	Position predictPosition = servoPosition.pos + speedVector*(nextPos.atTime-servoPosition.atTime);

	// compute the necessary acceleration to the position we actually want to reach [mm/ms^2]
	// s = 0.5*a*t*t -> a = 2*s/(t*t)
	int dT = nextPos.atTime - servoPosition.atTime;
	Position acc = (nextPos.pos - predictPosition) * (2.0 / ( dT*dT )); // [mm/ms^2]

	// check all dimensions for maximum acceleration
	const float dimAccLimit = 0.4*9.81*1000.0/(1000.0*1000.0); 		// allowed acceleration for x,y,z is 0.4g
	const float accLimitx = dimAccLimit;
	const float accLimity = dimAccLimit;
	const float accLimitz = dimAccLimit/2;							// movements in z are sensitive to osscillation, so half the allowed acceleration

	const float rotAccLimit= 100.0*2.0*M_PI/360.0*dimAccLimit; 		// allowed acceleration for rot
																	// assume distance to cog of 100mm
	const float accLimitRoty = rotAccLimit;
	const float accLimitRotz = rotAccLimit;

	// compute the allowed way in the intended direction that does
	// not exceed the maximum acceleration in any dimension.
	// Ratio gives the allowed part of the translation between
	// predicted position and to-be position. If acceleration is ok
	// ratio is 1.0, otherwise it is smaller.
	float ratio = 1.0; // allowed ratio between predicted and to-be position
	ratio = max(ratio, absFloat(acc.point.x) / accLimitx);
	ratio = max(ratio, absFloat(acc.point.y) / accLimity);
	ratio = max(ratio, absFloat(acc.point.z) / accLimitz);

	ratio = max(ratio, absFloat(acc.rot.y) / accLimitRoty);
	ratio = max(ratio, absFloat(acc.rot.z) / accLimitRotz);

	ratio = min(1.0, 1.0/ratio);

	// do we have to adapt the to-be position since it exceeds the maximum acceleration?
	if (ratio< 1.0) {
		// difference between current position and to-be position is reduced to allows acceleration
		Position limitedTranslation = predictPosition + (nextPos.pos-predictPosition)*ratio;
		Position formerPosition(nextPos.pos);
		nextPos.pos = limitedTranslation;
		if (debugKinematic) {
			nextPos.pos.print("limited acc");
			formerPosition.print("instead");
			speedVector.println("speed");
		}
	}
}

bool Kinematics::moveServosTo(const Position& eye) {
	unsigned long now = millis();
	if (debugKinematic)
		eye.println("eye");

	// store last position for speed/acceleration
	speedVector = getSpeed(servoPosition, TimedPosition(now,eye));

	lastServoPosition = servoPosition;
	servoPosition.pos = eye;
	servoPosition.atTime = now;

	// store a short history of the passed tensors
	addToHistory(TimedPosition(now,eye));

	bool validated = true;

	// compute the position of the head by taking the eye position, and rotation the relative
	// vector to the head by the eye orientation
	Point headFromEye (EYE_LENGTH, 0, 0);
	Matrix3x3 rotationMatrixHead;
	rotationMatrixHead.computeRotationMatrix(eye.rot);
	if (debugKinematic)
		headFromEye.print(std::string("head-from eye "));
	rotationMatrixHead.rotatePoint(headFromEye);
	if (debugKinematic) {
		eye.rot.print(" rotated around");
		headFromEye.print(" gives");
	}
	Point headPosition = eye.point;
	if (debugKinematic)
		headPosition.print((const string&)". head-pos ");
	headPosition.translate (headFromEye);
	if (debugKinematic)
		headPosition.println(" translates to ");

	// now compute the angle of the base
	float baseTurnAngle = degrees(atan2Fast(-headPosition.z, -headPosition.x  ));

	// compute the rotation of the neck out of head's orientation and the base angle
	float neckTurnAngle = eye.rot.y - baseTurnAngle;
	float headNickAngle = eye.rot.z;

	if (debugKinematic)
		cout << "base-turn=" << baseTurnAngle << "°" << " neck-turn=" << neckTurnAngle << "° neck-nick=" << headNickAngle << "°" << endl;

	// now let's go to the neck. Out of the head position
	// compute the position of the neck relative to the head, rotate
	// around z (orientation of head) and y (neckTurnAngle)
	Point neckFromHead (+NECK_LENGTH+SHOULDER_LENGTH, -NECK_HEIGHT,0);
	Rotation neckOrientation (0,baseTurnAngle, eye.rot.z);

	Matrix3x3 rotationNeck;
	rotationNeck.computeRotationMatrix(neckOrientation);
	// if (debugKinematic) {
	// 	neckFromHead.print("neck-from-head ");
	// 	neckOrientation.print(" rotated around");
	// }
	rotationNeck.rotatePoint(neckFromHead);
	// if (debugKinematic)
	// 	neckFromHead.print(" gives");
	Point neckPosition = headPosition;
	// if (debugKinematic)
	// 	neckPosition.print(". neck-pos");
	neckPosition.translate(neckFromHead);
	// if (debugKinematic)
	// 	neckPosition.println(" translates to");

	// rotate the foot point around the base in order to get the position of the foot
	Point footPosition( -FOOT_LENGTH, BASE_HEIGHT,0.0);
	// if (debugKinematic)
	// 	footPosition.println("foot-before-rot");
	Rotation baseOrientation(0.0,baseTurnAngle,0.0);
	Matrix3x3 rotationMatrixBase;
	rotationMatrixBase.computeRotationMatrix(baseOrientation);
	rotationMatrixBase.rotatePoint(footPosition);
	// if (debugKinematic)
	// 	footPosition.println("foot-after-rot");

	// compute angle of leg and arm with cosine law c*c = a*a + b*b - 2*a*b*cos (angle-at-c)
	// which gives angle-at_c = acos((c*c - a*a - b*b)/(2*a*b))
	float distance = neckPosition.distance(footPosition); // that's "c" in the cosine law, a = LEG_LENGTH b = ARM_LENGTH

	// check if triangle is valid, otherwise go to full length position
	// use consinus law for this as well by patching distance
	if (distance> (LEG_LENGTH+ARM_LENGTH)) {
		cerr << "ERR:kinematic triangle exceeded d=" << distance << " ";
		distance = LEG_LENGTH+ARM_LENGTH;
		eye.println();
	}

	float distanceSqr = distance*distance;
	float triangleAngleLeg = degrees(acosFast((distanceSqr + (LEG_LENGTH*LEG_LENGTH - ARM_LENGTH*ARM_LENGTH))/((2*LEG_LENGTH)*distance)));
	float triangleAngleArm = degrees(acosFast((ARM_LENGTH*ARM_LENGTH+LEG_LENGTH*LEG_LENGTH-distanceSqr)/(2*ARM_LENGTH*LEG_LENGTH)));
	float legAngle = triangleAngleLeg - degrees(acosFast((neckPosition.y-footPosition.y)/distance)) ;
	float armAngle = triangleAngleArm-90+legAngle;
	if (debugKinematic)
		cout << fixed << std::setprecision(1) << " triangle dist=" << distance << " arm=" << triangleAngleArm<< "° triangle leg=" << triangleAngleLeg << "°"
			 << " leg-angle=" << legAngle << "° arm-angle=" << armAngle << "°" << endl
			 << " res=(" << baseTurnAngle << "," << legAngle << "," << armAngle << "," << headNickAngle << "," << neckTurnAngle << ")" << endl;

	// the neck needs to be restricted for positions close to the base
	float maxHeadNickAngle = min(45. +(armAngle+45),45.);
	if (headNickAngle > maxHeadNickAngle) {
		cerr << "WARN: HeadNickAngle " << headNickAngle << " reduced to " << maxHeadNickAngle<< endl;
		headNickAngle = constrain(headNickAngle, -80,maxHeadNickAngle);
	}

	// the arm needs to be restricted for positions close to the base
	float armMin = -65;
	if ((armAngle< armMin)) {
		cerr << "WARN: armAngle " << armAngle << "<" << armMin << ", reduced to " << armMin << endl;
		armAngle = armMin;
	}

	// now set the servos
	ServoController& servoController = ServoController::getInstance();
	bool ok1 = servoController.getServo(SERVO_BASE_TURN).setAngle(baseTurnAngle);
	bool ok2 = servoController.getServo(SERVO_LEG).setAngle(legAngle);
	bool ok3 = servoController.getServo(SERVO_ARM).setAngle(armAngle);
	bool ok4 = servoController.getServo(SERVO_HEAD_NICK).setAngle(headNickAngle);
	bool ok5 = servoController.getServo(SERVO_HEAD_TURN).setAngle(neckTurnAngle);
	if (!(ok1 && ok2 && ok3 && ok4 && ok5)) {
		eye.print("wrong servo setting for ");
	}

	servoController.setServoData();
	/*
	if (debugKinematic) {
		servoController.print();
		cout << "t=" << millis()-now << "ms" << endl;
	}
	*/
	return validated;
}

Position Kinematics::getManualPosition() {
	return manualPosition;
}

Position Kinematics::getCurrentPosition() {
	return servoPosition.pos;
}

void Kinematics::printMenu() {
	cout << "Kinematics" << endl
		<< "p     - power " << string(MicroControllerInterface::getInstance().getPowerOn()?"off":"on") << endl
		<< "x/y/z - set eye's position" << endl
		<< "Y/Z   - set eye's orientation" << endl
		<< "+/-   - position" << endl
		<< "*/_   - more position" << endl
		<< "r     - realtime movement" << endl

		<< "c     - set kinematics" << endl;

	printManualPosition();
}

void Kinematics::printManualPosition() {
	manualPosition.println("current pos");
}

void Kinematics::callMenu() {
	while (true) {
		char c = keyPressed();

		TrajectoryController::getInstance().loop(); // send changed servo values to servos

		if ((c != 0) && (c != EOF) && (c != '\n')){
			switch (c) {
				case 'h':
					printMenu();
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
				case 'p': {
					bool powerOn = MicroControllerInterface::getInstance().getPowerOn();
					cout << "switch servo power " << (powerOn?"off":"on")<< endl;
					ServoController::getInstance().setServoData();
					MicroControllerInterface::getInstance().power(!powerOn);
					break;
				}
				case '+':
				case '-':
				case '*':
				case '_': {
					float inc = 0;
					if (c == '+') inc = 1.0;
					if (c == '-') inc = -1.0;
					if (c == '*') inc = 3.0;
					if (c == '_') inc = -3.0;

					increaseManualPosition(positionAxis,inc);
					if (moveDirectly)
						moveServosTo(manualPosition);
					break;
				}
				case 'r':
					moveDirectly = !moveDirectly;
					if (moveDirectly)
						cout << "direct movement" << endl;
					else
						cout << "no movement with position setting" << endl;
					break;
				case 'c':
					moveServosTo(manualPosition);
					break;
				case '\e':
					return;

				default:
				break;
			} // switch input char
		} // if (Serial.available)
	}
}

