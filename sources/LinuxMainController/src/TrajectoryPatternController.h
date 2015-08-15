/*
 * TrajectoryPatternController.h
 *
 *  Created on: 06.06.2015
 *      Author: JochenAlt
 */

#ifndef SRC_TRAJECTORYPATTERNCONTROLLER_H_
#define SRC_TRAJECTORYPATTERNCONTROLLER_H_

#include "Kinematic.h"
#include "Util.h"
#include "TrajectoryPattern.h"


extern bool debugTrajectory;
class TrajectoryPatternController {
public:
	TrajectoryPatternController () {
		currentPattern = NULL;
		patternStartTime = 0;
		startPatternWithNextCall = true;
	}

	void printMenu();
	void callMenu();
	void test1();
	void test2();


	TrajectoryPatternType getType();

	// returns true, if current pattern has been fetched completely (not necessarily carried out yet)
	bool isDone() {
		if (currentPattern != NULL)
			return currentPattern->isDone();
		return true;
	}

	unsigned long getStartTime() {
		return patternStartTime;
	}
	TimedPosition fetchNextPosition(TimedPosition lastPosition);

	static TrajectoryPatternController& getInstance() {
		static TrajectoryPatternController instance;
		return instance;
	};
	void setPattern(TrajectoryPatternType patternType);
	void setup() {};
	void loop();
	void changePatternAsAppropriate();
	void runPattern(unsigned long duration_ms);


private:
	TrajectoryPatternImpl* currentPattern;

	SoBoredPattern soBoredPattern;
	FaceInteractionPattern faceInteractionPattern;
	DepressionPattern depressionPattern;
	WakeUpPattern wakeupPattern;
	NaughtyLampPattern naughtyLampPattern;

	unsigned long patternStartTime;
	bool startPatternWithNextCall;
	static TrajectoryPatternController sInstance;

};

#endif /* SRC_TRAJECTORYPATTERNCONTROLLER_H_ */
