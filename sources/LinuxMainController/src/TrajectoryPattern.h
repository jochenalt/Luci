/*
 * TrajectoryPattern.h
 *
 *  Created on: 09.04.2015
 *      Author: JochenAlt
 */

#ifndef TRAJECTORY_PATTERN_H_
#define TRAJECTORY_PATTERN_H_

#include "Kinematic.h"
#include "Util.h"


enum TrajectoryPatternType { NO_PATTERN,SO_BORED, FACE_INTERACTION, DEPRESSION_PATTERN, WAKEUP_PATTERN, NAUGHTYLAMP_PATTERN };


class TrajectoryPatternImpl {
	friend class TrajectoryPatternController;
public:
	virtual ~TrajectoryPatternImpl() {};
	TrajectoryPatternImpl(int firstPhase) {
		lastPosition.pos = Kinematics::getInstance().getCurrentPosition();
		lastPosition.atTime = millis();
		currentPosition = lastPosition;
		startPhase = firstPhase;
		phaseInt = 0;
		subPhaseCounter = 0;
		subPhaseData = 0;
		lookAt = Kinematics::getInstance().getNullPosition().point;
		lookAt.x = -1000;

	}

	virtual TimedPosition fetchNextPosition( const TimedPosition& lastPosition) = 0;
	virtual TrajectoryPatternType getType() = 0;
	virtual bool isDone() { return false; };

	void setPhase(int pPhase) {	phaseInt = pPhase; subPhaseCounter = 0;};
	bool isPhase(int pPhase) {	return phaseInt == pPhase; };
	void init();
	virtual string getPhaseName()= 0;
	virtual string getPatternName()= 0;
	Point& getLookAt() { return lookAt; };

	int getPhase() { return phaseInt; };
	void setSubPhase(int pSubPhaseCounter) {
		subPhaseCounter = pSubPhaseCounter;
	}
	void setSubPhase(int pSubPhaseCounter, int x) {
		subPhaseCounter = pSubPhaseCounter;
		subPhaseData = x;
	}
	void setSubPhase(int pSubPhaseCounter, const Point & p) {
		subPhaseCounter = pSubPhaseCounter;
		subPhasePoint  = p;
	}

	void setOrDecSubPhase(int maxPhaseNo, int nextPhase) {
		if (getSubPhase() == 0) {
			setSubPhase(maxPhaseNo);
		} else {
			decSubPhase();
			if (getSubPhase() == 0)
				setPhase(nextPhase);
		}
	}


	void decSubPhase() {subPhaseCounter--;};
	int getSubPhaseData() { return subPhaseData; };
	Point& getSubPhasePoint() { return subPhasePoint; };

	int getSubPhase() { return subPhaseCounter; };
protected:
	TimedPosition lastPosition;
	TimedPosition currentPosition;
	int  phaseInt;
	int subPhaseCounter;
	int subPhaseData;
	Point subPhasePoint;
	int startPhase;
	static Point lookAt;
}; // TrajectoryPattern

// SoBoredPattern is a bored movement like looking around
// pattern runs forever, it will never stop delivering movements
class SoBoredPattern : public TrajectoryPatternImpl {
public:
	SoBoredPattern() : TrajectoryPatternImpl (MOVE_TO_NEW_POSITION){
	};
	virtual ~SoBoredPattern() {};

	virtual TimedPosition fetchNextPosition(const TimedPosition& lastPosition);
	TrajectoryPatternType getType() { return SO_BORED; };
	virtual string getPatternName() { return "SoBoredPattern"; };

	virtual string getPhaseName()  {
		switch (getType()) {
		case LOOKING_AT: return "LOOKING_AT";
		case MOVE_TO_NEW_POSITION: return "MOVE_TO_NEW_POSITION";
		default:
			return "";
		}
	}

	Point& getScanForFacePosition();
private:
	enum phaseType { MOVE_TO_NEW_POSITION, LOOKING_AT  };
	Point mainPosition;
	Point scanPosition;

}; // SoBoredPattern

class FaceInteractionPattern : public TrajectoryPatternImpl {
public:
	FaceInteractionPattern():TrajectoryPatternImpl(RECOGNIZE_FACE) {
	};
	virtual ~FaceInteractionPattern() {};
	TrajectoryPatternType getType() { return FACE_INTERACTION; };
	virtual TimedPosition fetchNextPosition(const TimedPosition& lastPosition);
	virtual string getPatternName() { return "FaceInteractionPattern"; };
	virtual string getPhaseName()  {
				switch (getPhase()) {
				case RECOGNIZE_FACE: return "RECOGNIZE_FACE";
				case SURPRISED_LOOK: return "SURPRISED_LOOK";
				case SLOW_APPROACH: return "SLOW_APPROACH";
				case QUICK_MOVE_TO_CORNER: return "QUICK_MOVE_TO_CORNER";
				case SLOW_MOVE_AROUND_FACE: return "SLOW_MOVE_AROUND_FACE";
				case NODDING_MOVE: return "NODDING_MOVE";

				default:
					return "";
				}
			}
private:
	enum phaseType { RECOGNIZE_FACE, SURPRISED_LOOK, SLOW_APPROACH, QUICK_MOVE_TO_CORNER, SLOW_MOVE_AROUND_FACE, NODDING_MOVE};
};

class WakeUpPattern : public TrajectoryPatternImpl {
public:
	WakeUpPattern():TrajectoryPatternImpl(SLEEP) {
	};
	virtual ~WakeUpPattern() {};
	TrajectoryPatternType getType() { return WAKEUP_PATTERN; };
	virtual bool isDone() { return (getPhase() == DONE); };
	virtual TimedPosition fetchNextPosition(const TimedPosition& lastPosition);
	virtual string getPatternName() { return "WakeUpPattern"; };

	string getPhaseName()  {
			switch (getPhase()) {
			case SLEEP: return "SLEEP";
			case HEAD_UP: return "HEAD_UP";
			case BLINK: return "BLINK";
			case LOOK_AROUND: return "LOOK_AROUND";
			case DONE: return "DONE";

			default:
				return "";
			}
		}
private:
	enum phaseType { SLEEP, HEAD_UP, BLINK, LOOK_AROUND, DONE};
};

class NaughtyLampPattern : public TrajectoryPatternImpl {
public:
	NaughtyLampPattern():TrajectoryPatternImpl(SLEEP) {
	};
	virtual  ~NaughtyLampPattern() {};
	TrajectoryPatternType getType() { return NAUGHTYLAMP_PATTERN; };
	virtual bool isDone() { return false;};
	virtual TimedPosition fetchNextPosition(const TimedPosition& lastPosition);
	virtual string getPatternName() { return "NaughtyLampPattern"; };

	virtual string getPhaseName()  {
			switch (getPhase()) {
			case SLEEP: return "SLEEP";
			case WATCH_CAREFULLY: return "WATCH_CAREFULLY";
			case BLINK: return "BLINK";
			case PUSH_BOX: return "PUSH_BOX";
			case CHECK: return "CHECK";
			case WATCH: return "WATCH";
			case WATCH_INTENSIVELY: return "WATCH_INTENSIVELY";
			case DONE: return "DONE";

			default:
				return "";
			}
		}
	private:
		enum phaseType { SLEEP, WATCH_CAREFULLY, BLINK, PUSH_BOX, CHECK, WATCH,WATCH_INTENSIVELY, DONE};
};


class DepressionPattern : public TrajectoryPatternImpl {
public:
	DepressionPattern():TrajectoryPatternImpl(MOVE_TO_SORROW_POSITION) {
	};
	virtual ~DepressionPattern() {};
	TrajectoryPatternType getType() { return DEPRESSION_PATTERN; };
	virtual bool isDone() { return (getPhase() == DONE); };
	virtual TimedPosition fetchNextPosition(const TimedPosition& lastPosition);
	enum phaseType { MOVE_TO_SORROW_POSITION, SORROW_BREAK, BREATH_IN, DEPRESSION_POSITION, DEPRESSION_HIGH, DEEP_DEPRESSION_POSITION, DONE};
	virtual string getPatternName() { return "DepressionPattern"; };

	virtual string getPhaseName()  {
			switch (getPhase()) {
			case MOVE_TO_SORROW_POSITION: return "MOVE_TO_SORROW_POSITION";
			case SORROW_BREAK: return "SORROW_BREAK";
			case BREATH_IN: return "BREATH_IN";
			case DEPRESSION_POSITION: return "DEPRESSION_POSITION";
			case DEPRESSION_HIGH: return "DEPRESSION_HIGH";
			case DEEP_DEPRESSION_POSITION: return "DEEP_DEPRESSION_POSITION";
			case DONE: return "DONE";

			default:
				return "";
			}
		}

};


#endif /* TRAJECTORY_PATTERN_H_ */
