#ifndef UTIL_H_
#define UTIL_H_

#include <iostream>
#include <string>

using namespace std;

int timediff(unsigned long a, unsigned long b);
float intervalRatio(unsigned long a, unsigned long t,unsigned long b);

long mapLong(long x, long in_min, long in_max, long out_min, long out_max);
float absFloat(float x);
int absInt(int x);
float signFloat(float x, float limit);


float radians(float degree);
float degrees(float radians);

unsigned long millis();
void setTime(unsigned long time);
void stopTime();
void startTime();

float atan2Fast(float y, float x);
float atanFast(float x);
float cosFast(float x);
float sinFast(float x);
float acosFast(float x);

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
extern bool debug;
extern bool useFastMath;

void prepareReadKey();
char keyPressed();
void shutdownKey();

int randomInt(int min,int max);
float randomFloat (float a, float b);

bool randomBool();
int randomPosNeg();

class Printable {
private:
	bool debugOutput;
public:
	Printable() {
		debugOutput = true;
	}
	virtual ~Printable() {};

	void setDebug(bool doDebug);

	virtual void print() const = 0;
	virtual void println() const;
	virtual void print(const std::string& msg) const;
	virtual void println(const std::string& msg) const;

};


// small helper class to measure the passed time since an event and to check
// whether something that is supposed to run at a specific time (due_ms/due_us)
// use:
//     TimePassedBy timer(MS);						// initialize timer that is supposed to execute something periodically
//     int16_t passed_ms;
//     while (true) {								// to be used in a loop
//			<do what you like>
//			if (timer.due_ms(200, passed_ms))		// check whether 200ms have been passed and
//				<do something that has to be called every 200ms>
//		}
//
class TimePassedBy {
	public:
	// initialize this timer to work
	TimePassedBy () {
		mLastCall_ms = millis();
		now = mLastCall_ms;
	}
	// true, if at least <ms> milliseconds passed since last invocation that returned true.
	// returns the actual passed time in addition
	bool isDue_ms(int ms, int &passed_ms) {
		unsigned long now = millis();
		passed_ms = now-mLastCall_ms;
		if (passed_ms>=ms) {
			mLastCall_ms = now;
			return true;
		}
		return false;
	}
	void setDueTime (int due_ms) {
		mLastCall_ms = millis()-due_ms;
	}

	bool isDue_ms(int  ms) {
		int passed_ms;
		return isDue_ms(ms, passed_ms);
	}

	unsigned long mLastCall_ms;	// last due time in milliseconds
	unsigned long now;		// current time
};

#endif
