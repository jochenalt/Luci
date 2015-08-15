
#include "Util.h"

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include "math.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>

bool useFastMath = false; // strange enough, is actually not faster than built-in trigonometry
bool debug = true;
bool kinmeaticDebug = true;

float absFloat(float x) {
	return (x>0.0)?x:-x;
}
int absInt(int x) {
	return (x>0)?x:-x;
}

float signFloat(float x, float limit) {
	if (abs(x)<limit)
		return randomPosNeg();
	return (x>0.0)?1.0:-1.0;
}


int timediff(unsigned long a, unsigned long b) {
	if (a>b)
		return a-b;
	else
		return b-a;
}

long mapLong(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int randomInt(int min,int max) {
	int r = std::rand()>>16;
	int result = mapLong(r,0,(RAND_MAX>>16)-1,min,max+1);
	return result;
}

float randomFloat (float a, float b) {
	return randomInt(a*1000, b*1000)/1000.;
}


bool randomBool() {
	return randomInt(0,100)>50;
}

int randomPosNeg() {
	return (randomInt(0,100)>50)?+1:-1;
}


static struct termios oldSettings, newSettings;

void prepareReadKey() {

   tcgetattr( fileno( stdin ), &oldSettings );
   newSettings = oldSettings;
   newSettings.c_lflag &= (~ICANON & ~ECHO);
   tcsetattr( fileno( stdin ), TCSANOW, &newSettings );
}

char keyPressed() {
       fd_set set;
       struct timeval tv;

       tv.tv_sec = 0;
       tv.tv_usec = 10;

       FD_ZERO( &set );
       FD_SET( fileno( stdin ), &set );

       int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );

       if( res > 0 )
       {
           char c;
           read( fileno( stdin ), &c, 1 );
           return c;
       }
       else if( res < 0 )
       {
           return 0;
       }
       else
       {
           return 0;
       }
       return 0;
}


void shutdownKey() {
   tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
}


float radians(float degree) {
	return degree*(M_PI/180.0);
}
float degrees(float radians) {
	return radians*(180.0/M_PI);
}


long timeOffset = 0;
unsigned long stopedTime = 0;


void stopTime() {
	stopedTime = millis();
}

void startTime() {
	stopedTime = 0;
}

unsigned long realMillis() {
	boost::posix_time::ptime current_date_microseconds = boost::posix_time::microsec_clock::local_time();
    unsigned long milliseconds = current_date_microseconds.time_of_day().total_milliseconds();

    static unsigned long startTime = 0;
    if (startTime == 0) {
    	startTime = milliseconds;
    }
    unsigned long ret = milliseconds -startTime;
    return ret;
}

void setTime(unsigned long time) {
	timeOffset = realMillis()-time;
	if (stopedTime != 0)
		stopedTime = time;
}

unsigned long millis() {
	if (stopedTime != 0)
		return stopedTime;
	return realMillis()-timeOffset;
}


// return a number 0..1 representing the position of t between a and b
float intervalRatio(unsigned long a, unsigned long t, unsigned long b) {
	return ((float)t-(float)a)/((float)b-(float)a);
}

// With 10 points we have a maximum error of 0.37%. Seems to be ok.
#define SUPPORT_POINTS 21
const float sinSupportArray[SUPPORT_POINTS] = { 0,            // 0°
												(float)sin(radians(1*90/(SUPPORT_POINTS-1))),   //  9°
												(float)sin(radians(2*90/(SUPPORT_POINTS-1))),   // 18°
												(float)sin(radians(3*90/(SUPPORT_POINTS-1))),   // 27°
												(float)sin(radians(4*90/(SUPPORT_POINTS-1))),   // 36°
												(float)sin(radians(5*90/(SUPPORT_POINTS-1))),   // 45°
												(float)sin(radians(6*90/(SUPPORT_POINTS-1))),   // 54°
												(float)sin(radians(7*90/(SUPPORT_POINTS-1))),   // 63°
												(float)sin(radians(8*90/(SUPPORT_POINTS-1))),   // 72°
												(float)sin(radians(9*90/(SUPPORT_POINTS-1))),   // 81°
												(float)sin(radians(10*90/(SUPPORT_POINTS-1))),  // 90°
												(float)sin(radians(11*90/(SUPPORT_POINTS-1))),   //  9°
												(float)sin(radians(12*90/(SUPPORT_POINTS-1))),   // 18°
												(float)sin(radians(13*90/(SUPPORT_POINTS-1))),   // 27°
												(float)sin(radians(14*90/(SUPPORT_POINTS-1))),   // 36°
												(float)sin(radians(15*90/(SUPPORT_POINTS-1))),   // 45°
												(float)sin(radians(16*90/(SUPPORT_POINTS-1))),   // 54°
												(float)sin(radians(17*90/(SUPPORT_POINTS-1))),   // 63°
												(float)sin(radians(18*90/(SUPPORT_POINTS-1))),   // 72°
												(float)sin(radians(19*90/(SUPPORT_POINTS-1))),   // 81°
												(float)sin(radians(20*90/(SUPPORT_POINTS-1))),  // 90°
};


const float asinSupportArray[SUPPORT_POINTS] = { 0,
												 asin(1*1.0/(SUPPORT_POINTS-1)),
												 asin(2*1.0/(SUPPORT_POINTS-1)),
												 asin(3*1.0/(SUPPORT_POINTS-1)),
												 asin(4*1.0/(SUPPORT_POINTS-1)),
												 asin(5*1.0/(SUPPORT_POINTS-1)),
												 asin(6*1.0/(SUPPORT_POINTS-1)),
												 asin(7*1.0/(SUPPORT_POINTS-1)),
												 asin(8*1.0/(SUPPORT_POINTS-1)),
												 asin(9*1.0/(SUPPORT_POINTS-1)),
												 asin(10*1.0/(SUPPORT_POINTS-1)),
												 asin(11*1.0/(SUPPORT_POINTS-1)),
												 asin(12*1.0/(SUPPORT_POINTS-1)),
												 asin(13*1.0/(SUPPORT_POINTS-1)),
												 asin(14*1.0/(SUPPORT_POINTS-1)),
												 asin(15*1.0/(SUPPORT_POINTS-1)),
												 asin(16*1.0/(SUPPORT_POINTS-1)),
												 asin(17*1.0/(SUPPORT_POINTS-1)),
												 asin(18*1.0/(SUPPORT_POINTS-1)),
												 asin(19*1.0/(SUPPORT_POINTS-1)),
												 M_PI_2
												};

const float acosSupportArray[SUPPORT_POINTS] = { 0,
	acos(1*1.0/(SUPPORT_POINTS-1)),
	acos(2*1.0/(SUPPORT_POINTS-1)),
	acos(3*1.0/(SUPPORT_POINTS-1)),
	acos(4*1.0/(SUPPORT_POINTS-1)),
	acos(5*1.0/(SUPPORT_POINTS-1)),
	acos(6*1.0/(SUPPORT_POINTS-1)),
	acos(7*1.0/(SUPPORT_POINTS-1)),
	acos(8*1.0/(SUPPORT_POINTS-1)),
	acos(9*1.0/(SUPPORT_POINTS-1)),
	acos(10*1.0/(SUPPORT_POINTS-1)),
};


float interpolate (const float supportPoints[], float rezRangePerInterval, float x)
{
	float xByWidth = x*rezRangePerInterval;
	int leftBoundary = int(xByWidth); // index of left support point
	float ratioWithinInterval = xByWidth-leftBoundary; // ratio of x within bot/top interval. x=bot-> t = 0.0, x=top->t = 1.0
	float leftValue = supportPoints[leftBoundary];
	float rightValue = supportPoints[leftBoundary+1];
	return leftValue + ratioWithinInterval * (rightValue - leftValue);
}

float asinFast(float x) {
	if (!useFastMath)
		return asin(x);

	if (x<0.0)
		return -asinFast(-x);

	float result = interpolate(asinSupportArray, (SUPPORT_POINTS-1) / 1.0, x);
	return result ;
}

float acosFast(float x) {
	if (!useFastMath)
		return acos(x);

	if (x<0.0)
		return M_PI_2-acosFast(x);

	float result = interpolate(acosSupportArray, (SUPPORT_POINTS-1) / 1.0, x);
	return result ;
}

float sinFast(float x) {
	if (!useFastMath)
		return sin(x);


	if (x<0.0)
		return -sinFast(-x);

	if (x>M_PI*2.0)
		return sinFast(x-M_PI*2.0);

	if (x>M_PI)
		return -sinFast(x-M_PI);

	if (x> M_PI_2)
		return sinFast(M_PI-x);

	float result = interpolate(sinSupportArray, ((float)SUPPORT_POINTS-1)/M_PI_2 , x);
	return result ;
}


float cosFast(float x) {
	return sinFast(x+M_PI/2);
}


// compute atanfast by approximately as desribed in wikipedia
// max. deviation <0.005 rad
float atanFast(float x) {
	if (!useFastMath)
		return atan(x);

	if (x> 1.0)
		return (M_PI/2.0) - (x/(x*x+0.28));
	if (x<-1.0)
		return (-M_PI/2.0) - (x/(x*x+0.28));

	return (x/(1.0+ 0.28*x*x));
}


float atan2Fast(float y, float x) {
	if (!useFastMath)
		return atan2(y,x);

	if (x > 0.0)
		return atanFast(y/x);

	if (x == 0.0) {
		if (y>0.0)
			return (M_PI/2.0);
		if (y<0.0)
			return (-M_PI/2.0);
		return 0.0; // actually this is undefined
	}

	// x < 0.0
	if (y>= 0.0)
		return atanFast(y/x)+M_PI;

	return atanFast(y/x)-M_PI;
}

void Printable::setDebug(bool doDebug) {
	debugOutput = doDebug;
}

void Printable::println() const {
		print();
		if (debugOutput)
			std::cout << std::endl;
}

void Printable::print(const std::string& msg) const  {
		if (debugOutput)
			std::cout << msg << "=";
		print();
}

void Printable::println(const std::string& msg) const {
		if (debugOutput)
			std::cout << msg << "=";
		println();
}
