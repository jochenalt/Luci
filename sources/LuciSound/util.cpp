#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>

#include "Util.h"

bool useFastMath = false;

float sqrFloat(float x) {
	return x*x;
}

int sqrInt(int x) {
	return x*x;
}


int timediff(unsigned long a, unsigned long b) {
	if (a>b)
		return a-b;
	else
		return b-a;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float radians(float degree) {
	return degree*(M_PI/180.0);
}
float degrees(float radians) {
	return radians*(180.0/M_PI);
}

#define SUPPORT_POINTS 21
const float sinSupportArray[SUPPORT_POINTS] = { 0,            // 0°
												(float)sin(radians(1.*90/(SUPPORT_POINTS-1))),   //  9°
												(float)sin(radians(2.*90/(SUPPORT_POINTS-1))),   // 18°
												(float)sin(radians(3.*90/(SUPPORT_POINTS-1))),   // 27°
												(float)sin(radians(4.*90/(SUPPORT_POINTS-1))),   // 36°
												(float)sin(radians(5.*90/(SUPPORT_POINTS-1))),   // 45°
												(float)sin(radians(6.*90/(SUPPORT_POINTS-1))),   // 54°
												(float)sin(radians(7.*90/(SUPPORT_POINTS-1))),   // 63°
												(float)sin(radians(8.*90/(SUPPORT_POINTS-1))),   // 72°
												(float)sin(radians(9.*90/(SUPPORT_POINTS-1))),   // 81°
												(float)sin(radians(10.*90/(SUPPORT_POINTS-1))),  // 90°
												(float)sin(radians(11.*90/(SUPPORT_POINTS-1))),   //  9°
												(float)sin(radians(12.*90/(SUPPORT_POINTS-1))),   // 18°
												(float)sin(radians(13.*90/(SUPPORT_POINTS-1))),   // 27°
												(float)sin(radians(14.*90/(SUPPORT_POINTS-1))),   // 36°
												(float)sin(radians(15.*90/(SUPPORT_POINTS-1))),   // 45°
												(float)sin(radians(16.*90/(SUPPORT_POINTS-1))),   // 54°
												(float)sin(radians(17.*90/(SUPPORT_POINTS-1))),   // 63°
												(float)sin(radians(18.*90/(SUPPORT_POINTS-1))),   // 72°
												(float)sin(radians(19.*90/(SUPPORT_POINTS-1))),   // 81°
												(float)sin(radians(20.*90/(SUPPORT_POINTS-1))),  // 90°
};


const float asinSupportArray[SUPPORT_POINTS] = { 0,
												 asin(1.*1.0/(SUPPORT_POINTS-1)),
												 asin(2.*1.0/(SUPPORT_POINTS-1)),
												 asin(3.*1.0/(SUPPORT_POINTS-1)),
												 asin(4.*1.0/(SUPPORT_POINTS-1)),
												 asin(5.*1.0/(SUPPORT_POINTS-1)),
												 asin(6.*1.0/(SUPPORT_POINTS-1)),
												 asin(7.*1.0/(SUPPORT_POINTS-1)),
												 asin(8.*1.0/(SUPPORT_POINTS-1)),
												 asin(9.*1.0/(SUPPORT_POINTS-1)),
												 asin(10.*1.0/(SUPPORT_POINTS-1)),
												 asin(11.*1.0/(SUPPORT_POINTS-1)),
												 asin(12.*1.0/(SUPPORT_POINTS-1)),
												 asin(13.*1.0/(SUPPORT_POINTS-1)),
												 asin(14.*1.0/(SUPPORT_POINTS-1)),
												 asin(15.*1.0/(SUPPORT_POINTS-1)),
												 asin(16.*1.0/(SUPPORT_POINTS-1)),
												 asin(17.*1.0/(SUPPORT_POINTS-1)),
												 asin(18.*1.0/(SUPPORT_POINTS-1)),
												 asin(19.*1.0/(SUPPORT_POINTS-1)),
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

	float result = interpolate(asinSupportArray, ((float)SUPPORT_POINTS-1) / 1.0, x);
	return result ;
}

float acosFast(float x) {
	if (!useFastMath)
		return acos(x);

	if (x<0.0)
		return M_PI_2-acosFast(x);

	float result = interpolate(acosSupportArray, ((float)SUPPORT_POINTS-1) / 1.0, x);
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


