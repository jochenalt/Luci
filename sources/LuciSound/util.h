#ifndef UTIL_H_
#define UTIL_H_

long map(long x, long in_min, long in_max, long out_min, long out_max);

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
float sqrFloat(float x);
int sqrInt(int x);

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a,b) (((a)>(b))?(b):(a))
#define abs(a) (((a)>(0))?(a):-(a))


extern bool debug;
extern bool useFastMath;

void prepareReadKey();
char keyPressed();
void shutdownKey();


#endif
