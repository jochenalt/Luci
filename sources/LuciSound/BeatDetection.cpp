/*
 * Audio.cpp
 *
 *  Created on: 16.03.2015
 *      Author: JochenAlt
 */

#include "BeatDetection.h"
#include "stdint.h"
#include <sys/time.h>
#include "math.h"
#include <iostream>
#include "util.h"
#include <iomanip>

using namespace std;

unsigned long millis() {
	struct timeval tp;
	gettimeofday(&tp, 0);
	long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
    return ms;
}

void BeatDetection::setup(){
	detectedBeatCycle = 0;
	detectedNextBeat = 0;
	detectedLastBeat = 0;
	lastBeatPrediction = 0;
	cleanDrumHistory();
}

bool BeatDetection::audioReceived(int16_t input[], int bufferSize) {
  uint32_t currentTime_ms = millis();

  // fetch audio data of constant size
  beat.audioReceived(input, bufferSize);

 // process fft data
  beat.processFFTData();

  int drumWeight = weightedDrums();

  cout << " " << (isKick()?'*':' ') << " " << (isSnare()?'*':' ') << " " << (isHiHat()?'*':' ')  << drumWeight << " " << drumHistoryLen ;

  // add detected base, snear and hihat to drum history
  addToDrumHistory();

  // process beat detection
  bool beatFound = updateBeatDetection(currentTime_ms);
  return beatFound;
}

#define WEIGHTED_DRUMS_MAX (128L*128L)

int BeatDetection::weightedDrums() {
	lastKick = beat.kick();
	lastSnare= beat.snare();
	lastHiHat = beat.hiHat();

	float drum = (lastKick*32 + 16.*lastSnare + 0*lastHiHat) /4;
	return constrain(drum,0,255);
}


void BeatDetection::print(int beatPeriodArrIdx) {
	cout << "print(" << beatPeriodArrIdx << ")" << endl;

	cout << "goback " ;
	for (int i = 0;i<beatPeriodArrIdx;i++) {
		cout << setw(4) << right << i;
	}
	cout << endl;
	for (int widx = 0;widx<CONVOLUTED_WINDOWS;widx++) {
		cout << "drum(" << widx << ")";
		for (int i = 0;i<beatPeriodArrIdx;i++) {
			int drumPos = beatPeriodArrIdx*widx + i;
			cout << right << setw(4) << (int)getDrumByPos(drumPos);
		}
		cout << endl;
	}

	int beatPeriodValue = convoluteCurrentDrumWindowWith(beatPeriodArrIdx);
	cout << "convo=" << beatPeriodValue << endl;
}


// returns a metric [0..255] representing a beat difference of the
// current window and the window which is goBack entries earlier.
uint8_t BeatDetection::convoluteCurrentDrumWindowWith(int16_t goBack) {
	// compare window of  entries with history
	long diffSum = 0;

	int len = goBack;
	// current windows goes from [BEAT_WINDOW_ARRAY_LEN..0] (id=0 means latest sample)
	// compared window goes from [BEAT_WINDOW_ARRAY_LEN+goBack..goBack

	// start comparison of window defined by goBack and current window
	int cmpWinSamplePos = len+goBack-1;
	int count = 0;
	for (int currWinSamplePos = len-1;
			currWinSamplePos>0;) {
		int16_t currSample =getDrumByPos(currWinSamplePos--);
		for (int backWindowIdx = 0;backWindowIdx<CONVOLUTED_WINDOWS-1;backWindowIdx++) {
			int16_t goBackIdx = cmpWinSamplePos+backWindowIdx*len;
			if (goBackIdx < drumHistoryLen) {
				int16_t winSample = getDrumByPos(goBackIdx);
				int d = 0; // = difference in %
				if (currSample > winSample)
					d = ((currSample+10)*100)/(winSample+10)-100;
				else
					d = ((winSample+10)*100)/(currSample+10)-100;
				d = constrain(d,0,500);
				diffSum += d*d;
				count ++;
			}
		}
		cmpWinSamplePos--;

	}
	uint8_t result = 0;
	if (count > 0)
		result = constrain(sqrtf(((float)diffSum)/((float)count)),0,255);
	return result;
}

bool BeatDetection::isBeatWindow(uint16_t variance) {
	return variance<(WEIGHTED_DRUMS_MAX/8);
}

void BeatDetection::addToDrumHistory() {
	// add current drum to history
	uint8_t drums = weightedDrums();

	drumHistoryPos++;
	if (drumHistoryPos >= HISTORY_ARRAY_LEN)
		drumHistoryPos = 0;

	drumHistoryLen++;
	if (drumHistoryLen > HISTORY_ARRAY_LEN )
		drumHistoryLen = HISTORY_ARRAY_LEN;

	drumHistory[drumHistoryPos]= drums;
}

uint16_t BeatDetection::getDrumHistoryLen() {
	return drumHistoryLen;
}

bool BeatDetection::isDrumHistoryFull() {
	return drumHistoryLen == HISTORY_ARRAY_LEN;
}


void BeatDetection::cleanDrumHistory() {
	for (int i = 0;i<HISTORY_ARRAY_LEN;i++) {
		drumHistory[i] = 0;
		convolutedDrumHistory[i] = 0;
	}
	drumHistoryLen = 0;
	drumHistoryPos = 0;
}

int BeatDetection::getSlotFromMS(uint32_t time_ms) {
	 return (time_ms*SAMPLES_PER_SECOND)/1000;
}

bool BeatDetection::processBeatConvolution(uint32_t currentTime_ms, uint32_t &pNextBeat_ms, int &pBeatCycle_ms) {
	static int callCounter= 0;
	callCounter++;

	// now find the best fitting frequency by linear
	// search through all possible frequencies
	int minBeatPeriodArrLen = MIN_BEAT_PERIOD_ARR_LEN; // =53
	int maxBeatPeriodArrLen = MAX_BEAT_PERIOD_ARR_LEN; // =160
	// int beatCycleArrLen= getSlotFromMS(pBeatCycle_ms);
	int beatCmpValue[maxBeatPeriodArrLen];
	int beatCmpValueAvr = 0;
	for (int beatPeriodArrIdx = minBeatPeriodArrLen;beatPeriodArrIdx<maxBeatPeriodArrLen;beatPeriodArrIdx++) {
		int beatPeriodValue = convoluteCurrentDrumWindowWith(beatPeriodArrIdx);
		// to get more stability of the result, sub 20% weight of the previous beat cycle to the assessment value
		int filteredBeatPerioValue = beatPeriodValue;
		/*
		if (pBeatCycle_ms != 0) {
			int c = 100-100/(500+abs(filteredBeatPerioValue-beatCycleArrLen));// add 20% weight of previous beat cycle
			filteredBeatPerioValue = (filteredBeatPerioValue*c)/100;
		}
		*/
		beatCmpValue[beatPeriodArrIdx] = filteredBeatPerioValue;
		beatCmpValueAvr += filteredBeatPerioValue;
		if ((callCounter == 104) && ((beatPeriodArrIdx == 74) || (beatPeriodArrIdx == 99)))
			print(beatPeriodArrIdx);

	}
	beatCmpValueAvr = (8*beatCmpValueAvr)/((maxBeatPeriodArrLen-minBeatPeriodArrLen)*10);
	cout << " avr=" << beatCmpValueAvr;

	int minBeatPeriodValue = 256;
	int bestBeatPeriodArrLen = 0;
	for (int beatPeriodArrIdx = minBeatPeriodArrLen;beatPeriodArrIdx<maxBeatPeriodArrLen;beatPeriodArrIdx++) {
		if (beatCmpValue[beatPeriodArrIdx] >= beatCmpValueAvr) {
			beatCmpValue[beatPeriodArrIdx] = 255;
		}
		else {
			cout << " " << beatPeriodArrIdx << ":" << beatCmpValue[beatPeriodArrIdx] ;
			if (beatCmpValue[beatPeriodArrIdx] < minBeatPeriodValue ) {
				minBeatPeriodValue = beatCmpValue[beatPeriodArrIdx];
				bestBeatPeriodArrLen = beatPeriodArrIdx;
			}
		}
	}

	// check if the comparison value is sufficient
	cout << " #" << callCounter << " X(" <<bestBeatPeriodArrLen<< "," << minBeatPeriodValue  << ")";
	if ((minBeatPeriodValue > 1) && (minBeatPeriodValue < beatCmpValueAvr)) {
		if (pBeatCycle_ms > 0) {
			// if there was a beat cycle already, bring it in the same period (solve 120bpm-vs-60-bpm-problem)

			/*
			// if we have more than 120 bpm, norm to half bpm to bring it between 60 and 120 BPM
			if ((60UL*4UL*SAMPLES_PER_SECOND)/bestBeatPeriodArrLen > 120) {
				bestBeatPeriodArrLen *= 2;
				cout << "<<";
			}

			if ((60UL*4UL*SAMPLES_PER_SECOND)/bestBeatPeriodArrLen < 60) {
				bestBeatPeriodArrLen /= 2;
				cout << ">>";
			}
			*/

		}
		// ok, beat cycle detected, return in [ms]
		pBeatCycle_ms = (bestBeatPeriodArrLen*1000)/SAMPLES_PER_SECOND;
		cout << "-> (" << bestBeatPeriodArrLen<< "," << minBeatPeriodValue  << "," << pBeatCycle_ms << "ms)";

		// now get the prediction of the next beat, by checking
		// the detected window for the biggest beat
		int drumFilteredMax = 0;
		uint16_t maxDrumPos = 0;

		int16_t corrPrevBeatIdx = 0;

		if (pNextBeat_ms > 0) {
			// compute the previous beat that is closest to current point in time (gives a negative number)
			int32_t corrPrevBeat_ms = (((pNextBeat_ms-currentTime_ms) + pBeatCycle_ms) % pBeatCycle_ms) - pBeatCycle_ms;
			corrPrevBeatIdx = getSlotFromMS(-corrPrevBeat_ms);
		}

		for (int i = 0;i<bestBeatPeriodArrLen;i++) {
			int drumSum = 0;
			int s = 0;
			// multiply all available frames from history to get a better average
			// try to find the beat with the smallest deviation
			for (int j = i;j<min(CONVOLUTED_WINDOWS*bestBeatPeriodArrLen,drumHistoryLen);j += bestBeatPeriodArrLen) {
				int drum = getDrumByPos(j);
				drumSum += drum*drum;
				s++;
			}
			drumSum /= s;
			int filteredDrum = drumSum;
			if (pNextBeat_ms != 0) {
				int c = 100+100/(5+sqrInt(i-corrPrevBeatIdx));// add 20% weight of previous beat cycle
				filteredDrum *= c/100;
			}
			if (filteredDrum > drumFilteredMax) {
				maxDrumPos = i;
				drumFilteredMax = filteredDrum;
			}
		}
		pNextBeat_ms = currentTime_ms + (bestBeatPeriodArrLen - maxDrumPos)*1000/SAMPLES_PER_SECOND;
		return true;
	}

	return false;
}



bool BeatDetection::updateBeatDetection(uint32_t currentTime_ms) {
	if (getDrumHistoryLen() >= BEAT_WINDOW_ARRAY_LEN*2) { // wait until we have enough samples
		uint32_t localNextBeat_ms = detectedNextBeat;
		int localBeatCycle_ms = detectedBeatCycle;
		bool beatPredictionSucces = processBeatConvolution(currentTime_ms, localNextBeat_ms, localBeatCycle_ms);
		if (beatPredictionSucces) {
			// if next beat has nearly the same period, slightly adapt it but take it over
			// otherwise ignore this next beat and take last beat + cycle time to avoid hiccups
			// assuming that the music has some phrasing that let the same beat repeat after a full phrase
			detectedBeatCycle = localBeatCycle_ms;
			detectedNextBeat = localNextBeat_ms;
			return true;
		}
	}
	return false;
}

int BeatDetection::getNextBeat_ms() {
	return detectedLastBeat + getBeatCycle_ms();
}

int BeatDetection::getBeatCycle_ms() {
	if (beatSqueezeFactor >= 0)
		return detectedBeatCycle<<beatSqueezeFactor;
	else
		return detectedBeatCycle>>-beatSqueezeFactor;
}

bool BeatDetection::getBeat(int &nextBeat_ms) {
	uint32_t lNextBeat = getNextBeat_ms();
	int lCycleBeat = getBeatCycle_ms();
	uint32_t now = millis();
	if (lCycleBeat > 0 && (now > lNextBeat)) {
		nextBeat_ms = lNextBeat + lCycleBeat;

		detectedLastBeat = lNextBeat;
		detectedNextBeat = lNextBeat + lCycleBeat;
		return true;
	}
	return false;
}
