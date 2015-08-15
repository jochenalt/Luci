/*
 * Audio.h
 *
 *  Created on: 16.03.2015
 *      Author: JochenAlt
 */

#ifndef AUDIO_H_
#define AUDIO_H_

#include "DrumDetection.h"
#include <stdint.h>

#define SOUNDBUFFERSIZE 1024
#define FFT_SIZE 512
#define SOUND_SAMPLE_RATE 22050 // sample/s
#define MIN_BPM 50
#define MAX_BPM 160
#define DRUM_BUFFER_DURATION 200 // ms

#define SAMPLES_PER_SECOND (SOUND_SAMPLE_RATE/SOUNDBUFFERSIZE)
#define CONVOLUTED_WINDOWS 4 // number of convoluted windows

#define DRUM_BUFFER_FILTER ((1000./SAMPLES_PER_SECOND)/DRUM_BUFFER_DURATION)

#define MIN_BEAT_PERIOD_MS (60L*4L*1000L/MAX_BPM) // 1333
#define MAX_BEAT_PERIOD_MS (60L*4L*1000L/MIN_BPM) // 4000

#define BEAT_WINDOW_ARRAY_LEN MAX_BEAT_PERIOD_ARR_LEN // 160
#define HISTORY_ARRAY_LEN (BEAT_WINDOW_ARRAY_LEN*CONVOLUTED_WINDOWS)

#define MIN_BEAT_PERIOD_ARR_LEN ((SAMPLES_PER_SECOND * MIN_BEAT_PERIOD_MS) / 1000) // 53
#define MAX_BEAT_PERIOD_ARR_LEN ((SAMPLES_PER_SECOND * MAX_BEAT_PERIOD_MS) / 1000) // 160
#define BEAT_WINDOW_SIZE_MS MAX_BEAT_PERIOD_MS


class BeatDetection {
public:
	BeatDetection() {
		drumHistoryPos = 0;
		for (int i = 0;i<HISTORY_ARRAY_LEN;i++) {
			drumHistory[i] = 0;
			convolutedDrumHistory[i] = 0;
		}
		detectedBeatCycle = 0;
		detectedNextBeat = 0;
		detectedLastBeat = 0;
		lastBeatPrediction = 0;
		beatSqueezeFactor = 0;
		drumHistoryLen = 0;
		sampleRate = 0;
	}

	void setSampleRate(int16_t pSampleRate) { sampleRate = pSampleRate; };
	int16_t getSampleRate() { return sampleRate; };

	static BeatDetection& getInstance() {
		static BeatDetection instance;
		return instance;
	}
	void print(int beatPeriodArrIdx);
	void setup();
	bool audioReceived(int16_t input[], int bufferSize);

	bool getBeat(int & nextBeat_ms);
	int getNextBeat_ms();
	int getBeatCycle_ms();

	bool isKick()
	{
		return lastKick >= 1.;
	}
	bool isSnare()
	{
		return lastSnare>= 1.;
	}
	bool isHiHat()
	{
		return lastHiHat >= 1.;
	}
private:
	int weightedDrums();
	bool updateBeatDetection(uint32_t currentTime_ms);
	bool processBeatConvolution(uint32_t currentTime_ms, uint32_t &nextBeat_ms, int &beatCycle_ms);
	void addToDrumHistory();
	bool isDrumHistoryFull();

	void sample(float& kick, float& snare, float& hihat);

	// returns the drum value of the current position minus a passed offset
	uint8_t getDrumByPos(int16_t goBackPos) {
		if (goBackPos >= drumHistoryLen)
			return 0;
		int idx = (drumHistoryPos-goBackPos+HISTORY_ARRAY_LEN) % HISTORY_ARRAY_LEN;
		return drumHistory[idx];
	}
	void cleanDrumHistory();
	uint16_t getDrumHistoryLen();

	uint8_t convoluteCurrentDrumWindowWith(int16_t goBack_ms);
	bool isBeatWindow(uint16_t variance);
	int getSlotFromMS(uint32_t time_ms);

	DrumDetection beat;

	uint8_t drumHistory[HISTORY_ARRAY_LEN];
	uint8_t convolutedDrumHistory[HISTORY_ARRAY_LEN];

	uint16_t drumHistoryPos;
	uint16_t drumHistoryLen;

	uint16_t detectedBeatCycle;
	uint32_t detectedNextBeat;
	uint32_t detectedLastBeat;

	uint32_t lastBeatPrediction;
	int beatSqueezeFactor;
	int16_t sampleRate;

	float lastKick;
	float lastSnare;
	float lastHiHat;

};

#endif /* AUDIO_H_ */
