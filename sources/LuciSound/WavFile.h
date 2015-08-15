/*
 * WavFile.h
 *
 *  Created on: 20.03.2015
 *      Author: JochenAlt
 */

#ifndef WAVFILE_H_
#define WAVFILE_H_

#include <iostream>
#include "stdio.h"
#include <stdint.h>
#include "string.h"

using namespace std;

#define SOUND_BLOCK_SIZE 128 // block size that is read from file, is an arbitrary size indicating how often the file is accessed. Harmless

class WavFile {
public:
	WavFile() {
		soundBlockPos = 0;
		soundDataSize = 0;
		soundDataPos = 0;
		channels = 0;
		bits_per_sample = 0;
		soundBlockLen = 0;
		sample_rate = 0;
		readSamplesTotal = 0;
		fp = NULL;
	}

	void readFileHeader (string filename);
	int16_t getSoundSample();
	long fetchSoundSamples(int16_t buffer[], uint16_t &bufferSize);
	int16_t getSampleRate() { return sample_rate; };
private:
	void fetchSoundBuffer();
	int16_t getSoundData();

	FILE *fp;
	uint32_t soundDataSize; // size of sound data in entire file
	uint32_t soundDataPos; // position of current data in entire file
	uint32_t readSamplesTotal;
	int8_t soundBlockBuffer[SOUND_BLOCK_SIZE];
	uint16_t soundBlockPos;
	uint16_t soundBlockLen;
	uint16_t bits_per_sample;
	uint32_t sample_rate;
	uint16_t channels;

};

#endif /* WAVFILE_H_ */
