/*
 * WavFile.cpp
 *
 *  Created on: 20.03.2015
 *      Author: JochenAlt
 */

#include "WavFile.h"
#include "stdio.h"
#include <stdint.h>
#include "string.h"

void WavFile::readFileHeader (string filename) {

	const char* filename_c = filename.data();
	readSamplesTotal = 0;
    fp = fopen(filename_c,"rb");
	if (fp)
	{
		char id[4];
	    uint32_t size; //32 bit value to hold file size
	    uint16_t format_tag, block_align, dummy; //our 16 values
	    uint32_t format_length, avg_bytes_sec; //our 32 bit values
	    readSamplesTotal = 0;
	    fread(id, sizeof(char), 4, fp); //read in first four bytes
	    if (!strncmp(id, "RIFF",4))
	    { //we had 'RIFF' let's continue
	        fread(&size, sizeof(uint32_t), 1, fp); //read in 32bit size value
	        fread(id, sizeof(uint8_t ), 4, fp); //read in 4 byte string now
	        if (!strncmp(id,"WAVE",4))
	            { //this is probably a wave file since it contained "WAVE"
	        		// read fmt block
	                fread(id, sizeof(char), 4, fp); //read in 4 bytes "fmt ";
	                fread(&format_length, sizeof(uint32_t),1,fp);
	                fread(&format_tag, sizeof(uint16_t), 1, fp); //check mmreg.h  for other
	                                                             // possible format tags like ADPCM
	                fread(&channels, sizeof(uint16_t),1,fp);     //1 mono, 2 stereo
	                fread(&sample_rate, sizeof(uint32_t), 1, fp); //like 44100, 22050, etc...
	                fread(&avg_bytes_sec, sizeof(uint32_t), 1, fp); //probably won't need this
	                fread(&block_align, sizeof(uint16_t), 1, fp);  //probably won't need this
	                fread(&bits_per_sample, sizeof(uint16_t), 1, fp); //8 bit or 16 bit file?

	                if (format_length == 18)
		                fread(&dummy, sizeof(uint16_t), 1, fp); //additional two bytes

	                	// read data
	                bool dataBlock = false;
	                do {
	                	fread(id, sizeof(char), 4, fp); //read in 'data'
	                	fread(&soundDataSize, sizeof(uint32_t), 1, fp); //how many bytes of sound data we have
	                	dataBlock = (strncmp(id,"data",4) == 0);
	                	if (!dataBlock) {
	                		char dummy[soundDataSize];
	                		fread(dummy, sizeof(uint8_t), soundDataSize, fp);
	                	}
	                } while (!dataBlock);
	                soundDataPos = 0;
	            }
            else
                cout << "Error: RIFF file but not a wave file\n" << endl;
        }
        else
            cout << "Error: not a RIFF file" << endl;
    }
}

void WavFile::fetchSoundBuffer() {
	if (soundBlockPos == soundBlockLen) {
		if (soundDataPos + SOUND_BLOCK_SIZE > soundDataSize)
			soundBlockLen = soundDataSize-soundDataPos;
		else
			soundBlockLen = SOUND_BLOCK_SIZE;
	    fread(soundBlockBuffer, sizeof(uint8_t), soundBlockLen, fp);
	    soundBlockPos = 0;
	}
}

int16_t WavFile::getSoundData() {
	if (soundBlockPos == soundBlockLen)
		fetchSoundBuffer();
	uint8_t data = soundBlockBuffer[soundBlockPos];
	soundBlockPos++;
	soundDataPos++;
	return data;
}

int16_t WavFile::getSoundSample() {
	if (fp != NULL) {
		if ((bits_per_sample == 8) && (channels == 1))
	    	return getSoundData()-128;
		if ((bits_per_sample == 8) && (channels == 2))
	    	return (getSoundData() + getSoundData()) / 2;
		if ((bits_per_sample == 16) && (channels == 1))
	    	return (getSoundData()) + (getSoundData()<< 8);
		if ((bits_per_sample == 16) && (channels == 2))
	    	return ((getSoundData()) + (getSoundData()<<8) + (getSoundData()) + (getSoundData()<<8)) / 2;
		cerr << "could not fetch sound sample" << endl;
	}
	return 0;
}


long WavFile::fetchSoundSamples(int16_t buffer[], uint16_t &bufferSize) {
	if (fp != NULL) {
		readSamplesTotal += bufferSize;
		for (int i = 0;i<bufferSize;i++) {
			int16_t buf = getSoundSample();
			if (i<bufferSize) {
				buffer[i] = buf;
				if (soundDataPos >= soundDataSize) {
					bufferSize = i;
					break;
				}
			}
		}
	}
	return (readSamplesTotal*1000)/sample_rate;
}
