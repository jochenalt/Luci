
#include <iostream>
#include "DrumDetection.h"
#include "math.h"
#include <string.h>
#include "BeatDetection.h"
#include "util.h"

DrumDetection::DrumDetection() : buffer_size(SOUNDBUFFERSIZE), fft_size(FFT_SIZE) {

  for(int i = 0; i < FFT_SUBBANDS; i++)
  {
    for(int l = 0; l < ENERGY_HISTORY; l++){
      energyHistory[i][l] = 0;
    }
    fftSubbands[i] = 0;
    averageEnergy[i] = 0;
    fftVariance[i] = 0;
    beatValueArray[i] = 0;
  }

  audio_input = new float[buffer_size];
  magnitude = new float[fft_size];

  for (int i = 0; i < fft_size; i++) {
    magnitude[i] = 0;
  }

  historyPos = 0;
}


void DrumDetection::processFFTData() {
    in_fft = magnitude;

    for(int i = 0; i < FFT_SUBBANDS; i++) {

    	 // compute a band pass for allsubbands (sum up all included frequencies)
        float currSubBand = 0;
        int fftStartSubBandIdx = i*(fft_size/FFT_SUBBANDS);
        for(int b = 0; b < fft_size/FFT_SUBBANDS; b++) {
        	currSubBand +=  in_fft[fftStartSubBandIdx+b];
        }
        fftSubbands[i] = currSubBand*(float)FFT_SUBBANDS/(float)fft_size;

        // compute variance of fft energy
        // fftVariance[i] = 0;
        float currVariance = 0;
        for(int b=0; b < fft_size/FFT_SUBBANDS; b++) {
          float d = (in_fft[fftStartSubBandIdx+b] - fftSubbands[i]);
          fftVariance[i] += d*d;
        }
        fftVariance[i] = fftVariance[i]*(float)FFT_SUBBANDS/(float)fft_size;

    	// compute C using a linear digression of C with V
        // (no idea where these values come from, but it seems to be common sense)
		float C = (-0.0025714 * fftVariance[i]) + 1.5142857;

		// use positive values only
		beatValueArray[i] = C<1.?1:C;
	}

	// compute average energy
	for(int i = 0; i < FFT_SUBBANDS; i++) {
		float avrEnergy = 0;
		for(int h = 0; h < ENERGY_HISTORY; h++)
			avrEnergy += energyHistory[i][h];
		averageEnergy[i] = avrEnergy / ENERGY_HISTORY;
	}
    // store in rollating energy history for computation of average energy in the next loop
	for(int i = 0; i < FFT_SUBBANDS; i++)
		energyHistory[i][historyPos] = fftSubbands[i];

	historyPos = (historyPos+1) % ENERGY_HISTORY;

}


void DrumDetection::audioReceived(int16_t input[], int pBufferSize) {
	if (pBufferSize != buffer_size) {
		std::cerr << "buffer size " << pBufferSize << " != " << buffer_size << std::endl;
	}

	for (int i=0;i<buffer_size;i++)
	  audio_input[i] = input[i];

  myfft.powerSpectrum(0, (int)fft_size, audio_input, buffer_size, magnitude);

  for (int i = 0; i < fft_size; i++) {
    // magnitude[i] = (magnitude[i]);
    magnitude[i] = sqrt(magnitude[i]);
  }
}


// returns a factor how much the beat is above the average in the passed subband.
// > 1 means there is a beat, otherwise it is 0
float DrumDetection::beat(int subband)
{
  float result = (fftSubbands[subband])/(averageEnergy[subband]);
  if (result > beatValueArray[subband])
	  return constrain(result,0,5);
  else
	  return 0.;
}


// compute a kick out of the base drum subband
// first compute the ratio the current drum is above the average (beatRange()), sum up the increments (forget decreases)
// and filter it by a maximum increase and decrease rate to allow small differences in timing.
// In theory this should be a hanning window again, but this simpler approach works as well
float DrumDetection::kick() {
	static int16_t LoFreq = 40; // Hz
	static int16_t HiFreq = 60;
	int16_t loBand  = (LoFreq *FFT_SUBBANDS)/BeatDetection::getInstance().getSampleRate();
	int16_t hiBand  = (HiFreq *FFT_SUBBANDS)/BeatDetection::getInstance().getSampleRate();
	float result = beatRange(loBand,hiBand);
	static float prevResult = 0;
	static float inc = 0;
	if (result - prevResult > 0)
		inc += result - prevResult;
	prevResult = result;
	static float filter = 0.;
	float d = inc;
	// the increement may increase the filter by 30% max
	d = constrain(inc,0,(((1.-DRUM_BUFFER_FILTER)*32 +1) * (1.+DRUM_BUFFER_FILTER)*100)/100 - ((1.-DRUM_BUFFER_FILTER)*32+1));
	filter = (1.-DRUM_BUFFER_FILTER)*filter; // decrease result value from last call = decay of drums
	filter += d; // add up the limited increment
	inc -= d;    // dont forget what has been cut off but save for the next call, sub the limited amount only.

	return filter;
}
float DrumDetection::snare()
{

	static int16_t LoFreq = 200;  // Hz
	static int16_t HiFreq = 500; // Hz
	int16_t loBand  = (LoFreq *FFT_SUBBANDS)/BeatDetection::getInstance().getSampleRate();
	int16_t hiBand  = (HiFreq *FFT_SUBBANDS)/BeatDetection::getInstance().getSampleRate();
	float result = beatRange(loBand,hiBand);
	static float prevResult = 0;
	static float inc = 0;
	if (result - prevResult > 0)
		inc += result - prevResult;
	prevResult = result;
	static float filter = 0.;
	float d = inc;
	// the increement may increase the filter by 30% max
	d = constrain(inc,0,(((1.-DRUM_BUFFER_FILTER)*32 +1) * (1.+DRUM_BUFFER_FILTER)*100)/100 - ((1.-DRUM_BUFFER_FILTER)*32+1));
	filter = (1.-DRUM_BUFFER_FILTER)*filter;
	filter += d;
	inc -= d;

	return filter;
}


float DrumDetection::hiHat()
{
	static int16_t LoFreq = 3000;  // Hz
	static int16_t HiFreq = 10000; // Hz
	int16_t loBand  = (LoFreq *FFT_SUBBANDS)/BeatDetection::getInstance().getSampleRate();
	int16_t hiBand  = (HiFreq *FFT_SUBBANDS)/BeatDetection::getInstance().getSampleRate();
	float result = beatRange(loBand,hiBand);
	static float prevResult = 0;
	static float inc = 0;
	if (result - prevResult > 0)
		inc += result - prevResult;
	prevResult = result;
	static float filter = 0.;
	float d = inc;
	// the increement may increase the filter by 30% max
	d = constrain(inc,0,(((1.-DRUM_BUFFER_FILTER)*128 +1) * (1.+DRUM_BUFFER_FILTER)*100)/100 - ((1.-DRUM_BUFFER_FILTER)*128+1));
	filter = (1.-DRUM_BUFFER_FILTER)*filter;
	filter += d;
	inc -= d;

	return filter;
}

float  DrumDetection::beatRange(int low, int high)
{
  float sum = 0.;
  for(int i = low; i < high+1; i++) {
	  sum += beat(i);
  }
  return sum/(high-low+1);
}
