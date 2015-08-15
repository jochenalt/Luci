
#ifndef _DRUM_DETECTION_H_
#define _DRUM_DETECTION_H_

#include "fft.h"
#include "stdint.h"

#define FFT_SUBBANDS 64
#define ENERGY_HISTORY 43

class DrumDetection {

  float averageEnergy[FFT_SUBBANDS];
  float fftVariance[FFT_SUBBANDS];
  float beatValueArray[FFT_SUBBANDS];
  float energyHistory[FFT_SUBBANDS][ENERGY_HISTORY];
  float *in_fft;
  int historyPos;

  float fftSubbands[FFT_SUBBANDS];

  int buffer_size;
  int fft_size;

  float *magnitude, *audio_input;

  fft myfft;


public:
  DrumDetection();
  void processFFTData();

  float beat(int subband);
  float  beatRange(int low, int high);

  float kick();
  float snare();
  float hiHat();

  void audioReceived(int16_t[], int);
};

#endif
