
#include <iostream>
#include "BeatDetection.h"
#include "unistd.h" // for sleep
#include "TimePassedBy.h"
#include "WavFile.h"

void setup();
void loop();
void teardown();

WavFile wavfile;
int16_t soundBuffer[SOUNDBUFFERSIZE];



using namespace std;
static float elapsedTime= 0;
static unsigned long start = 0;
static unsigned long loopCounter = 0;
unsigned long elapsedTimeRead_ms = 0;

void setup() {

	unsigned long now = millis();
	while (millis()-now < 2000);

	BeatDetection::getInstance().setup();
	wavfile.readFileHeader("SlimShady.wav");
	elapsedTime = 0;
	BeatDetection& audio = BeatDetection::getInstance();
	audio.setSampleRate(wavfile.getSampleRate());
	start = 0;
	loopCounter = 0;
	elapsedTimeRead_ms = 0;
}

void loop() {
	BeatDetection& audio = BeatDetection::getInstance();

	static TimePassedBy processAudioTimer;

	if (start == 0) {
		elapsedTimeRead_ms = 0;
		start = millis();
	}
	unsigned long now = millis();
	unsigned long elapsedTime = now-start;
	if (elapsedTime> elapsedTimeRead_ms) {
		loopCounter++;
		unsigned long now = millis();
		elapsedTime = now-start;

		uint16_t soundBufferSize = SOUNDBUFFERSIZE;
		elapsedTimeRead_ms = wavfile.fetchSoundSamples (soundBuffer, soundBufferSize);

		if (soundBufferSize > 0) {
			int timediff = (int)elapsedTimeRead_ms - (int)elapsedTime;
			cout << "t=" << elapsedTime << "ms "
				 << timediff << "ms " << " lc=" << loopCounter << " " << soundBufferSize << " ";
			bool beatFound = audio.audioReceived(soundBuffer,soundBufferSize);
			int beatCycle_ms = audio.getBeatCycle_ms();
			int bpm = 0;
			if (beatCycle_ms != 0)
				bpm  = 60UL*4*1000UL/beatCycle_ms;
			if (beatFound)
				cout << "*(" << bpm << ")" << endl;
			else
				cout << "?(" << bpm << ")" << endl;

		}
	} else {
		usleep(1000);
	}
}

void teardown() {

}

int main() {
	setup();

	while (true) {
		loop();
	};

	teardown();
	return 0;
}
