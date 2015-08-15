/*
 * Sight.h
 *
 *  Created on: 16.03.2015
 *      Author: JochenAlt
 */

#ifndef SIGHT_H_
#define SIGHT_H_

#include "Spatial.h"

void setupSight();
void loopSight();
void tearDownSight();


#ifdef INCLUDE_LUCI_SIGHT
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <boost/thread.hpp>

class Sight {
public:
	Sight();
	bool setup(boost::thread *workerThreadFaceRecognition,boost::thread *workerThreadImageCapture);
	static Sight& getInstance() {
		static Sight instance;
		return instance;
	}

	void loop();
	bool isNewFaceDetected(std::vector<TimedPosition> &faces, Position& pPositionAtFaceDetection);
	float getFps() { return fps; }
	float getCameraFps() { return cameraFps; }
	void setCamLatency(int pCamLatency_ms) { cameraLatency_ms = pCamLatency_ms; };
	int getCamLatency() { return cameraLatency_ms; };

	void setImage(cv::Mat *newImage);
	void getImage(cv::Mat& image);
	bool captureImage(cv::Mat& image);

private:
		cv::CascadeClassifier faceCascade;
		cv::VideoCapture captureDevice;
	    std::vector<cv::Rect> detectedFaces;
	    std::vector<TimedPosition> transferFaces;
	    Position positionAtFaceDetection;

	    bool newFrameIsAvailable;
		float fps;
		float cameraFps;
		int cameraLatency_ms;

		// double buffering for communication between image grabbing thread and main thread
		cv::Mat* imageBuffer;
		volatile bool imageBufferInUse;
};

#endif
#endif /* SIGHT_H_ */
