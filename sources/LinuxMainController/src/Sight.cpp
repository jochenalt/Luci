/*
 * Sight.cpp
 *
 *  Created on: 16.03.2015
 *      Author: JochenAlt
 */

#include "Spatial.h"

#ifdef INCLUDE_LUCI_SIGHT

#include <unistd.h>
#include <sched.h>
#include "Sight.h"
#include "Util.h"
#include "LuciController.h"
#include "Kinematic.h"

using namespace std;

// possible resolutions with HD3000
// 320x240, 352x288, 416x240 (7,10,15,20,30 fps), 640x360
#define WIDTH 416			// camera resolution in x
#define HEIGHT 240			// camera resolution in y
#define FACE_SIZE_MM 180	// real-life size of the face detection rectangle
#define CAMERA_FPS 15
#define OPENCV_FPS 10
#define CAMERA_ANGLE_DEG_2 (39) // half angle in degrees


// #define PATH_CASCADES "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt_tree.xml"
#define PATH_CASCADES "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt2.xml"
// #define PATH_CASCADES "/usr/local/share/OpenCV/lbpcascades/lbpcascade_frontalface.xml"
// #define PATH_CASCADES2 "/usr/local/share/OpenCV/haarcascades/haarcascade_profileface.xml"

bool debugFaceRecognition = false;
using namespace std;
bool cameraDetected = false;

volatile bool runFaceRecognitionLoop = true; 	// currently not used, supposed to stop webcam thread
volatile bool storeDetectedFaceLock = false; 	// flag used as semaphore for synchronisation between webcam thread and main thread. True, if recognition is running
volatile bool handOverDetectedFaceLock = false; // synchronisation flag, indicating that reconized faces are currently fetched
volatile bool captureImageTheadReady = false; 	// indicator that image grab thread has its first image

void faceRecognitionLoop() {
	// wait until image capturing has been started
	cv::Mat captureFrame;
	do {
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		Sight::getInstance().getImage(captureFrame);
	}
	while (captureFrame.empty());

	// start face recognition loop (until runFaceRecognitionLoop becomees false)
	while (runFaceRecognitionLoop) {
		// wait until last faces have been taken over to main thread completely
		// before starting a face recognition loop
		if (!handOverDetectedFaceLock) {
			static TimePassedBy loopThrottle;
			if (loopThrottle.isDue_ms(1000/OPENCV_FPS))
				Sight::getInstance().loop();
		} else {
			// dont loop active if the semaphore handOverDetectedFaceLock locks
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}
}

void captureImageFromCamLoop() {
	TimePassedBy loopTime;
	int captureBufferIdx = 0;
	cv::Mat captureBuffer[2];

	while (runFaceRecognitionLoop) {
		bool readOk = Sight::getInstance().captureImage(captureBuffer[captureBufferIdx]);
		if (readOk && !captureBuffer[captureBufferIdx].empty()) {
			captureBufferIdx = 1-captureBufferIdx;
			Sight::getInstance().setImage(&captureBuffer[captureBufferIdx]);
			captureImageTheadReady = true; // first image has been grabbed, first recognition can start
		} else {
			// if there is no new image have a break (prevents active loop)
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	};
}

void startBatchThread(boost::thread* workerThread, void (*functionPtr)(void) ) {
	boost::thread captureImageThread(functionPtr);
	workerThread = &captureImageThread;
	int retcode;
	int policy;

	pthread_t threadID = (pthread_t) captureImageThread.native_handle();
	struct sched_param param;

	if ((retcode = pthread_getschedparam(threadID, &policy, &param)) != 0) {
		errno = retcode;
		cerr << "pthread_getschedparam returned " << retcode << endl;
		return;
	}

	policy = SCHED_BATCH;
	param.sched_priority = 0;

	if ((retcode = pthread_setschedparam(threadID, policy, &param)) != 0) {
		errno = retcode;
		cerr << "pthread_setschedparam returned " << retcode << endl;

		if (retcode == EPERM) {
			cerr << "process has to be started as root to lower thread priority." << endl;
		}
	}
}

TimedPosition getFaceCoord(int x, int y, int dx, int dy, unsigned long t) {
	// computation is z = FACE_SIZE_MM/(tan(radian(widthImage/WIDTH*CAMERA_ANGLE))
	// but that's too complex, so simplify it
	TimedPosition result;
	result.pos.point.x = -(WIDTH * (FACE_SIZE_MM/2) *tan(radians(CAMERA_ANGLE_DEG_2))) / dx; // mm
	result.pos.point.z = (x * FACE_SIZE_MM/dx);
	result.pos.point.y = (y * FACE_SIZE_MM/dy);
	result.atTime = t;
	return result;
}

Sight::Sight() {
	newFrameIsAvailable = false;
	fps = 0;
	cameraFps = 0.0;
	cameraLatency_ms = 235;
	imageBuffer = NULL;
	imageBufferInUse = false;
};

void Sight::setImage(cv::Mat *newImage) {
	static unsigned long lastFrameTime = 0;
	unsigned long now = millis();
	unsigned long fps = 1000/(now - lastFrameTime);
	if (lastFrameTime == 0)
		cameraFps = fps;
	else
		cameraFps = 0.8*cameraFps + 0.2*fps; // compute fps with sliding average

	lastFrameTime = now;

	if (!imageBufferInUse)
		imageBuffer = newImage;
}

void Sight::getImage(cv::Mat& image) {
	imageBufferInUse = true; // block critical block
	if (imageBuffer != NULL)
		imageBuffer->copyTo(image);
	imageBufferInUse = false; // unblock critical block
}

bool Sight::captureImage(cv::Mat& image) {
		bool readOk = false;
		if (cameraDetected && captureDevice.isOpened()) {
			readOk = captureDevice.grab();
		}

		if (readOk) {
			readOk = captureDevice.retrieve(image);
			return readOk;
		}
		return false;

}



bool Sight::setup(boost::thread *workerThreadFaceRecognition,boost::thread *workerThreadImageCapture) {
	bool ok = true;

	//setup video capture device and link it to the first capture device
	if (debugFaceRecognition)
		cout << "open camera" << endl;
	captureDevice.open(0);
	captureDevice.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	captureDevice.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
	captureDevice.set(CV_CAP_PROP_FPS, CAMERA_FPS);
	if (captureDevice.isOpened()) {
		cameraDetected = true;
	} else {
		cerr << "opencv::failed to connect to camera" << endl;
		ok = false;
		cameraDetected = false;
	}

	string cascadeFilename = string(PATH_CASCADES);
	if (debugFaceRecognition)
		cout << "load face recognition configuration" << endl;
	faceCascade.load(cascadeFilename);

	if (faceCascade.empty()) {
		cerr << "opencv::cascade " << cascadeFilename << " could not be loaded" << endl;
		ok = false;
	}

	// grab an image to get the camera time to warm up the camera (crash otherwise, really)
	if (debugFaceRecognition)
		cout << "grab test image" << endl;

	cv::Mat image;
	captureImage(image);

	if (ok) {
		boost::this_thread::sleep(boost::posix_time::milliseconds(500)); // give camera time to warm up
		if (debugFaceRecognition)
			cout << "start background image grabber" << endl;

		startBatchThread(workerThreadImageCapture,captureImageFromCamLoop);

		// wait (max 1s) until image grabbing thread has its first image
		unsigned long startWait = millis();
		while (!captureImageTheadReady && (startWait-millis() < 2000))
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));

		// now we have our first image and can start face recognition
		if (debugFaceRecognition)
			cout << "start background face recognizer" << endl;
		startBatchThread(workerThreadFaceRecognition,faceRecognitionLoop);
	}
	cout << endl;
	return ok;
}

void Sight::loop() {
	if (!cameraDetected)
		return;

    cv::Mat grayscaleFrame;
    cv::Mat captureFrame;

    getImage(captureFrame);
	if (!captureFrame.empty()) {
		static unsigned long lastFrameTime = 0;
		unsigned long now = millis(); // later on used as time when we grabbed the image with the face

		Position position;
		Kinematics::getInstance().getHistoricalPosition(position,now-cameraLatency_ms);
		// draw two rectangles to indicate middle of the image
		rectangle(captureFrame, cv::Point(10,10), cv::Point(WIDTH-10,HEIGHT-10), cvScalar(200, 200, 200, 0));
		rectangle(captureFrame, cv::Point(WIDTH/2-2,HEIGHT/2-2), cv::Point(WIDTH/2+2,HEIGHT/2+2), cvScalar(200, 200, 200, 0));

		cvtColor(captureFrame, grayscaleFrame, CV_BGR2GRAY);
		equalizeHist(grayscaleFrame, grayscaleFrame);

		//find faces and store them in the vector array
		faceCascade.detectMultiScale(grayscaleFrame, detectedFaces, 1.04 /* scaling factor */, 8,
					0 | 0, cv::Size(35, 35));
		unsigned long frameProcessingTime = now - lastFrameTime;
		lastFrameTime = now;

		fps = 0.8*fps + (0.2*1000.0)/frameProcessingTime; // compute fps with sliding average

		if (detectedFaces.size() != 0) {
			storeDetectedFaceLock = true; // indicate that transferFaces is not modified

			transferFaces.clear();
			//draw a rectangle for all found faces in the vector array on the original image
			for (int16_t i = detectedFaces.size()-1; i >=0 ; i--) {

				cv::Point pt1(detectedFaces[i].x + detectedFaces[i].width,
						detectedFaces[i].y + detectedFaces[i].height);
				cv::Point pt2(detectedFaces[i].x, detectedFaces[i].y);
				rectangle(captureFrame, pt1, pt2, cvScalar(0, 200, 200, 0));

				TimedPosition face =
						getFaceCoord((detectedFaces[i].x + detectedFaces[i].width/2) - WIDTH/2,
						  HEIGHT/2 - (detectedFaces[i].y + detectedFaces[i].height/2),
						  detectedFaces[i].width,
						  detectedFaces[i].height,now);

				// filter the face
				static TimedPosition filteredFace;
				if (filteredFace.isNull())
					filteredFace = face;
				else
					filteredFace = TimedPosition(face.atTime, (filteredFace.pos*2 + face.pos)/3);

				transferFaces.push_back(face);
				positionAtFaceDetection = position;
			}
			newFrameIsAvailable = true;
		}
		storeDetectedFaceLock = false; // semaphore to indicate we are out of the critical block changing transferFaces

		//print the output
		imwrite("../camera/out.jpg", captureFrame);

	} else
		cerr << "opencv::failed to capture image" << endl;
}

bool Sight::isNewFaceDetected(std::vector<TimedPosition> &faces,Position &pPositionAtFaceDetection) {
	if (!storeDetectedFaceLock) {

		// we are not in the critical block during phase recognition
		if (newFrameIsAvailable) {
			handOverDetectedFaceLock = true;
			faces.clear();
			for (uint16_t i = 0; i < transferFaces.size(); i++) {
				faces.push_back(transferFaces[i]);
			}
			pPositionAtFaceDetection = positionAtFaceDetection;
			handOverDetectedFaceLock = false;
			newFrameIsAvailable = false;
			return true;
		}
	}
	return false;
}



boost::thread *workerThreadFaceRecognictionPtr = NULL;
boost::thread *workerThreadCaptureImagePtr = NULL;

void setupSight() {
	Sight::getInstance().setup(workerThreadFaceRecognictionPtr,workerThreadCaptureImagePtr);
}


void  loopSight() {
	std::vector<TimedPosition> faces;
	Position positionAtFaceDetection;
	if (Sight::getInstance().isNewFaceDetected(faces,positionAtFaceDetection)) {
		if (faces.size() > 0) {
			Point facefromCam(faces[0].pos.point);

			// constraints to ensure that face detection dos not give absurd values
			facefromCam.x = constrain(facefromCam.x, -2000, -200);
			facefromCam.y = constrain(facefromCam.y, -1000, +1000);
			facefromCam.z = constrain(facefromCam.z, -1000, +1000);

			// transform coord of face local to the webcam into global coordinates
			TimedPosition facePosition;
			facePosition.pos.point = Kinematics::getInstance().transformWebcamFaceToPoint(facefromCam, positionAtFaceDetection);
			facePosition.atTime = faces[0].atTime;

			// hand over face position to LuciController
			setFaceDetection(facePosition);

			if (debugFaceRecognition) {
						faces[0].pos.print("webcam@");
						positionAtFaceDetection.print("pos@");
						facePosition.print("face@");
						cout << " t=" << faces[0].atTime
							<< "fps=(" << fixed << setprecision(1) << Sight::getInstance().getCameraFps()
							<< "," << fixed << setprecision(1) << Sight::getInstance().getFps() << ")" << endl;
					}

		}
	};
}

void tearDownSight() {
	if (workerThreadFaceRecognictionPtr != NULL)
		workerThreadFaceRecognictionPtr->join();
	if (workerThreadCaptureImagePtr != NULL)
		workerThreadCaptureImagePtr->join();

}

#else
void setupSight() {};
void loopSight() { };
void tearDownSight() {};
#endif
