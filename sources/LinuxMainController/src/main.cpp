#include "BezierCurve.h"
#include "LuciController.h"
#include "Spatial.h"
#include "Sight.h"
#include "string.h"

bool productiveOn = false;
extern bool debugServo;
extern bool debugBezier;
extern bool debugTrajectory;
extern bool debugBezier;
extern bool debugLED;
extern bool debugPattern;
extern bool debugKinematic;
#ifdef INCLUDE_LUCI_SIGHT
extern bool debugFaceRecognition;
#endif

int main(int argc, char** argv) {
	bool wrongCall = false;
    for (int i = 1; i < argc; ++i) {
    	if (!strcmp(argv[i],"-p")) {
    		productiveOn = true;
    		cout << "productive mode on" << endl;
    		continue;
    	}
    	if ( (!strcmp(argv[i],"-v")) && (i<argc-1)) {
    		i++;
    		if (!strcmp(argv[i],"servo") ) {
    			debugServo = true;
    			continue;
    		}
        	if (!strcmp(argv[i],"bezier")) {
        		debugBezier = true;
        		continue;
        	}
        	if (!strcmp(argv[i],"led")) {
        		debugLED = true;
        		continue;
        	}
        	if (!strcmp(argv[i],"trajectory")) {
        		debugTrajectory = true;
        		continue;
        	}
        	if (!strcmp(argv[i],"pattern")) {
        		debugPattern = true;
        		continue;
        	}
        	if (!strcmp(argv[i],"kinematic")) {
        		debugKinematic = true;
        		continue;
        	}
#ifdef INCLUDE_LUCI_SIGHT
        	if (!strcmp(argv[i],"cam")) {
        		debugFaceRecognition = true;
        		continue;
        	}
#endif
        	cout << "-v " << argv[i] << " is unknown. " << endl;
        	wrongCall = true;

    	}
    	if (!strcmp(argv[i],"-h")) {
    		cout << "usage: Luci [-p][-v (bezier|servo|led|trajectory|pattern|kinematic|cam)]" << endl;
    		cout << "-p      productive mode" << endl
    			 << "-v<xxx> print debug message of module x" << endl;
			return 0;
    	}
    	cout << "parameter[" << i << "]=" << argv[i] << " unknown." << endl;
    	wrongCall = true;
    }
    if (wrongCall)
    	return -1;

	setupLuci(productiveOn);		// setup everything besides webcam
	setupSight();					// start thread that continuously checks for faces

	bool continueLoop = true;
	while (continueLoop) {
		continueLoop = loopLuci();  // do all controller tasks
	};

	tearDownLuci();
	tearDownSight();

	return 0;
}
