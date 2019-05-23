CC=gcc
CXX=g++
RM=rm -f

SRC=../src
LIB=./lib
OPENCV_LIB=`pkg-config opencv --libs`
LDLIBS=$(OPENCV_LIB)  -lboost_thread -lboost_atomic -lboost_system -lboost_date_time -lboost_filesystem
OBJS=$(LIB)/Sight.o $(LIB)/main.o $(LIB)/Util.o $(LIB)/Configuration.o $(LIB)/Kinematic.o $(LIB)/LuciController.o\
     $(LIB)/MicroControllerInterface.o $(LIB)/ServoController.o $(LIB)/Spatial.o $(LIB)/TrajectoryController.o\
     $(LIB)/TrajectoryPattern.o $(LIB)/TrajectoryPatternController.o $(LIB)/BezierCurve.o $(LIB)/I2CInterface.o\
     $(LIB)/LEDController.o $(LIB)/PCA9685.o
INCLUDES=
CXX_FLAGS=-DINCLUDE_LUCI_SIGHT -O1 -g2 -Wall -c -fmessage-length=0 

all: Luci

Luci: $(OBJS) 
	$(CXX) $(LDFLAGS) -o Luci $(OBJS) $(LDLIBS) 

%.o: $(SRC)/%.cpp $(SRC)/%.h
	$(CXX) -o $@ $(INCLUDES) $(CXX_FLAGS) $<

%.o:$(SRC)/%.cpp 
	$(CXX) -o $@ -g -c $(INCLUDES) $(CXX_FLAGS) $<


clean:
	$(RM) $(OBJS) Luci




