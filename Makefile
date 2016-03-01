CC=g++ -g
LINKER=g++ -g

OPENCV_LIBS=`pkg-config --cflags --libs opencv`

current: xyz1

all: xyz1 test

xyz1: pose tracking xyz1.cpp
	$(LINKER) -o xyz1 xyz1.cpp Pose.o Tracking.o $(OPENCV_LIBS)
	./xyz1

test: pose tracking test.cpp
	$(LINKER) -o test test.cpp Pose.o Tracking.o $(OPENCV_LIBS)
	./test

pose: Pose.cpp Pose.h
	$(CC) -c Pose.cpp $(OPENCV_LIBS)

tracking: pose Tracking.h Tracking.cpp
	$(CC) -c Tracking.cpp $(OPENCV_LIBS)

reference_frame: pose Reference_Frame.h Reference_Frame.cpp
	$(CC) -c Reference_frame.cpp $(OPENCV_LIBS)