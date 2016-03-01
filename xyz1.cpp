#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>
#include <time.h>

#include "iostream"

using namespace cv;
using namespace std;

long long getTimeUsec()
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000LL + (now.tv_nsec / 1000);
}

long long ticTime;

void tic() {
    ticTime = getTimeUsec();
}
long long toc() {
    long long tocTime = getTimeUsec();
    return tocTime - ticTime;
}

int main(int argc, char const *argv[])
{
	// What's the focal length / format of the depth images?

	Mat sizeable = imread("xyz_sanitycheck/c1.png", CV_LOAD_IMAGE_GRAYSCALE);
	Tracking tracker(525.0, sizeable.cols, sizeable.rows);
	namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.

	Matx31d _t(1.1406, 0.6213, 1.4205);
	Matx33d _R( 0.6869071621348888,   0.02406923471782091, -0.7263464893202772,
			   -0.7266645567199284,   0.03763380653472703, -0.6859608725085831,
				0.010824630008607339, 0.9990016859574387, 	0.04334119102391609);

	//Matx33d _R(1, 0, 0, 0, 1, 0, 0, 0, 1);

	Pose initial(_t, _R);
	int frame_count = 19;
	tic();
	for (int frame = 1; frame < frame_count; frame++)
	{
		// read rgb
		char color_files[30]; // use string type
		sprintf(color_files, "xyz_sanitycheck/c%d.png", frame);
		Mat current_grayscale = imread(color_files, CV_LOAD_IMAGE_GRAYSCALE);
		current_grayscale.convertTo(current_grayscale, CV_64FC1);

		// read next rgb
		sprintf(color_files, "xyz_sanitycheck/c%d.png", frame + 1);
		Mat new_grayscale = imread(color_files, CV_LOAD_IMAGE_GRAYSCALE);
		new_grayscale.convertTo(new_grayscale, CV_64FC1);

		// read [inverse] depth
		sprintf(color_files, "xyz_sanitycheck/d%d.png", frame);
		Mat inverse_depth_and_variance = imread(color_files, CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
		inverse_depth_and_variance.convertTo(inverse_depth_and_variance, CV_64FC1);
		inverse_depth_and_variance = 5000.0 / inverse_depth_and_variance; // so 1.0 / 0 returns 0 in the mat? *shudder*

		Pose step = tracker.track(current_grayscale, inverse_depth_and_variance, new_grayscale);
		initial = step.compose_with(initial);
		initial.print();
		printf("%s%f\n", "got here - ", toc()/1000.0);
	}
	

	// initialize

	tic();

	

	return 0;
}	