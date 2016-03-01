#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"

using namespace cv;
using namespace std;

int main(int argc, char const *argv[])
{
	// initialize
	Tracking tracker(60.0);

	namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.

	// original
	Mat current_grayscale = imread("desk_1_38.png", CV_LOAD_IMAGE_GRAYSCALE);
	current_grayscale.convertTo(current_grayscale, CV_64FC1);
	Mat new_grayscale = imread("desk_1_48.png", CV_LOAD_IMAGE_GRAYSCALE);
	new_grayscale.convertTo(new_grayscale, CV_64FC1);

	// depth editing
	Mat inverse_depth_and_variance = imread("desk_1_38_depth.png", CV_LOAD_IMAGE_GRAYSCALE);
	assert(inverse_depth_and_variance.cols > 0);
	inverse_depth_and_variance.convertTo(inverse_depth_and_variance, CV_64FC1);
	inverse_depth_and_variance = 1.0 / (0.75 + (inverse_depth_and_variance * 0.25));
	Mat low_inverse_depth_and_variance;
	resize(inverse_depth_and_variance, low_inverse_depth_and_variance, Size(80, 60));
	tracker.track(current_grayscale, low_inverse_depth_and_variance, new_grayscale);
}