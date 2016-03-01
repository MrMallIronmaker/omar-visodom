#include "Pose.h"
#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;

Pose::Pose()
{
	t = Mat(3, 1, CV_64FC1, Scalar(0.0));
	R = Mat::eye(3, 3, CV_64FC1);
}

Pose::Pose(Mat pose_values)
{
	// redo this according to 242 specs soon.
	set_from_twist(pose_values.at<double>(0),
				   pose_values.at<double>(1),
				   pose_values.at<double>(2),
				   pose_values.at<double>(3),
				   pose_values.at<double>(4),
				   pose_values.at<double>(5));
}

Pose::Pose(double _x, double _y, double _z, double _a, double _b, double _g)
{
	set_from_twist(_x, _y, _z, _a, _b, _g);
}

Pose::Pose(Matx31d _t, Matx33d _R)
{
	t = Mat(_t);
	R = Mat(_R);
}

void Pose::set_from_twist(double _x, double _y, double _z, double _a, double _b, double _g)
{
	t = (Mat_<double>(3, 1) << _x, _y, _z);
	Mat RX = (Mat_<double>(3, 3) <<
              1,        0,        0,
              0,  cos(_a),  -sin(_a),
              0, sin(_a),  cos(_a)
             );
	Mat RY = (Mat_<double>(3, 3) <<
              cos(_b), 0, sin(_b),
                    0, 1,        0,
              -sin(_b), 0,  cos(_b)
             );
	Mat RZ = (Mat_<double>(3, 3) <<
               cos(_g), -sin(_g), 0,
              sin(_g), cos(_g), 0,
                     0,        0, 1
             );

	R = RZ * RY * RX; // note that RX is applied first when some point or transform comes through.
}

void Pose::print()
{
	// print in twist form.
	double a = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
	double b = atan2(-R.at<double>(2, 0), 
		sqrt(pow(R.at<double>(2, 1), 2) + pow(R.at<double>(2, 2), 2)));
	double g = atan2(R.at<double>(1, 0), R.at<double>(0, 0));

	cout << "\nX translation (right): " << t.at<double>(0)
	     << "\nY translation (down): " << t.at<double>(1)
	     << "\nZ translation (forward): " << t.at<double>(2)
	     << "\nAlpha rotation (pitch up): " << a
	     << "\nBeta rotation (yaw right): " << b
	     << "\nGamma rotation (roll clockwise): " << g
	     << "\n";
}

Pose Pose::compose_with(Pose other)
{
	// transform this translation and add it to the original.
	Mat q = (other.R.t() * t);
	Pose new_pose;
	new_pose.t = other.t + q;
	new_pose.R = R * other.R;
	return new_pose;
}

Pose Pose::relative_to(Pose other)
{
	// Returns the pose difference - the pose it takes to go from other to self.
	Pose new_pose;
	new_pose.t = other.R * (t - other.t);
	new_pose.R = R * other.R.t();
	return new_pose;
}

Mat Pose::observe(Mat points)
{
	// Returns the position of each point relative to this camera pose.
	// first translate, then rotate.
	// TODO: ensure points is a 3xn matrix
	Mat translate = 
	repeat(t, 1, points.cols);
	return R * (points + translate);
}
