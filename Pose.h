#ifndef POSE_H
#define POSE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class Pose
{
	/**
	 * A pose is a combination of a 3D rotation and 3D translation.
	 */

private:
	/**
	 * Translations and Rotations in OpenCV matrix form.
	 */
	Mat t;
	Mat R;


public:
	/** 
	 * creates a new Pose object with the identity transformation. 
	 */
	Pose();

	/**
	 * creates a new Pose object with translation / Euler angle rotations.
	 * Variables x, y, and z, refer to translations.
	 * Variables a, b, and g are short for alpha, beta, and gamma, and refer to
	 * rotations on the x, y', and z'' axes in units of radians.
	 */
	Pose(double _x, double _y, double _z, double _a, double _b, double _g);

	/**
	 * Todo
	 */
	Pose(Mat pose_values);

	Pose(Matx31d _t, Matx33d _R);

	void set_from_twist(double _x, double _y, double _z, double _a, double _b, double _g);

	/**
	 * compose_with takes another pose and composes them in standard immediate
	 * application / left compositional / matrix style.
	 *
	 * That is, a.compose_with(b) means b is applied, then a, just as it was 
	 * a(b(x)).
	 */
	Pose compose_with(Pose other);

	/**
	 * Returns the pose difference - the pose it takes to go from other to self.
	 */
	Pose relative_to(Pose other);

	/**
	 * Transforms a set of openCV points (represented as a Mat)
	 */
	Mat observe(Mat points);

	/**
	 * TODO
	 */
	void print();
};

#endif