#ifndef TRACKING_H
#define TRACKING_H

#include "Pose.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Tracking
{
private:
	double f_length; //< Focal length, in pixels
	int image_height; //< input image height, in pixels.
	int image_width; //< input image width, in pixels.

	/**
	 * The sub-step that track uses multiple times to refine the pose. The magic happens here.
	 */
	Pose iterate_lk_tracking(Mat current_grayscale, Mat inverse_depth, Mat new_grayscale, Pose estimate, Mat jacobian);

	/**
	 * Returns the xyzabg Jacobian for the specified image and camera parameters, size (w*h) rows and 6 columns.
	 */
	Mat compute_jacobian(Mat grayscale, Mat inverse_depth);

	/**
	 * todo
	 */
	Mat compute_residuals(Mat current_grayscale, Mat inverse_depth,
								Mat new_grayscale, Pose estimate);

	Mat compute_weights(Mat residuals);

	Mat resize_depth_and_variance(Mat inverse_depth_and_variance, Size size);

	/**
	 * Given the image and decimal-valued x and y coordinates, find the
	 * bilinearly interpolated value at that point. Pixels beyond the border
	 * are handled by finding the closest certain pixel in the image.
	 */
	double bilinearly_interpolate(Mat grayscale, double x, double y);
public:
	/**
	 * Given the current image, depth, and new image, estimate the new pose to subpixel accuracy.
	 */
	Pose track(Mat current_grayscale, Mat inverse_depth_and_variance, Mat new_grayscale);

	Tracking(double _f_length, int _image_width, int _image_height);
};

#endif