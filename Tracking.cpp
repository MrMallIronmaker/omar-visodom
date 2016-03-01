#include "Tracking.h"
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;

#define DEPTH_UNAVAILABLE 0.0 
// also, countNonZero is used, so if this changes, update those functions too.
#define DEGREES_OF_FREEDOM 5

double Tracking::bilinearly_interpolate(Mat grayscale, double x, double y)
{
	int x_whole = (int) x;
	int y_whole = (int) y;
	// int a_whole = (int) hahaha;
	double x_dec = x - x_whole;
	double y_dec = y - y_whole;

	// bound-detect.
	int x0 = MAX(0, MIN(x_whole, grayscale.cols - 1));
	int x1 = MAX(0, MIN(x_whole + 1, grayscale.cols - 1));
	int y0 = MAX(0, MIN(y_whole, grayscale.rows - 1));
	int y1 = MAX(0, MIN(y_whole + 1, grayscale.rows - 1));
	// yes it's possible to have both c0 and c1 referring to the same value,
	// that's OK because extrapolation method is nearest pixel.

	double intensity = grayscale.at<double>(y1, x1) * y_dec * x_dec
	                 + grayscale.at<double>(y0, x1) * (1 - y_dec) * x_dec
	                 + grayscale.at<double>(y1, x0) * y_dec * (1 - x_dec)
	                 + grayscale.at<double>(y0, x0) * (1 - y_dec) * (1 - x_dec);
	return intensity;
}

Mat Tracking::resize_depth_and_variance(Mat inverse_depth_and_variance, Size size)
{
	// assume equal variance because trying to make sure it's OK
	Mat inverse_depth(inverse_depth_and_variance.rows, 
		inverse_depth_and_variance.cols,
		CV_64FC1);

	int fromTo[2] = {0, 0};
	mixChannels(&inverse_depth_and_variance, 1, &inverse_depth, 1, fromTo, 1);

	Mat fin;
	resize(inverse_depth, fin, size);

	return fin; //rename
}

Mat Tracking::compute_jacobian(Mat grayscale, Mat inverse_depth)
{
	assert(grayscale.rows == inverse_depth.rows && grayscale.cols == inverse_depth.cols);
	int pixel_count = countNonZero(inverse_depth); 
	double this_f_length = grayscale.rows * f_length / (double) image_height;

	Mat jacobian = Mat(pixel_count, 6, CV_64FC1);

	int i = 0;
	for (int row = 0; row < grayscale.rows; row++)
	{
		for (int col = 0; col < grayscale.cols; col++)
		{
			if (inverse_depth.at<double>(row, col) != DEPTH_UNAVAILABLE)
			{
				// handle boundaries.
				int x0 = MAX(0, MIN(col - 1, grayscale.cols - 1));
				int x1 = MAX(0, MIN(col + 1, grayscale.cols - 1));
				int y0 = MAX(0, MIN(row - 1, grayscale.rows - 1));
				int y1 = MAX(0, MIN(row + 1, grayscale.rows - 1));

				double x_grad = 0.5 * this_f_length * (grayscale.at<double>(row, x1) - 
												  (double)grayscale.at<double>(row, x0));
				double y_grad = 0.5 * this_f_length * (grayscale.at<double>(y1, col) - 
												  (double)grayscale.at<double>(y0, col));
				Matx12d pixel_gradient(x_grad, y_grad);

				// extract into matrix-making method
				double u_f = (col - (grayscale.cols / 2.0)) / this_f_length;
				double v_f = (row - (grayscale.rows / 2.0)) / this_f_length;
				double d = inverse_depth.at<double>(row, col);
				double u_f2 = u_f * u_f;
				double v_f2 = v_f * v_f;
				double ufvf = u_f * v_f;
				Matx<double, 2, 6> pixel_jacobian(
						-d,  0, u_f*d,    ufvf, -1-u_f2,  v_f,
						 0, -d, v_f*d, +1+v_f2,   -ufvf, -u_f);

				Matx16d pixel_pose_shift = pixel_gradient * pixel_jacobian;

				for (int j = 0; j < 6; j++)
				{
					jacobian.at<double>(i, j) = pixel_pose_shift(j);
				}

				i++;
			}
		}
	}

	return jacobian;
}

Mat Tracking::compute_residuals(Mat current_grayscale, Mat inverse_depth,
								Mat new_grayscale, Pose estimate)
{
	// A) Reconstruct 3D points from pixels
	int pixel_count = countNonZero(inverse_depth); 

	Mat points = Mat(3, pixel_count, CV_64FC1);
	Mat cut_current_grayscale = Mat(pixel_count, 1, CV_64FC1);
	double c_x = (current_grayscale.cols - 1) / 2.0;
	double c_y = (current_grayscale.rows - 1) / 2.0;

	int i = 0;
	for (int row = 0; row < current_grayscale.rows; row++)
	{
		for (int col = 0; col < current_grayscale.cols; col++)
		{
			if (inverse_depth.at<double>(row, col) != DEPTH_UNAVAILABLE)
			{
				points.at<double>(0, i) = (col - c_x) / (f_length * inverse_depth.at<double>(row, col));
				points.at<double>(1, i) = (row - c_y) / (f_length * inverse_depth.at<double>(row, col));
				points.at<double>(2, i) = 1.0 / inverse_depth.at<double>(row, col);
				cut_current_grayscale.at<double>(i) = current_grayscale.at<double>(row, col);
				i++;
			}
		}
	}
	//imshow( "Display window", abs(cut_current_grayscale.reshape(1, current_grayscale.rows)) / 256);
	////waitKey(0);

	// B) Transform old points into new coordinates
	Mat points_in_new_frame = estimate.observe(points);

	// C) Reach into the new image and grab the value at the new estimated positions.
	Mat estimated_new = Mat(pixel_count, 1, CV_64FC1);
	i = 0;
	for (int row = 0; row < new_grayscale.rows; row++)
	{
		for (int col = 0; col < new_grayscale.cols; col++)
		{
			if (inverse_depth.at<double>(row, col) != DEPTH_UNAVAILABLE)
			{
				double x = (points_in_new_frame.at<double>(0, i) 
							/ points_in_new_frame.at<double>(2, i)) * f_length + c_x;
				double y = (points_in_new_frame.at<double>(1, i) 
							/ points_in_new_frame.at<double>(2, i)) * f_length + c_y;
				estimated_new.at<double>(0, i) = bilinearly_interpolate(new_grayscale, x, y);
				i++;
			}
		}
	}
	//imshow( "Display window", abs(estimated_new.reshape(0, current_grayscale.rows)) / 256);
	////waitKey(0);

	// D) Compare new values at estimated positions with old image.
	Mat errors = estimated_new - cut_current_grayscale;
	return errors.clone();
}

Mat Tracking::compute_weights(Mat residuals)
{
	Mat weights;

	// A) Calculate initial variance
	Mat squared_residuals = residuals.mul(residuals);
	double new_variance = mean(squared_residuals)[0];
	double old_variance;
	do {
		old_variance = new_variance;
		// B) Calculate weights vector
		weights = (DEGREES_OF_FREEDOM + 1) 
						/ (DEGREES_OF_FREEDOM + squared_residuals / old_variance);
		// C) Calculate new variance
		new_variance = mean(weights.mul(squared_residuals))[0];
	}
	// D) Test if the variance has converged. (delta < 0.5%)
	while (abs(new_variance - old_variance) > 0.005 * new_variance);

	return weights;
}

Pose Tracking::iterate_lk_tracking(Mat current_grayscale, Mat inverse_depth, 
	                     Mat new_grayscale, Pose estimate, Mat jacobian)
{
	// 1) Compute residuals
	Mat residuals = compute_residuals(current_grayscale, inverse_depth, new_grayscale, estimate);
	
	double error = sum(residuals.mul(residuals))[0];
	//printf("\n\nError: %f", error);
	if (error < 0.000001)
		return estimate;

	//imshow( "Display window", abs(residuals.reshape(0, current_grayscale.rows)) / 256);
	////waitKey(0);

	// 2) Compute the matrix of weights [represented in 1D]
	Mat weights = compute_weights(residuals);
	
	//imshow( "Display window", abs(weights.reshape(0, current_grayscale.rows)) / 2);
	////waitKey(0);

	// 3) Solve the matrix equation.
	Mat pose_values;
	{
		// A) Compute J.T * W (it's used twice)
		//Mat jtw = jacobian.t().mul(repeat(weights.t(), 6, 1));
		Mat jtw = jacobian.t();

		// B) Compute the Gauss-Newton inverse Hessian, (J.T * W * J)^-1
		Mat inv_hess = Mat(6, 6, CV_64FC1);
		double success = invert(jtw * jacobian, inv_hess);
		assert(success > 0.0);
		// C) Compute the final pose values, - H^-1 * J.T * W * r

		pose_values = -inv_hess * jtw * residuals;
	}

	return Pose(pose_values);
}

Pose Tracking::track(Mat current_grayscale, Mat inverse_depth_and_variance, Mat new_grayscale)
{

	// initialize pose, sizes, and iterations
	Pose pose_guess, identity;
	int pyramid_levels = 3;
	int steps[] = {8, 3, 3};
	Size sizes[] = {Size(80, 60), Size(160, 120), Size(320, 240)};

	for (int i = 0; i < pyramid_levels; i++)
	{
		Mat resized_current_grayscale, resized_new_grayscale;
		resize(current_grayscale, resized_current_grayscale, sizes[i]);
		resize(new_grayscale, resized_new_grayscale, sizes[i]);
		Mat resized_inverse_depth = resize_depth_and_variance(inverse_depth_and_variance, sizes[i]);
		Mat resized_jacobian = compute_jacobian(resized_current_grayscale, resized_inverse_depth);

		for (int j = 0; j < steps[i]; j++)
		{
			// input for pose_guess should be the transform from I1 to I2
			Pose increment = iterate_lk_tracking(resized_current_grayscale, resized_inverse_depth, 
				resized_new_grayscale, pose_guess, resized_jacobian);
			Pose inverse_increment = identity.relative_to(increment);
			pose_guess = pose_guess.compose_with(inverse_increment);
		}
	}

	return pose_guess;

}

Tracking::Tracking(double _f_length, int _image_width, int _image_height)
{
	f_length = _f_length;
	image_width = _image_width;
	image_height = _image_height;
}