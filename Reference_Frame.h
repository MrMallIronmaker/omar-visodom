#ifndef REFERENCE_FRAME_H
#define REFERENCE_FRAME_H

#include "Pose.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Reference_Frame 
{
private:
	Reference_Frame *next; //!< Next oldest frame available
	unsigned int age; //!< Age of frame, from zero upwards.
	Pose pose; //!< Estimated absolute pose of frame
	Mat grayscale_image; //!< Grayscale openCV mat

	Reference_Frame(Reference_Frame *_next, unsigned int _age, Pose _pose, Mat _grayscale_image);
};

class Reference_Frames
{
private:
	Reference_Frame *head, *tail;
	unsigned int current_age;
public:
	Reference_Frames();

	/** 
	 * Given a particular reference frame pointer, position in the current image,
	 * and the inverse depth and variance of inverse depth,
	 * use stereo matching to estimate a new inverse depth.
	 */
	double estimate_inverse_depth(Reference_Frame *ref, int x, int y, 
								  double inverse_depth, double variance);

	/**
	 * Add the given OpenCV grayscale image to the list of reference frames.
	 */
	void add_frame(Mat image, Pose pose);

	/**
	 * Todo: add reference frame removal function.
	 * If there's nothing like that in the papers, just throw out all frames that aren't used in a certain cycle.
	 */
};

#endif