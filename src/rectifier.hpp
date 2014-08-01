#ifndef RECTIFIER_H
#define RECTIFIER_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "camera.hpp"
#include "image.hpp"

using namespace cv;
namespace visuallocalization {

/* The information extracted from the two cameras involved in the rectification
 * assume that the robot changes only the yaw angle. Moreover, the plane formed
 * by X and Y robot axes is parallel to the plane of motion, and the vehicle is
 * always lying on the ground, since the position of the camera in the robot
 * frame is used to extract the height of the camera (distance from the ground).
 * Use of particular vehicles (quadcopters or robots subjected to perturbation
 * in the stability) will need the introduction of the concept of virtual 
 * vehicle. See description of Camera object for further details.
 */
class Rectifier {
	private:
		/*** variables ***/
		/* rotation matrix from virtual to actual camera */
		Eigen::Matrix<float, 3, 3> ca_R_cv; 
		/* rotation matrix from actual to virtual camera */
		Eigen::Matrix<float, 3, 3> cv_R_ca;
		/* position of actual camera in the virtual camera frame */
		Eigen::Matrix<float, 3, 1> cv_tca;
		/* position of virtual camera in the actual camera frame */
		Eigen::Matrix<float, 3, 1> ca_tcv;
		/* unit vector normal to the plane in the actual camera frame */
		Eigen::Matrix<float, 3, 1> Na;
		/* distance of the actual camera from the plane */
		float da;
		/* distance of the virtual camera from the plane */
		float dv;

		bool verbose;

		/*** methods ***/

		/* Search for the first point that could be inside the area covered by 
		 * the virtual camera. The upper Y bound of the area is given.
		 *
		 * @Ia: list of points
		 * @Npts: number of points
		 * @y_bound: upper bound of the area
		 *
		 * returns the index to start with or -1
		 */
		int searchForFirstImagePoint(
			const ListPoints &Ia
			, const int Npts
			, const float y_bound) const;

	public:
		/*** constructor and destructor ***/
		Rectifier();
		Rectifier(const Camera &actual, const Camera &virt);

		/*** methods ***/

		/*
		 * Update the rotation matrices and position of the cameras.
		 *
		 * @actual: the actual camera
		 * @virt: the virtual camera
		 */
		void updateTransformation(const Camera &actual, const Camera &virt);

		/*
		 * Update the respective position of the cameras.
		 *
		 * @actual: the actual camera
		 * @virt: the virtual camera
		 */
		void updatePosition(const Camera &actual, const Camera &virt);

		/*
		 * Get the rotation matrix from virtual to actual camera frame.
		 *
		 * @ca_R_cv: will contain the rotation matrix
		 */
		const Eigen::Matrix<float, 3, 3> & getRotation_ca_R_cv() const;

		/*
		 * Get the rotation matrix from actual to virtual camera frame.
		 *
		 * @cv_R_ca: will contain the rotation matrix
		 */
		const Eigen::Matrix<float, 3, 3> & getRotation_cv_R_ca() const;

		/*
		 * Get the position of the actual camera in the virtual frame.
		 *
		 * @cv_tca: will contain the position
		 */
		const Eigen::Matrix<float, 3, 1> & getPosition_v_ta() const;

		/*
		 * Get the position of the virtual camera in the actual frame.
		 *
		 * @ca_tcv: will contain the position
		 */
		const Eigen::Matrix<float, 3, 1> & getPosition_a_tv() const;

		/*
		 * Compute the four corners of the projection of the virtual camera in
		 * the actual image.
		 * Update of transormation between actual camera and virtual camera is
		 * assumed to be already done. Call updateTransformation, or 
		 * updatePosition before this function.
		 *
		 * @actual: the actual camera
		 * @virt: the virtual camera
		 * @corners: the four corners that are returned (up_left, up_right,
		 *           down_left, down_right).
		 */
		void getActualImageCorners(
			const Camera &actual
			, const Camera &virt
			, Point corners[4]) const;

		/*
		 * Compute the four corners of the projection of the virtual camera in
		 * the actual image.
		 * Update of transormation between actual camera and virtual camera is
		 * assumed to be already done. Call updateTransformation, or 
		 * updatePosition before this function. And insert the region in the
		 * image.
		 *http://www.aeonity.com/ab/games/action-adventure/watch-out-behind-you-hunter.php
		 * @actual: the actual camera
		 * @virt: the virtual camera
		 * @corners: the four corners that are returned (up_left, up_right,
		 *           down_left, down_right).
		 * @image: the image in which the region is drawn
		 */
		void getActualImageCorners(
			const Camera &actual
			, const Camera &virt
			, Point corners[4]
			, Image &image) const;

		/*
		 * Determine the virtual image captured from the virtual camera based 
		 * on the image captured from the actual camera.
		 * Update of transormation between actual camera and virtual camera is
		 * assumed to be already done. Call updateTransformation, or 
		 * updatePosition before this function.
		 *
		 * @actual: the actual camera
		 * @virt: the virtual camera
		 *
		 * Returns the number of points in the virtual image
		 */
		int rectification(const Camera &actual, Camera &virt) const;
		int rectification(const Camera &actual, Camera &virt, const Mat &imgact, Mat &imgvirt) const;
		cv::Point point_rectification(const Camera &actual, Camera &virt,int width, int height, const cv::Point &p) const;

		/* Set verbose mode
		 *
		 * @verbose: the verbose mode
		 */
		void setVerbose(bool verbose);
};

} /* end namespace */

#endif /* RECTIFIER_H */
