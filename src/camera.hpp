#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include "image.hpp"

namespace visuallocalization {

/* Camera is characterized by rotation from robot coordinate system to camera 
 * coordinate system and position in the robot frame. The axes of the robot are
 * such that X points forward, Y points to the left side, and Z points upwards.
 * The center of the robot is in the center of the robot frame for what 
 * concerns X and Y, while the zero on the Z axis corresponds to the ground 
 * level (when the vehicle lies on the ground, i.e. quadcopter).
 * The robot is assumed to change only the yaw angle. In case the last 
 * assumption does not hold true, consider to add information about rotation and
 * position with reference to a virtual robot which respect the above 
 * assumptions, and let the external user to take care of changes in attitude 
 * (and position if needed) of the camera in case the real vehicle has a 
 * different configuration from the virtual one.
 */
class Camera {
	private:
		/*** variables ***/
		/* position vector in robot frame */
		Eigen::Matrix<float, 3, 1> tR;
		/* rotation matrix from robot to camera frame */
		Eigen::Matrix<float, 3, 3> c_R_r;
		float focal;
		float roll;
		float pitch;
		float yaw;

	public:
		/*** variables ***/
		Image *image;

		/*** constructor and destructor ***/
		Camera();
		Camera(const int width, const int height);
		Camera(const Camera &camera);
		~Camera();

		/*** methods ***/

		/*
		 * Determine the position of the camera in the robot coordinate system,
		 * provided a reference camera with known position in the robot 
		 * coordinate system, and know position of the reference camera in the
		 * camera coordinate system. Rotation between camera and robot is known.
		 *
		 * @camera: the reference camera
		 * @position: the position of the reference camera in the camera 
		 *            coordinate system
		 */
		void determineCameraRobotPosition(
			const Camera &camera
			, const Eigen::Matrix<float, 3, 1>  &v_tc);

		/*
		 * Set orientation of the camera w.r.t. the robot. Compute the rotation 
		 * matrix c_R_r.
		 *
		 * @roll: roll angle, rotation along axis x
		 * @pitch: pitch angle, rotation along axis y
		 * @yaw: yaw angle, rotation along axis z
		 */
		void setRotation(const float roll, const float pitch, const float yaw);

		/*
		 * Set position of the camera w.r.t. the robot.
		 *
		 * @position: 3x1 matrix (vector) expressing the 3 axes coordinates
		 */
		void setPosition(const Eigen::Matrix<float, 3, 1> &position);

		/*
		 * Set focal lenght of the camera.
		 *
		 * @focal: focal lenght
		 */
		int setFocal(const float focal);

		/*
		 * Returns the 3 angles expressing the orientation of the camera w.r.t. 
		 * the robot.
		 *
		 * @roll: will contain value of roll angle
		 * @pitch: will contain value of pitch angle
		 * @yaw: will contain value of yaw angle
		 */
		void getRotation(float &roll, float &pitch, float &yaw) const;

		/*
		 * Returns the rotation matrix (3x3) expressing the rotation from the 
		 * robot to the camera frame. The result is a read-only object.
		 */
		const Eigen::Matrix<float, 3, 3> & getRotation() const;

		/*
		 * Returns position of the camera w.r.t. the robot. The result is a
		 * read-only object.
		 */
		const Eigen::Matrix<float, 3, 1> & getPosition() const;

		/*
		 * Returns the focal lenght.
		 */
		float getFocal() const;

		/*
		 * Returns the resolution of the camera.
		 *
		 * @width: will contain the width of the image
		 * @height: will contain the height of the image
		 */
		void getImageSize(int &width, int &height) const;
};

} /* end namespace */

#endif /* CAMERA_H */
