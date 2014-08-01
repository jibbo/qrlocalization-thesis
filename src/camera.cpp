#include <stdlib.h>

#include "camera.hpp"
#include "debug.hpp"

using namespace visuallocalization;

Camera::Camera()
{
	image = new Image();

	tR = Eigen::MatrixXf::Zero(3, 1);
	c_R_r = Eigen::MatrixXf::Zero(3, 3);

	focal = 0;
	roll = 0;
	pitch = 0;
	yaw = 0;
}

Camera::Camera(const int width, const int height)
{
	image = new Image(width, height);

	tR = Eigen::MatrixXf::Zero(3, 1);
	c_R_r = Eigen::MatrixXf::Zero(3, 3);

	focal = 0;
	roll = 0;
	pitch = 0;
	yaw = 0;
}

Camera::Camera(const Camera &camera)
{
	int width, height;
	camera.getImageSize(width, height);
	image = new Image(width, height);

	focal = camera.getFocal();

	float roll, pitch, yaw;
	camera.getRotation(roll, pitch, yaw);
	setRotation(roll, pitch, yaw);

	const Eigen::Matrix<float, 3, 1> &position = camera.getPosition();
	setPosition(position);
}

Camera::~Camera()
{
	if (image != 0) {
		delete image;
		image = 0;
	}
}

void Camera::determineCameraRobotPosition(
const Camera &camera
, const Eigen::Matrix<float, 3, 1>  &v_tc)
{
	const Eigen::Matrix<float, 3, 1> r_tc = camera.getPosition();
	const Eigen::Matrix<float, 3, 3> v_R_r = this->getRotation();
	const Eigen::Matrix<float, 3, 3> r_R_v = v_R_r.transpose();

	/* vector from v to c in robot coordinate system */
	const Eigen::Matrix<float, 3, 1> r_tvc = r_R_v * v_tc;

	const Eigen::Matrix<float, 3, 1> r_tv = r_tc - r_tvc;
	this->setPosition(r_tv);
}

void Camera::setRotation(const float roll, const float pitch, const float yaw)
{
	this->roll = roll;
	this->pitch = pitch;
	this->yaw = yaw;

	Eigen::Matrix<float, 3, 3> Rx, Ry, Rz;
	Rx << 1, 0,          0,
	      0, cos(roll),  sin(roll),
	      0, -sin(roll), cos(roll);
	Ry << cos(pitch), 0, -sin(pitch),
	      0,          1, 0,
	      sin(pitch), 0, cos(pitch);
	Rz << cos(yaw),  sin(yaw), 0,
	      -sin(yaw), cos(yaw), 0,
	      0,         0,        1;

	/* c_R_r <- Rx * Ry * Rz */
	this->c_R_r = Rx * Ry * Rz;
}

void Camera::setPosition(const Eigen::Matrix<float, 3, 1> &position)
{
	tR(0, 0) = position(0, 0);
	tR(1, 0) = position(1, 0);
	tR(2, 0) = position(2, 0);
}

int Camera::setFocal(const float focal)
{
	this->focal = focal;
	return 0;
}

void Camera::getRotation(float &roll, float &pitch, float &yaw)
const
{
	roll = this->roll;
	pitch = this->pitch;
	yaw = this->yaw;
}

const Eigen::Matrix<float, 3, 3> & Camera::getRotation()
const
{
	return c_R_r;
}

const Eigen::Matrix<float, 3, 1> & Camera::getPosition()
const
{
	return tR;
}

float Camera::getFocal()
const
{
	return this->focal;
}

void Camera::getImageSize(int &width, int &height)
const
{
	width = image->getWidth();
	height = image->getHeight(); 
}
