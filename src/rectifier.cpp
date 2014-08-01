#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "rectifier.hpp"
#include "debug.hpp"

using namespace visuallocalization;


Rectifier::Rectifier()
{
	this->da = 0;
	this->dv = 0;
	verbose = 0;
}

Rectifier::Rectifier(const Camera &actual, const Camera &virt)
{
	verbose = 0;
	this->updateTransformation(actual, virt);
}

void Rectifier::updateTransformation(const Camera &actual, const Camera &virt)
{
	Eigen::Matrix<float, 3, 3> r_R_ca;
	Eigen::Matrix<float, 3, 3> r_R_cv;

	/* get rotation matrix for actual camera */
	const Eigen::Matrix<float, 3, 3> &ca_R_r = actual.getRotation();
	r_R_ca = ca_R_r.transpose();

	/* get rotation matrix for virtual camera */
	const Eigen::Matrix<float, 3, 3> &cv_R_r = virt.getRotation();
	r_R_cv = cv_R_r.transpose();

	/* compute rotation matrix between the two cameras */
	this->ca_R_cv = ca_R_r * r_R_cv;
	this->cv_R_ca = cv_R_r * r_R_ca;

	/* get position of the cameras in the robot coordinate system */
	const Eigen::Matrix<float, 3, 1> &r_tca = actual.getPosition();
	const Eigen::Matrix<float, 3, 1> &r_tcv = virt.getPosition();

	/* compute cameras position */
	this->cv_tca = cv_R_r * (r_tca - r_tcv);
	this->ca_tcv = ca_R_r * (r_tcv - r_tca);

	/* normal vector normal to the plane in virtual camera frame */
	Eigen::Matrix<float, 3, 1> Nv;
	Nv(0,0) = 0;
	Nv(1,0) = 0;
	Nv(2,0) = 1;
	this->Na = this->ca_R_cv * Nv;

	/* Distance from the ground of the virtual camera. The way it is computed 
	 * is fine as long as the virtual camera is posed in such way the z axes
	 * of the camera is orthogonal to the ground.
	 */
	this->dv = r_tcv(2,0);

	/* Distance from the ground of the actual camera */
	this->da = this->dv - Nv.dot(this->cv_tca); 
}

void Rectifier::updatePosition(const Camera &actual, const Camera &virt)
{
	/* get rotation matrix for actual camera */
	const Eigen::Matrix<float, 3, 3> &ca_R_r = actual.getRotation();

	/* get rotation matrix for virtual camera */
	const Eigen::Matrix<float, 3, 3> &cv_R_r = virt.getRotation();

	/* get position of the cameras in the robot coordinate system */
	const Eigen::Matrix<float, 3, 1> r_tca = actual.getPosition();
	const Eigen::Matrix<float, 3, 1> r_tcv = virt.getPosition();

	/* compute cameras position */
	this->cv_tca = cv_R_r * (r_tca - r_tcv);
	this->ca_tcv = ca_R_r * (r_tcv - r_tca);

	/* normal vector normal to the plane in virtual camera frame */
	Eigen::Matrix<float, 3, 1> Nv;
	Nv(0,0) = 0;
	Nv(1,0) = 0;
	Nv(2,0) = 1;

	/* Distance from the ground of the virtual camera. The way it is computed 
	 * is fine as long as the virtual camera is posed in such way the z axes
	 * of the camera is orthogonal to the ground.
	 */
	this->dv = r_tcv(2,0);

	/* Distance from the ground of the actual camera */
	this->da = this->dv - Nv.dot(this->cv_tca);
}

const Eigen::Matrix<float, 3, 3> & Rectifier::getRotation_ca_R_cv()
const
{
	return this->ca_R_cv;
}

const Eigen::Matrix<float, 3, 3> & Rectifier::getRotation_cv_R_ca()
const
{
	return this->cv_R_ca;
}

const Eigen::Matrix<float, 3, 1> & Rectifier::getPosition_v_ta()
const
{
	return this->cv_tca;
}

const Eigen::Matrix<float, 3, 1> & Rectifier::getPosition_a_tv()
const
{
	return this->ca_tcv;
}

void Rectifier::getActualImageCorners(
const Camera &actual
, const Camera &virt
, Point corners[4])
const
{
	/* images pointers */
	Image *v_img = virt.image;

	/* dimensions of the virtual image */
	int sX = v_img->getWidth();
	int sY = v_img->getHeight();
	int hsX = sX / 2;
	int hsY = sY / 2;

	/* get focal lenght of the two cameras */
	float focal_a = actual.getFocal();
	float focal_v = virt.getFocal();

	/* Consider the four corners of the virtual image and compute the potential
	 * points in the original image. This will allow to not consider all the 
	 * points which won't be possible to find in the virtual image
	 */
	Eigen::Matrix<float, 3, 1> cv_p_upleft;
	Eigen::Matrix<float, 3, 1> cv_p_upright;
	Eigen::Matrix<float, 3, 1> cv_p_downleft;
	Eigen::Matrix<float, 3, 1> cv_p_downright;

	/* Points in the 3D space in the virtual camera frame
	 * The four corners of the virtual image are considered: cv_p
	 */
	cv_p_upleft(0,0) = (-hsX * dv) / focal_v;
	cv_p_upleft(1,0) = (-hsY * dv) / focal_v;
	cv_p_upleft(2,0) = dv;
	cv_p_upright(0,0) = (hsX * dv) / focal_v;
	cv_p_upright(1,0) = (-hsY * dv) / focal_v;
	cv_p_upright(2,0) = dv;
	cv_p_downleft(0,0) = (-hsX * dv) / focal_v;
	cv_p_downleft(1,0) = (hsY * dv) / focal_v;
	cv_p_downleft(2,0) = dv;
	cv_p_downright(0,0) = (hsX * dv) / focal_v;
	cv_p_downright(1,0) = (hsY * dv) / focal_v;
	cv_p_downright(2,0) = dv;

	/* Convertion of the points in the actual camera frame 
	 * ca_p = ca_R_cv * cv_p - ca_R_cv * cv_tca
	 */
	Eigen::Matrix<float, 3, 1> ca_p_upleft;
	Eigen::Matrix<float, 3, 1> ca_p_upright;
	Eigen::Matrix<float, 3, 1> ca_p_downleft;
	Eigen::Matrix<float, 3, 1> ca_p_downright;
	ca_p_upleft = ca_R_cv * (cv_p_upleft - cv_tca);
	ca_p_upright = ca_R_cv * (cv_p_upright - cv_tca);
	ca_p_downleft = ca_R_cv * (cv_p_downleft - cv_tca);
	ca_p_downright = ca_R_cv * (cv_p_downright - cv_tca);

	/* compute the four corners in the actual image */
	corners[0](0,0) = (int)(focal_a
		* (ca_p_upleft(0,0) / ca_p_upleft(2,0)));
	corners[0](1,0) = (int)(focal_a
		* (ca_p_upleft(1,0) / ca_p_upleft(2,0)));
	corners[1](0,0) = (int)(focal_a
		* (ca_p_upright(0,0) / ca_p_upright(2,0)));
	corners[1](1,0) = (int)(focal_a
		* (ca_p_upright(1,0) / ca_p_upright(2,0)));
	corners[2](0,0) = (int)(focal_a
		* (ca_p_downleft(0,0) / ca_p_downleft(2,0)));
	corners[2](1,0) = (int)(focal_a
		* (ca_p_downleft(1,0) / ca_p_downleft(2,0)));
	corners[3](0,0) = (int)(focal_a
		* (ca_p_downright(0,0) / ca_p_downright(2,0)));
	corners[3](1,0) = (int)(focal_a
		* (ca_p_downright(1,0) / ca_p_downright(2,0)));
}

void Rectifier::getActualImageCorners(
const Camera &actual
, const Camera &virt
, Point corners[4]
, Image &image)
const
{
	getActualImageCorners(actual, virt, corners);

	Image *a_img = actual.image;
	int asX = a_img->getWidth();
	int asY = a_img->getHeight();
	int ahsX = asX / 2;
	int ahsY = asY / 2;
	Point offset_img;
	offset_img << ahsX, ahsY;

	image.createImage(WHITE_ON_BLACK);
	image.addLine(corners[0] + offset_img, corners[1] + offset_img);
	image.addLine(corners[1] + offset_img, corners[3] + offset_img);
	image.addLine(corners[3] + offset_img, corners[2] + offset_img);
	image.addLine(corners[2] + offset_img, corners[0] + offset_img);
	image.show("Rectification Area", WAIT_FOR_KEY);
	cv::destroyWindow("Rectification Area");
}

int Rectifier::rectification(const Camera &actual, Camera &virt, const Mat &a_img, Mat &v_img)
const
{
	/* dimensions of the virtual image */
	int sX = v_img.cols;
	int sY = v_img.rows;
	int hsX = sX / 2;
	int hsY = sY / 2;

	/* get focal lenght of the two cameras */
	float focal_a = actual.getFocal();
	float focal_v = virt.getFocal();

	/* Compute some constants used for rectification */
	float foc_d = focal_v / dv;
	float temp1_0 = foc_d * cv_tca(0,0);
	float temp1_1 = foc_d * cv_tca(1,0);
	float temp2 = foc_d * da;
	float temp3;
	float na_1 = Na(0,0);
	float na_2 = Na(1,0);
	float na_3 = Na(2,0);
	float est_ca_r_cv_1_1 = ca_R_cv(0,0);
	float est_ca_r_cv_2_1 = ca_R_cv(1,0);
	float est_ca_r_cv_3_1 = ca_R_cv(2,0);
	float est_ca_r_cv_1_2 = ca_R_cv(0,1);
	float est_ca_r_cv_2_2 = ca_R_cv(1,1);
	float est_ca_r_cv_3_2 = ca_R_cv(2,1);

	int newNpts = 0;
	int newNpts_norep = 0;
	float xp, yp;
	int x, y;

	for (int i = 0; i < a_img.rows; i++) {
		for(int j=0; j< a_img.cols;j++)
		{

			xp = j - hsX;
			yp = i - hsY;

			temp3 = (na_1 * xp + na_2 * yp + na_3 * focal_a);

			float xvp = temp1_0 + (est_ca_r_cv_1_1 
				* xp + est_ca_r_cv_2_1 * yp + est_ca_r_cv_3_1 * focal_a) 
				* (temp2 / temp3);

			float yvp = temp1_1 + (est_ca_r_cv_1_2 
				* xp + est_ca_r_cv_2_2 * yp + est_ca_r_cv_3_2 * focal_a) 
				* (temp2 / temp3);

			x = (int)(xvp + hsX);
			y = (int)(yvp + hsY);

			/* write the point in the image in matrix format */
			if(x>=0 && x<v_img.cols && y>=0 && y<v_img.rows)
			{
				int color = a_img.at<uchar>(i,j);
				v_img.at<uchar>(y,x) = color;
			}
			newNpts_norep++;
			newNpts++;
		}
	}

	return (newNpts_norep);
}

cv::Point Rectifier::point_rectification(const Camera &actual, Camera &virt,int width,int height,const cv::Point &p)
const
{
	/* dimensions of the virtual image */
	int hsX = width / 2;
	int hsY = height / 2;

	/* get focal lenght of the two cameras */
	float focal_a = actual.getFocal();
	float focal_v = virt.getFocal();

	/* Compute some constants used for rectification */
	float foc_d = focal_v / dv;
	float temp1_0 = foc_d * cv_tca(0,0);
	float temp1_1 = foc_d * cv_tca(1,0);
	float temp2 = foc_d * da;
	float temp3;
	float na_1 = Na(0,0);
	float na_2 = Na(1,0);
	float na_3 = Na(2,0);
	float est_ca_r_cv_1_1 = ca_R_cv(0,0);
	float est_ca_r_cv_2_1 = ca_R_cv(1,0);
	float est_ca_r_cv_3_1 = ca_R_cv(2,0);
	float est_ca_r_cv_1_2 = ca_R_cv(0,1);
	float est_ca_r_cv_2_2 = ca_R_cv(1,1);
	float est_ca_r_cv_3_2 = ca_R_cv(2,1);

	float xp, yp;
	int x, y;

	xp = p.x - hsX;
	yp = p.y - hsY;

	temp3 = (na_1 * xp + na_2 * yp + na_3 * focal_a);

	float xvp = temp1_0 + (est_ca_r_cv_1_1 
		* xp + est_ca_r_cv_2_1 * yp + est_ca_r_cv_3_1 * focal_a) 
		* (temp2 / temp3);

	float yvp = temp1_1 + (est_ca_r_cv_1_2 
		* xp + est_ca_r_cv_2_2 * yp + est_ca_r_cv_3_2 * focal_a) 
		* (temp2 / temp3);

	x = (int)(xvp + hsX);
	y = (int)(yvp + hsY);

	return cv::Point(x,y);
}

int Rectifier::rectification(const Camera &actual, Camera &virt)
const
{
	/* images pointers */
	Image *a_img = actual.image;
	Image *v_img = virt.image;

	/* dimensions of the virtual image */
	int sX = v_img->getWidth();
	int sY = v_img->getHeight();
	int hsX = sX / 2;
	int hsY = sY / 2;

	/* get focal lenght of the two cameras */
	float focal_a = actual.getFocal();
	float focal_v = virt.getFocal();

	/* create a white background */
	Eigen::Matrix<uchar, Eigen::Dynamic, Eigen::Dynamic> image_frame;
	image_frame.setConstant(sY, sX, 255);

	/* get four corners in the actual image plane */
	Point corners[4];

	if (verbose) {
		Image clone_img(*a_img);
		getActualImageCorners(actual, virt, corners, clone_img);
	} else {
		getActualImageCorners(actual, virt, corners);
	}

	/* define four bounderies */
	int left_bound;
	if (corners[0](0,0) < corners[2](0,0))
		left_bound = corners[0](0,0);
	else
		left_bound = corners[2](0,0);
	int right_bound;
	if (corners[1](0,0) > corners[3](0,0))
		right_bound = corners[1](0,0);
	else
		right_bound = corners[3](0,0);
	int up_bound;
	if (corners[0](1,0) < corners[1](1,0))
		up_bound = corners[0](1,0);
	else
		up_bound = corners[1](1,0);
	int down_bound;
	if (corners[2](1,0) > corners[3](1,0))
		down_bound = corners[2](1,0);
	else
		down_bound = corners[3](1,0);

	/* Compute some constants used for rectification */
	float foc_d = focal_v / dv;
	float temp1_0 = foc_d * cv_tca(0,0);
	float temp1_1 = foc_d * cv_tca(1,0);
	//float temp1_2 = foc_d * cv_tca(2,0);
	float temp2 = foc_d * da;
	float temp3;
	float na_1 = Na(0,0);
	float na_2 = Na(1,0);
	float na_3 = Na(2,0);
	float est_ca_r_cv_1_1 = ca_R_cv(0,0);
	float est_ca_r_cv_2_1 = ca_R_cv(1,0);
	float est_ca_r_cv_3_1 = ca_R_cv(2,0);
	float est_ca_r_cv_1_2 = ca_R_cv(0,1);
	float est_ca_r_cv_2_2 = ca_R_cv(1,1);
	float est_ca_r_cv_3_2 = ca_R_cv(2,1);
	//float est_ca_r_cv_1_3 = ca_R_cv(0,2);
	//float est_ca_r_cv_2_3 = ca_R_cv(1,2);
	//float est_ca_r_cv_3_3 = ca_R_cv(2,2);

	/* Allocate space for new points in the virtual image. Maximum number of 
	 * points is the number of points in the actual image.
	 */
	int a_img_nPts = a_img->getNumberPoints();
	ListPoints Iv;
	Iv.resize(2, a_img_nPts);
	
	/* Get points of the actual image */
	const ListPoints &Ia = a_img->getListPts();

	/* variables which will contain the new computed point */
	Eigen::Matrix<float, 3, 1> newC;
	int newXmat, newYmat;

	int newNpts = 0;
	int newNpts_norep = 0;

	int start_index = 
		searchForFirstImagePoint(Ia, a_img_nPts, up_bound);
	if (start_index < 0)
		start_index = a_img_nPts; /* loop will be avoided */
	for (int i = start_index; i < a_img_nPts; i++) {

		if (Ia(1,i) > down_bound)
			break;

		if (Ia(0,i) < left_bound)
			continue;
		else if (Ia(0,i) > right_bound)
			continue;

		temp3 = (na_1 * Ia(0,i) + na_2 * Ia(1,i) + na_3 * focal_a);

		newC(0,0) = temp1_0 + (est_ca_r_cv_1_1 
			* Ia(0,i) + est_ca_r_cv_2_1 * Ia(1,i) + est_ca_r_cv_3_1 * focal_a) 
			* (temp2 / temp3);

		newC(1,0) = temp1_1 + (est_ca_r_cv_1_2 
			* Ia(0,i) + est_ca_r_cv_2_2 * Ia(1,i) + est_ca_r_cv_3_2 * focal_a) 
			* (temp2 / temp3);

		//newC(2,0) = temp1_2 + (est_ca_r_cv_1_3 
		//	* Ia(0,i) + est_ca_r_cv_2_3 * Ia(1,i) + est_ca_r_cv_3_3 * focal_a) 
		//	* (temp2 / temp3);

		/* Coordinates in the image matrix: (0,0) is the top-left corner */
		newYmat = round(newC(1,0) + hsY);
		newXmat = round(newC(0,0) + hsX);

		if ( (newXmat > 0) && (newXmat < sX) 
		&& (newYmat > 0) && (newYmat < sY) ) {
			/* avoid double insertions */
			if (image_frame(newYmat,newXmat) == 255) {
				/* write the point in the image in matrix format */
				image_frame(newYmat,newXmat) = 0;
				Iv(0,newNpts_norep) = round(newC(0,0));
				Iv(1,newNpts_norep) = round(newC(1,0));
				newNpts_norep++;
			}
			newNpts++;
		}
	}

	v_img->loadListPts(Iv, newNpts_norep);
	Iv.resize(2, 0);
	image_frame.resize(0, 0);

	return (newNpts_norep);
}


int Rectifier::searchForFirstImagePoint(
const ListPoints &Ia
, int Npts
, float y_bound)
const
{
	int start = 0;
	int end = Npts - 1;
	int target = start;
	int range = 0;
	
	while (1) {
		if (Ia(1,target) >= y_bound) {
			if (target == 0)
				return 0;
			if (Ia(1,target - 1) < y_bound)
				return target;
			end = target;
			range = end - start;
			target = start + (range / 2);
			if (target == start)
				return start;
		} else {
			start = target;
			range = end - start;
			target = start + (range / 2);
			if (target == start)
				return -1;
		}
	} 
}

void Rectifier::setVerbose(bool verbose)
{
	this->verbose = verbose;
}
