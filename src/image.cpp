#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdlib.h>

#include "image.hpp"
#include "debug.hpp"

using namespace visuallocalization;

Image::Image()
{
	int width_def = 640;
	int height_def = 480;

	width = width_def;
	height = height_def;

	nPts = 0;

	conf_opencv = WHITE_ON_BLACK;
	opencv_generated = 0;
	opencv_image = cv::Mat(height, width, CV_8U);
}

Image::Image(const Image &image)
{
	width = image.getWidth();
	height = image.getHeight();

	const ListPoints & temp_points = image.getListPts();
	loadListPts(temp_points, image.getNumberPoints());

	conf_opencv = image.conf_opencv;
	opencv_generated = image.opencv_generated;
	opencv_image = image.opencv_image.clone();
}

Image::Image(const int width, const int height)
{
	this->width = width;
	this->height = height;

	nPts = 0;

	conf_opencv = WHITE_ON_BLACK;
	opencv_generated = 0;
	opencv_image = cv::Mat(height, width, CV_8U);
}

Image::~Image()
{
	if (nPts > 0) {
		Pts.resize(2, 0);
		nPts = 0;
	}

	opencv_generated = 0;
}

int Image::loadOpenCVImage(const cv::Mat1b &opencv_image, const int conf)
{
	/* reset data in the image */
	if (nPts > 0) {
		resetPoints();
	}

	if ((opencv_image.rows != height) 
	| (opencv_image.cols != width)) {
		return -1;
	}
	this->opencv_image = opencv_image.clone();
	opencv_generated = 1;
	conf_opencv = conf;
	return 0;
}

void Image::copyImage(cv::Mat &copy)
const
{
	copy = this->opencv_image.clone();
}

int Image::loadListPts(const ListPoints &newPts, const int nPts)
{
	if ((nPts < 1) || (nPts > newPts.cols())){
		resetPoints();
		return -1;
	}

	Pts = newPts.block(0, 0, 2, nPts);
	this->nPts = nPts;
	opencv_generated = 0;

	return 0;
}

const ListPoints & Image::getListPts()
const
{
	return Pts;
}

void Image::setNumberPoints(int nPts)
{
	this->nPts = nPts;
}

int Image::getNumberPoints()
const
{
	return nPts;
}

int Image::getWidth()
const
{
	return width;
}

int Image::getHeight()
const
{
	return height;
}

int Image::findPoints()
{
	if (opencv_generated == 0)
		return 0;

	int i, j;
	int hheight = height / 2;
	int hwidth = width / 2;

	if (nPts > 0) {
		resetPoints();
	}

	for (i = 0; i < height; i++)
		for (j = 0; j < width; j++)
			/* white points on black background */
			if (opencv_image(i,j) == 255) {
				nPts++;
			}

	Pts.resize(2, nPts);
	int ctr = 0;
	for (i = 0; i < height; i++)
		for (j = 0; j < width; j++)
			/* white points on black background */
			if (opencv_image(i, j) == 255) {
				Pts(0,ctr) = j - hwidth;
				Pts(1,ctr) = i - hheight;
				ctr++;
			}

	return nPts;
}

void Image::createImage(const int conf)
{
	/* avoid useless computation */
	if (opencv_generated)
		return;

	int hheight = height / 2;
	int hwidth = width / 2;

	int points_color = 0;
	int background_color = 0;

	if (conf == WHITE_ON_BLACK) {
		points_color = 255;
		background_color = 0;
	} else {
		points_color = 0;
		background_color = 255;
	}
	conf_opencv = conf;

	int i = 0, j = 0;
	for (i = 0; i < height; i++)
		for (j = 0; j < width; j++)
			opencv_image(i,j) = background_color;

	for (i = 0; i < nPts; i++)
		opencv_image(Pts(1,i) + hheight, Pts(0,i) + hwidth) = points_color;
	opencv_generated = 1;
}

void Image::resetPoints()
{
	nPts = 0;
	Pts.resize(2, 0);
}

int Image::show(const char *window_name, const int wait)
const
{
	if ((wait != NO_WAIT_FOR_KEY) && (wait != WAIT_FOR_KEY))
		return -1;
	if (!opencv_generated)
		return -1;
	if (nPts < 1)
		return -1;

	imshow(window_name, opencv_image);
	if (wait == WAIT_FOR_KEY)
		cv::waitKey(0);
	
	return 0;
}

void Image::addLine(const Point &a, const Point &b)
{
	int points_color = 0;
	if (conf_opencv == WHITE_ON_BLACK)
		points_color = 255;
	else
		points_color = 0;

	cv::line(opencv_image
	         , cv::Point2d(a(0,0), a(1,0)), cv::Point2d(b(0,0), b(1,0))
	         , cv::Scalar(points_color, points_color
	                      , points_color, points_color)
	         , 1, CV_AA, 0);
}

void Image::addCircle(const Point &a)
{
	int points_color = 0;
	if (conf_opencv == WHITE_ON_BLACK)
		points_color = 255;
	else
		points_color = 0;

	cv::circle(opencv_image
	           , cv::Point2d(a(0,0), a(1,0)), 3
	           , cv::Scalar(points_color, points_color
	                        , points_color, points_color)
	           , 2, 1, 0);
}

int Image::getColoredVersion(cv::Mat &colored) const
{
	if (!opencv_generated)
		return -1;

	cv::cvtColor(opencv_image, colored, CV_GRAY2BGR);

	return 0;
}
