#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdlib.h>
#include <iostream>

#include "imageloader.hpp"
#include "image.hpp"

using namespace visuallocalization;

ImageLoader::ImageLoader()
{
	verbose = 0;
}

int ImageLoader::loadFromFile(const char *filename, Image &image) const
{
	/* load the image from file */
	cv::Mat1b read_image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
	if ((read_image.rows != image.getHeight()) 
	| (read_image.cols != image.getWidth())) {
		std::cout << "Mismatch between size file and image size" << std::endl;
		return -1;
	}

	cv::Mat1b canny_image;

	/* The parameters 3 and 4 of Canny have to be found. Other possible values
	 * for images very dark could be 30 and 50.
	 */
	cv::Canny(read_image, canny_image, 350, 400, 3);
	image.loadOpenCVImage(canny_image);

	/* show image for debug purposes */
	if (this->verbose) {
		cv::imshow("Input", canny_image);
		cv::waitKey(0);
		cv::destroyWindow("Input");
	}

	/* find list of points from the image */
	image.findPoints();

	return 0;
}

void ImageLoader::setVerbose(bool verbose)
{
	this->verbose = verbose;
}
