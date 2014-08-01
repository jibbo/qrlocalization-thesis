#ifndef IMAGE_H
#define IMAGE_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/Core>
#include <stdlib.h>

#define WHITE_ON_BLACK 0
#define BLACK_ON_WHITE 1

#define NO_WAIT_FOR_KEY 0
#define WAIT_FOR_KEY 1

namespace visuallocalization {

typedef Eigen::Matrix<int, 2, 1> Point;
typedef Eigen::Matrix<int, 2, Eigen::Dynamic> ListPoints;

class Image {
	private:
		/*** variables ***/

		/* image size */
		int width;
		int height;

		/* Points in the image plane: origin is pinhole */
		int nPts;
		ListPoints Pts;

		/* Image frame in the openCV format */
		int opencv_generated;
		int conf_opencv;
		cv::Mat1b opencv_image;

	public:
		/*** variables ***/

		/*** constructor and destructor ***/
		Image();
		Image(const Image &image);
		Image(const int width, const int height);
		~Image();

		/*** methods ***/

		/*
		 * Load opencv_image
		 *
		 * @opencv_image: the opencv image to load
		 * @conf: 0 - points white on black background
		 *        1 - points black on white background
		 *        default 0
		 *
		 * Returns 0 on succes, -1 on failure
		 */
		int loadOpenCVImage(
			const cv::Mat1b &opencv_image
			, const int conf = WHITE_ON_BLACK);

		void copyImage(cv::Mat &copy) const;

		/*
		 * Load list of points
		 *
		 * @newPts: list of new points
		 *
		 * Returns 0 on succes, -1 on failure
		 */
		int loadListPts(const ListPoints &newPts, const int nPts);

		/*
		 * Returns a read-only reference to the list of points
		 */
		const ListPoints & getListPts() const;

		/*
		 * Set the number of points in the image.
		 */
		void setNumberPoints(int nPts);

		/*
		 * Returns the number of points in the image.
		 */
		int getNumberPoints() const;

		/*
		 * Returns the width of the image.
		 */
		int getWidth() const;

		/*
		 * Returns the height of the image.
		 */
		int getHeight() const;

		/*
		 * Find the list of points in the opencv image and store them as point
		 * of the image plane: a point in the center of the image is in (0,0)
		 *
		 * Returns the number of points found.
		 */
		int findPoints();

		/*
		 * Create the opencv image from the list of points.
		 *
		 * @conf: 0 - points white on black background
		 *        1 - points black on white background
		 *        default 0
		 */
		void createImage(const int conf = WHITE_ON_BLACK);

		/*
		 * Reset the list of points.
		 */
		void resetPoints();

		/*
		 * Show in an opencv window the image
		 *
		 * @window_name: name of the window
		 * @wait: 0 do not wait for key pressed before continuing
		 *        1 wait for key pressed before continuing
		 *
		 * Returns 0 on success, -1 on failure
		 */
		int show(const char *window_name, const int wait) const;

		/*
		 * Add a line stating from a and ending to b in the openCV image
		 *
		 * @a: starting point
		 * @b: ending point
		 */
		void addLine(const Point &a, const Point &b);

		/*
		 * Add a circle with center in a in the openCV image
		 *
		 * @a: center point
		 */
		void addCircle(const Point &a);

		/*
		 * Gives colored version of the image, that is a copy of the image but
		 * in a proper data structure.
		 *
		 * @colored: the matrix that will return the colored version
		 *
		 * Returns 0 on succes, -1 on failure.
		 */
		int getColoredVersion(cv::Mat &colored) const;
};

} /* end namespace */

#endif /* IMAGE_H_ */
