#ifndef IMAGE_LOADER_H
#define IMAGE_LOADER_H

#include <stdlib.h>

#include "image.hpp"

namespace visuallocalization {

class ImageLoader {
	private:
		/*** variables ***/

		bool verbose;

	public:
		/*** constructor and destructor ***/

		ImageLoader();

		/*** methods ***/

		/*
		 * Load an image from file.
		 *
		 * @file_name: string with the path of the file
		 * @image: object Image in which the frame is loaded
		 */
		int loadFromFile(const char *file_name, Image &image) const;

		/*TODO*/
		int loadFromCapture();

		/*
		 * Set verbose mode for the input image.
		 *
		 * @verbose: the verbose mode
		 */
		void setVerbose(bool verbose);
};

} /* end namespace */

#endif /* IMAGE_LOADER_H */
