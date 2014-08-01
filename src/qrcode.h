#ifndef QRCODE_H
#define QRCODE_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <zbar.h>
#include <vector>
#include <string>
#include <cmath>

#include "imageloader.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "rectifier.hpp"

class QrCode
{
    public:
      QrCode(cv::Mat& picture,float focal, float camera_angle,
              float camera_height, float camera_x, float camera_y,
              float virtual_camera_angle, float virtual_camera_height,
              float virtual_camera_x, float virtual_camera_y);
      // QrCode(zbar::Image::SymbolIterator symbol);
      // QrCode(zbar::Image::SymbolIterator symbol,bool rotated);
      double get_angle();
      int get_quadrant();
      cv::Point get_center();
      float* get_center_in_meter();
      std::string get_data();
      double get_distance(cv::Point from);

    private:
      std::vector<cv::Point> points;
      bool rotated;
      double angle;
      int quadrant;
      cv::Point center;
      float* mcenter;
      std::string data;

      cv::Mat picture;
      float focal;
      float camera_angle;
      float camera_height;
      float camera_x;
      float camera_y;
      float virtual_camera_angle;
      float virtual_camera_height;
      float virtual_camera_x; 
      float virtual_camera_y;
      
      zbar::Image *get_image(cv::Mat frame);
      vector<zbar::Image::SymbolIterator> get_read_symbols(zbar::Image *img);
      void rotate(cv::Mat& src, cv::Mat& dst, double angle);
      void filter(Mat& src,Mat& dest,int threeshold);
      void rectify(Mat& src,Mat& dest);
      vector<zbar::Image::SymbolIterator> searchQrCode();
      void initialize(zbar::Image::SymbolIterator symbol);
      void compute_points(zbar::Image::SymbolIterator symbol);
      void compute_quadrant();
      void compute_shape();
      void compute_center_in_meters();
};
#endif