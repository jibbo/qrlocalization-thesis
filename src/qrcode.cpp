#include "qrcode.h"

using namespace std;
using namespace cv;
using namespace zbar;


QrCode::QrCode(Mat& picture, float focal, float camera_angle,
    float camera_height, float camera_x, float camera_y,
    float virtual_camera_angle, float virtual_camera_height,
    float virtual_camera_x, float virtual_camera_y)
{
    this->picture = picture;
    this->focal = focal;
    this->camera_angle = camera_angle;
    this->camera_height = camera_height;
    this->camera_x = camera_x;
    this->camera_y = camera_y;
    this->virtual_camera_angle = virtual_camera_angle;
    this->virtual_camera_height = virtual_camera_height;
    this->virtual_camera_x = virtual_camera_x; 
    this->virtual_camera_y = virtual_camera_y;
    
    vector<Image::SymbolIterator> symbols = searchQrCode();
    
    if(symbols.size()<1)
    {
        this->points=vector<Point>(4,Point(-1,-1));
        this->quadrant=-1;
        this->angle = -1;
        this->center = Point(-1,-1);
        this->mcenter = new float[2]{-1,-1};
        data = "";
    }
    else
    {
        initialize(symbols.at(0));
    }
}

vector<Image::SymbolIterator> QrCode::searchQrCode(){
    int trycont = 6;
    vector<Image::SymbolIterator> symbols;
    bool rotated;
    Mat dest;
    do
    {
        rotated=false;
        Mat tmp =picture.clone(),tmp2 = picture.clone();
        switch(trycont){
            case 0:
                rectify(picture,tmp); 
                filter(tmp,tmp2,120);
                rotate(tmp2,dest,180);
                rotated = true;
            break;
            case 1 :
                rectify(picture,tmp);
                rotate(tmp,dest,180);
                rotated = true; 
            break; 
            case 2 :
                rectify(picture,dest); 
                filter(dest,dest,120);
            break;
            case 3 :
                rectify(picture,dest);
            break;
            case 4 :
                filter(picture,dest,120);
            break;
            case 5 :
                rotate(picture,dest,180);
                rotated = true;
            break;
            case 6 :
                dest = picture.clone();
            default:
            break;
        }
        Image *img = get_image(dest);
        symbols = get_read_symbols(img);
        trycont--;

        //clean
        img->set_data(NULL, 0);
        delete img;

    }while(symbols.size()==0 && trycont>=0);

    this->rotated = rotated; 
    return symbols;

}

void QrCode::initialize(Image::SymbolIterator symbol)
{
    compute_points(symbol);
    compute_quadrant();
    compute_shape();
    compute_center_in_meters();
    data = symbol->get_data();
}

vector<Image::SymbolIterator> QrCode::get_read_symbols(Image *img)
{
    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    //scan the image for barcodes
    int n = scanner.scan(*img);
    // extract results
    vector<Image::SymbolIterator> out;
    for (Image::SymbolIterator symbol = img->symbol_begin();
        symbol != img->symbol_end();
        ++symbol)
    {
        out.push_back(symbol);
    }
    return out;
}

Image* QrCode::get_image(Mat frame)
 {
    int width = frame.cols;
    int height = frame.rows;
    return new zbar::Image(width, height, "Y800", frame.data, width * height);
}

void QrCode::rotate(Mat& src, Mat& dst, double angle)
 {
    cv::Point2f pt(src.cols/2., src.rows/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
}

void QrCode::filter(Mat& src,Mat& dest,int threeshold){
    for (int i = 0; i < src.rows; i++) 
    {
        for(int j=0; j< src.cols;j++)
        {
            if(src.at<uchar>(i,j)<threeshold)
                dest.at<uchar>(i,j)=0;
            else
                dest.at<uchar>(i,j)=255;
        }
    }
}

void  QrCode::rectify(Mat& src,Mat& dest){
    int width = picture.cols;
    int height = picture.rows;
    visuallocalization::Camera camera(width, height);
    visuallocalization::Camera v_camera(width, height);

    float roll = 0, pitch = 0, yaw = 0;
    Eigen::Matrix<float, 3, 1> pos;

    roll = -M_PI;
    pitch = -(camera_angle * M_PI) / 180;
    yaw = 0;
    pos(0,0) = camera_x;
    pos(1,0) = camera_y;
    pos(2,0) = camera_height;
    camera.setRotation(roll, pitch, yaw);
    camera.setPosition(pos);
    camera.setFocal(focal);
    
    roll = -M_PI;
    pitch = -(virtual_camera_angle * M_PI) / 180;
    yaw = 0;
    pos(0,0) = virtual_camera_x;
    pos(1,0) = virtual_camera_y;
    pos(2,0) = virtual_camera_height;
    v_camera.setRotation(roll, pitch, yaw);
    v_camera.setPosition(pos);
    v_camera.setFocal(focal);
    visuallocalization::Rectifier rect(camera, v_camera);
    rect.rectification(camera, v_camera, src, dest);
}

void QrCode::compute_center_in_meters(){
    int width = picture.cols;
    int height = picture.rows;
    visuallocalization::Camera camera(width, height);
    visuallocalization::Camera v_camera(width, height);

    float roll = 0, pitch = 0, yaw = 0;
    Eigen::Matrix<float, 3, 1> pos;

    roll = -M_PI;
    pitch = -(camera_angle * M_PI) / 180;
    yaw = 0;
    pos(0,0) = camera_x;
    pos(1,0) = camera_y;
    pos(2,0) = camera_height;
    camera.setRotation(roll, pitch, yaw);
    camera.setPosition(pos);
    camera.setFocal(focal);
    
    roll = -M_PI;
    pitch = -(virtual_camera_angle * M_PI) / 180;
    yaw = 0;
    pos(0,0) = virtual_camera_x;
    pos(1,0) = virtual_camera_y;
    pos(2,0) = virtual_camera_height;
    v_camera.setRotation(roll, pitch, yaw);
    v_camera.setPosition(pos);
    v_camera.setFocal(focal);
    visuallocalization::Rectifier rect(camera, v_camera);
    Point p = rect.point_rectification(camera, v_camera,picture.cols,picture.rows, center);
    cv::Point imgcenter = cv::Point(picture.cols/2,picture.rows/2);
    float xv= (p.x - imgcenter.x) * (virtual_camera_height/focal);
    float yv= (p.y - imgcenter.y) * (virtual_camera_height/focal);

    float xp = (xv + virtual_camera_x);
    float yp = -yv;
    this->mcenter = new float[2]{xp,yp};
}

void QrCode::compute_points(Image::SymbolIterator symbol){
    for (int i = 0; i < symbol->get_location_size(); i++)
    {
        if(rotated){
            points.push_back(Point(abs(640-symbol->get_location_x(i)), 
                abs(480-symbol->get_location_y(i))));
        }
        else{
            points.push_back(Point(symbol->get_location_x(i), 
                symbol->get_location_y(i)));
        }
    }
}

/**
 * The index of the point with less y corresponds
 * to the quadrant needed
 */
void QrCode::compute_quadrant(){
    int point=0, min= points.at(0).y;
    for(int i=1;i<points.size();i++)
    {
      if(points.at(i).y<min-3)
      {
         min = points.at(i).y;
         point=i;
      }
    }
    quadrant= point+(rotated?3:1);
}

double QrCode::get_distance(Point from){
    double out= sqrt(pow(from.x-center.x,2)+pow(from.y-center.y,2));
    return out;
}

/**
 * retrieve the angle of an object
 */
 void QrCode::compute_shape()
 {
    RotatedRect r = minAreaRect(points);
    angle=(90*quadrant)-abs(r.angle);
    center=r.center;
}

int QrCode::get_quadrant()
{
    return quadrant;
}
double QrCode::get_angle()
{
    return angle;
}
string QrCode::get_data()
{
    return data;
}
Point QrCode::get_center()
{
    return center;
}
float* QrCode::get_center_in_meter()
{
    return mcenter;
}

