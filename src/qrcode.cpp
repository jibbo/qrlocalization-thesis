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
    
    //look for qrs inside the picture
    vector<Image::SymbolIterator> symbols = searchQrCode();
    
    if(symbols.size()<1)
    {
        this->points=vector<Point>(4,Point(-1,-1));
        this->angle = -1;
        this->center = Point(-1,-1);
        this->mcenter = new float[2]{-1,-1};
        data = "";
    }
    else
    {
        //it just pick the first
        initialize(symbols.at(0));
    }
}

//this method looks for qrcodes inside the image
vector<Image::SymbolIterator> QrCode::searchQrCode()
{
    int trycont = 3;
    vector<Image::SymbolIterator> symbols;
    bool rotated;
    Mat dest;

    //it try to look for qr with 7 tryings
    //from the 7th to 0

    do
    {
        rotated=false;
        Mat tmp =picture.clone(),tmp2 = picture.clone();
        switch(trycont)
        {
            case 1 :
                rotated=false;
                filter(picture,dest,120);
            break;
            case 2 :
                rotate(picture,dest,180);
                rotated = true;
            break;
            case 3 :
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
    //while it founds a qrcode or it finishes the tryings

    this->rotated = rotated; 
    return symbols;

}

void QrCode::initialize(Image::SymbolIterator symbol)
{
    //set the vertexes found in zbar
    compute_points(symbol);
    //calculates the angle of the qr
    compute_angle();
    //calculates the center of the qr in px
    compute_center();
    //calculates the center of the qr in meters 
    compute_center_in_meters();
    data = symbol->get_data();
}

//create a vector containing every qrcode found.
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

//transform the Mat to Image
Image* QrCode::get_image(Mat frame)
{
    int width = frame.cols;
    int height = frame.rows;
    return new zbar::Image(width, height, "Y800", frame.data, width * height);
}

//rotate an image by an angle
//the pivot is the center of the image.
void QrCode::rotate(Mat& src, Mat& dst, double angle)
 {
    cv::Point2f pt(src.cols/2., src.rows/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
}

//make the image black and white
void QrCode::filter(Mat& src,Mat& dest,int threeshold){
    for (int i = 0; i < src.rows; i++) 
    {
        for(int j=0; j< src.cols;j++)
        {
            if(src.at<uchar>(i,j)<threeshold)
                dest.at<uchar>(i,j)=0;//black
            else
                dest.at<uchar>(i,j)=255;//white
        }
    }
}


//it computes the center in meter of the qrcode
//giving it in real coordinates
void QrCode::compute_center_in_meters()
{
    Point p = rectifyPoint(center);
    Point imgcenter(picture.cols/2,picture.rows/2);

    //change of Origin
    float xv= (p.x - imgcenter.x) * (virtual_camera_height/focal);
    float yv= (p.y - imgcenter.y) * (virtual_camera_height/focal);

    //change of Orientation 
    //from the real camera to the virtual
    float xp = (xv + virtual_camera_x);
    float yp = -yv;
    this->mcenter = new float[2]{xp,yp};
}

//rectify only a point
//in order to have real coordinates
Point QrCode::rectifyPoint(Point p){

    //prepare cameras
    int width = picture.cols;
    int height = picture.rows;
    visuallocalization::Camera camera(width, height);

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

    visuallocalization::Camera v_camera(width, height);


    roll = -M_PI;
    pitch = -(virtual_camera_angle * M_PI) / 180;
    yaw = 0;
    pos(0,0) = virtual_camera_x;
    pos(1,0) = virtual_camera_y;
    pos(2,0) = virtual_camera_height;
    v_camera.setRotation(roll, pitch, yaw);
    v_camera.setPosition(pos);
    v_camera.setFocal(focal);

    return visuallocalization::Rectifier(camera,v_camera)
        .point_rectification(camera, v_camera,
            picture.cols,picture.rows, p);
}

//store all the points found by zbar
void QrCode::compute_points(Image::SymbolIterator symbol)
{
    for (int i = 0; i < symbol->get_location_size(); i++)
    {
        if(rotated){
            points.push_back(Point(abs(picture.cols-symbol->get_location_x(i)), 
                abs(picture.rows-symbol->get_location_y(i))));
        }
        else{
            points.push_back(Point(symbol->get_location_x(i), 
                symbol->get_location_y(i)));
        }
    }
}

//calculate the discance between the center and a point
double QrCode::get_distance(Point from)
{
    double out= sqrt(pow(from.x-center.x,2)
        +pow(from.y-center.y,2));
    return out;
}

 void QrCode::compute_angle()
 {
    //rectify every point of the qr 
    vector<Point> newPoints;
    for(int i =0; i<points.size(); i++)
    {
        Point p = rectifyPoint(points.at(i));
        newPoints.push_back(p);
    }

    float dx=(newPoints.at(3).x-newPoints.at(0).x);
    float dy=(newPoints.at(3).y-newPoints.at(0).y);

    if(dx==0||dy==0)
    {
        if(dy>0){
            this->angle= 90;
        }else if(dy<0){
            this->angle= 270;
        }else if(dx>0){
            this->angle= 0;
        }else if(dx<0){
            this->angle= 180;
        }else{
            this->angle = 0;
        }
        
    }
    else
    {
        double angle=(180*atan(dx/dy))/M_PI;
        
        if(dy<0&&dx>0){
            this->angle=270-angle;
        }else if(dx<0&&dy>0){
            this->angle=90-angle;
        }else if(dx<0&&dy<0){
            this->angle=270-angle;
        }else if(dx>0&&dy>0){
            this->angle=90-angle;
        }
       
    }
}


void QrCode::compute_center()
{
    RotatedRect r = minAreaRect(points);
    this->center=r.center;
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
