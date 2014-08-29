#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <vector>
#include <fstream>
#include <cstdio>
#include <dirent.h>
#include <string.h>

#include "imageloader.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "rectifier.hpp"
#include "qrcode.h"

//70 cm h


//frame resolution
const double WIDTH = 640.0;
const double HEIGHT = 480.0;

//sampling's time in ms
const double TIME = 30;

//angolo retto
const int RECT_ANGLE = 90;

const int THREESHOLD = 10;

const string IMG_FOLDER= "../test_img/";

using namespace std;
using namespace visuallocalization;
using namespace cv;
using namespace zbar;

zbar::Image *get_image(Mat frame);
void rotate(Mat& src, Mat& dst, double angle);
void filter(Mat& src,Mat& dest,int threeshold);
void rectify(Mat& src,Mat& dest);

int main()
{

    //namedWindow("Original", WINDOW_AUTOSIZE);
    //namedWindow("Processed", WINDOW_AUTOSIZE);
    Mat picture,dest;
    
    int first=0,second=0,third=0,forth=0,fifth=0,sixth=0,seventh=0,no=0;

    DIR *dir;
    struct dirent *dirent;

    char * path = new char [IMG_FOLDER.length()+1];
    strcpy (path, IMG_FOLDER.c_str());
    dir = opendir(path);

    if (dir != NULL)
    {
        while ((dirent = readdir(dir)))
        {
            if(dirent->d_name[0]=='.')
                continue;
            string name = string(dirent->d_name);
            string id = name.substr(0,name.find_first_of("_"));
            string angle = name.substr(name.find_first_of("_")
                +1,name.find_first_of("(")-2);

            picture = imread(IMG_FOLDER + name,CV_LOAD_IMAGE_UNCHANGED);

            
            //imshow("Original",picture);
            vector<zbar::Image::SymbolIterator> symbols;
            int trycont = 6;
            bool rotated = false;
            bool exit=false;
            do
            {
                //cout << "Try image: " << name <<" ...";
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
                //imshow("Processed",dest);
                symbols.clear();
                zbar::ImageScanner scanner;
                scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
                zbar::Image *img = get_image(dest);
                //scan the zbar::Image for barcodes
                int n = scanner.scan(*img);
                for (zbar::Image::SymbolIterator symbol = 
                        img->symbol_begin();
                    symbol != img->symbol_end();
                    ++symbol)
                {
                    symbols.push_back(symbol);
                }
                //cout <<endl;
                
                if(symbols.size()>0){
                    
                    switch(trycont){
                        case 6:
                            first++;
                            break;
                        case 5:
                            second++;
                            break;
                        case 4:
                            third++;
                            break;
                        case 3:
                            forth++;
                            break;
                        case 2:
                            fifth++;
                            break;
                        case 1:
                            sixth++;
                            break;
                        case 0:
                            seventh++;
                            break;
                    }
                    exit=true;
                }
                trycont--;
                if(trycont<0){
                    no++;
                    cout<<"Nothing found at: "<< name<<endl;
                }
                //waitKey(TIME); 
            }while( trycont>=0 && !exit);
            exit=false;
        }
            cout<<endl<<endl;
            cout<<"first: "<<first<<endl;
            cout<<"second: "<<second<<endl;
            cout<<"third: "<<third<<endl;
            cout<<"forth: "<<forth<<endl;
            cout<<"fifth: "<<fifth<<endl;
            cout<<"sixth: "<<sixth<<endl;
            cout<<"seventh: "<<seventh<<endl;
            cout<<"unfound: "<<no<<endl; 
    }  
    return 0;
}

/**
 * create an image from the Mat
 */
 zbar::Image *get_image(Mat frame)
 {
    int width = frame.cols;
    int height = frame.rows;
    //uchar *raw = (uchar *) frame.data;
    return new zbar::Image(width, height, "Y800", frame.data, width * height);
}

/**
 * Rotate an image
 */
 void rotate(Mat& src, Mat& dst, double angle)
 {
    cv::Point2f pt(src.cols/2., src.rows/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
}


void rectify(Mat& src,Mat& dest){
    Camera camera(WIDTH, HEIGHT);
    Camera v_camera(WIDTH, HEIGHT);

    float roll = 0, pitch = 0, yaw = 0;
    Eigen::Matrix<float, 3, 1> pos;

    roll = -M_PI;
    pitch = -(24.6 * M_PI) / 180;
    yaw = 0;
    pos(0,0) = 0.065;
    pos(1,0) = 0.045;
    pos(2,0) = 0.75;
    camera.setRotation(roll, pitch, yaw);
    camera.setPosition(pos);
    camera.setFocal(770.9);
    
    roll = -M_PI;
    pitch = 0;
    yaw = 0;
    pos(0,0) = 0.54;
    pos(1,0) = 0;
    pos(2,0) = 0.95;
    v_camera.setRotation(roll, pitch, yaw);
    v_camera.setPosition(pos);
    v_camera.setFocal(770.9);

    Rectifier rectifier(camera, v_camera);
    rectifier.rectification(camera, v_camera, src, dest);
}

/**
 * filtro la luce intermedia in modo da far diventare
 * l'immagine in bianco e nero e basta.
 */
 void filter(Mat& src,Mat& dest,int threeshold){
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

