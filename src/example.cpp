#include <iostream>
#include <dirent.h>

#include "qrcode.h"

const string IMG_FOLDER= "../test_img/";

using namespace std;

int main()
{

    //create and show the window
    namedWindow("Capture", WINDOW_AUTOSIZE);

    Mat picture;
    char * path = new char [IMG_FOLDER.length()+1];
    strcpy (path, IMG_FOLDER.c_str());

    struct dirent *dirent;
    DIR *dir = opendir(path);
    
    int cont=0;
    if (dir != NULL)
    {
         while ((dirent = readdir(dir)))
         {
            if(dirent->d_name[0]=='.')
                continue;
            string name = string(dirent->d_name);
            picture = imread(IMG_FOLDER + name,CV_LOAD_IMAGE_UNCHANGED);
            if (!picture.data)
            {
                cout << "Error: no frame data.\n";
                continue;
            }

            //imshow("Capture", picture);

            /*
             * QrCode(cv::Mat& picture,float focal, float camera_angle,
             * float camera_height, float camera_x, float camera_y,
             * float virtual_camera_angle, float virtual_camera_height,
             * float virtual_camera_x, float virtual_camera_y);
            */

            QrCode qr(picture, 519.15, 24.6,
              70, 0.065, 0.045, 0, 0.95,
              0.54, 0);
              
              if(qr.get_angle()<0)
                cont++;
                else{
            float * mcenter = qr.get_center_in_meter();
            cout
            << qr.get_angle() << " "
            << "(" << qr.get_center().x 
                <<","<< qr.get_center().y <<") "
            << "(" << mcenter[0] 
                <<","<< mcenter[1]<<") "
            << qr.get_data() <<" "
            << endl;
            }
        }
        cout<< "unfound: "<<cont;
    }
    else
    { 
        cout << "directory doesn't exist"<<endl;
    }
    return 0;
}

