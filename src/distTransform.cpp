#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;

float thresh = 0.1f;

int main(){
    std::string filename;
    filename = "SegmentedImages/seg0001.jpg";
    if(filename.empty()){
        std::cerr<<"Error:no such file \n";
    }
    Mat image = imread( filename, 1 );
    if(image.empty()){
        std::cerr<<"Error: cannot open image \n";
    }
  
    Mat dist, bw;
    cvtColor(image, bw, CV_BGR2GRAY);
    threshold(bw, bw, 10, 100, CV_THRESH_BINARY | CV_THRESH_OTSU);

  distanceTransform(bw, dist, DIST_L2, 3); //apply distance transfrom
  normalize(dist, dist, 0, 1., NORM_MINMAX);

    for(int x = 0; x<dist.rows; x++){
        for (int y =0; y<dist.cols; y++){
           if(dist.at<float>(x, y)>(0.05 + thresh)){
               dist.at<float>(x, y)= 0;
            }
            else if(dist.at<float>(x, y)<(thresh - 0.05)){
                dist.at<float>(x, y)= 1;
            }
            else{
                dist.at<float>(x, y)= 0.5;
            }
        }
       
    }

    namedWindow("myWindow", WINDOW_AUTOSIZE);
    char exit_key_press = 0;
    do {
        imshow("myWindow",dist);
        exit_key_press = cvWaitKey(1);
    }while (exit_key_press != '\x1b');

    return 0;
}