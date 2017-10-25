#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;

int contourThreshold = 255;



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
  

    Mat img_grey, dist;
    cvtColor(image, img_grey, CV_BGR2GRAY);
    blur( img_grey, img_grey, Size(3,3) );

        Mat canny_output;
        std::vector<std::vector<Point> > contours;
        std::vector<Vec4i> hierarchy;
      
        /// Detect edges using canny
        Canny( img_grey, canny_output, contourThreshold, contourThreshold*2, 3 );
        /// Find contours
        findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
      
        /// Draw contours
        Mat drawing = 255*Mat::ones( canny_output.size(), CV_8UC1 );
        for( int i = 0; i< contours.size(); i++ )
           {
            // Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
             drawContours( drawing, contours, i, Scalar(0,0,0), 2, 8, hierarchy, 0, Point() );
           }
           


      distanceTransform(drawing, dist, CV_DIST_L2, 3); //apply distance transfrom
       normalize(dist, dist, 0, 1., NORM_MINMAX);
    char exit_key_press = 0;
    do {
      imshow("myWindow",dist);
        exit_key_press = cvWaitKey(1);
    }while (exit_key_press != '\x1b');

    return 0;
}