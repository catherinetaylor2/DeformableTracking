#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <segmentation.h>

using namespace cv;

float thresh = 0.1f;

int SegmentImage(std::string PrevSeg, std::string filename){
    if(PrevSeg.empty()||filename.empty()){
        std::cerr<<"Error:no such file \n";
        return -1;
    }
    Mat image = imread( PrevSeg, 1 );
    Mat image2 = imread( filename, 1 );
    if(image.empty()||image2.empty()){
        std::cerr<<"Error: cannot open image \n";
        return -1;
    }
  
    Mat dist, bw;
    cvtColor(image, bw, CV_BGR2GRAY);
    bw = bw >0;
    //threshold(bw, bw, 255, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

 distanceTransform(bw, dist, DIST_L2, 3); //apply distance transfrom
normalize(dist, dist, 0, 1., NORM_MINMAX);

    for(int x = 0; x<dist.rows; x++){
        for (int y =0; y<dist.cols; y++){
           if(dist.at<float>(x, y)<(0.000001)){
               dist.at<float>(x, y)= 0.0f;
            }
            else if(dist.at<float>(x, y)>(0.1)){
                dist.at<float>(x, y)= 1.0f;
            }
            else{
                dist.at<float>(x, y)= 0.5f;
            }
        }
       
    }
    Mat bgdModel, fgdModel, img, m; 
    
    {
        FileStorage fs("mymodels.yml", FileStorage::READ);
        fs["BgdModel"] >> bgdModel;
        fs["FgdModel"] >> fgdModel;
    }

    Mat1b mask (dist.rows, dist.cols,  uchar(GC_PR_BGD));
    for(int i =0; i<mask.rows; ++i){
        for(int j=0; j<mask.cols; ++j){
            if (dist.at<float>(i, j)== 0.0f){
                circle(mask, Point(j,i), 1, Scalar(GC_BGD), -1);
            }
            else if (dist.at<float>(i, j)== 1.0f){
                circle(mask, Point(j,i), 1, Scalar(GC_FGD), -1);
            }
            else {
                circle(mask, Point(j,i), 1, Scalar(GC_PR_FGD), -1);
            }
        }
    }
   

   grabCut(image2, mask, Rect(), bgdModel, fgdModel, 1);
    m =  ( mask == cv::GC_FGD) | ( mask == cv::GC_PR_FGD);
    image2.copyTo(img, m);


    namedWindow("myWindow", WINDOW_AUTOSIZE);
    char exit_key_press = 0;
    do {
        imshow("myWindow", dist);
        exit_key_press = cvWaitKey(1);
    }while (exit_key_press != '\x1b');

    std::string outputName = "../SegmentedImages/seg" + filename.substr(15,18);
    imwrite(outputName, img);

    return 1;
}