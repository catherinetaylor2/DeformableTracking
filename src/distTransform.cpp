#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <segmentation.h>

using namespace cv;

float thresh = 0.1f;
int contourThreshold = 255;

int SegmentImage(std::string PrevSeg, std::string filename){
    if(PrevSeg.empty()||filename.empty()){
        std::cerr<<"Error:no such file in Segment image \n";
        return -1;
    }
    Mat image = imread( PrevSeg, 1 );
    Mat image2 = imread( filename, 1 );
    if(image.empty()||image2.empty()){
        std::cerr<<"Error: cannot open image in Segment image\n";
        return -1;
    }
  
    // Mat dist, bw;
    // cvtColor(image, bw, CV_BGR2GRAY);
    // bw = bw > 128;

    // distanceTransform(bw, dist, DIST_L2, 3); //apply distance transfrom
    // normalize(dist, dist, 0, 1., NORM_MINMAX);

    // for(int x = 0; x<dist.rows; x++){
    //     for (int y =0; y<dist.cols; y++){
    //        //std::cout<<dist.at<float>(x, y)<<"\n";
    //        if(dist.at<float>(x, y)<(0.0000000001)){
    //            dist.at<float>(x, y)= 1.0f;
    //         }
    //         else if(dist.at<float>(x, y)>(0.03)){
    //             dist.at<float>(x, y)= 0.0f;
    //         }
    //         else{
    //             dist.at<float>(x, y)= 0.5f;
    //         }
    //      }
       
    // }


    Mat img_grey, dist, canny_edges;
    cvtColor(image, img_grey, CV_BGR2GRAY);
    blur( img_grey, img_grey, Size(3,3) );

    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    Canny( img_grey, canny_edges, contourThreshold, contourThreshold*2, 3 );
    findContours( canny_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      
   Mat drawing2 = 255*Mat::ones( canny_edges.size(), CV_8UC1 );
    for( int i = 0; i< contours.size(); i++ ){
      drawContours( drawing2, contours, i, Scalar(0,0,0), 2, 8, hierarchy, 0, Point() );
    }


//     distanceTransform(drawing, dist, DIST_L1, 3); //apply distance transfrom
//    // normalize(dist, dist, -1, 1., NORM_MINMAX);
//        for(int x = 0; x<dist.rows; x++){
//         for (int y =0; y<dist.cols; y++){
//             if(dist.at<float>(x, y) < 0){
//                 std::cout<<dist.at<float>(x, y)<<"\n";
//             }
//         }
       
//     }
Mat raw_dist( image.size(), CV_32FC1 );
//std::cout<<pointPolygonTest( contours, Point2f(image.cols/2.0f,image.rows/2.0f), true )<<"\n";

for( int j = 0; j < raw_dist.rows; j++ )
{ for( int i = 0; i < raw_dist.cols; i++ )
     { raw_dist.at<float>(j,i) = pointPolygonTest( contours[0], Point2f(i,j), true ); 
     if(raw_dist.at<float>(j,i) > 0){
         std::cout<<"outside \n";
     }
    }
}

double minVal; double maxVal;
minMaxLoc( raw_dist, &minVal, &maxVal, 0, 0, Mat() );
minVal = abs(minVal); maxVal = abs(maxVal);

/// Depicting the  distances graphically
Mat drawing = Mat::zeros( image.size(), CV_8UC3 );

for( int j = 0; j < image.rows; j++ )
   { for( int i = 0; i < image.cols; i++ )
        {
          if( raw_dist.at<float>(j,i) < 0 )
            { drawing.at<Vec3b>(j,i)[0] = 255 - (int) abs(raw_dist.at<float>(j,i))*255/minVal; }
          else if( raw_dist.at<float>(j,i) > 0 )
            { drawing.at<Vec3b>(j,i)[2] = 255 - (int) raw_dist.at<float>(j,i)*255/maxVal; }
          else
            { drawing.at<Vec3b>(j,i)[0] = 255; drawing.at<Vec3b>(j,i)[1] = 255; drawing.at<Vec3b>(j,i)[2] = 255; }
        }
    }


//     Mat bgdModel, fgdModel, img, m; 
    
//     {
//         FileStorage fs("mymodels.yml", FileStorage::READ);
//         fs["BgdModel"] >> bgdModel;
//         fs["FgdModel"] >> fgdModel;
//     }

//     Mat1b mask (dist.rows, dist.cols,  uchar(GC_PR_BGD));
//     for(int i =0; i<mask.rows; ++i){
//         for(int j=0; j<mask.cols; ++j){
//             if (dist.at<float>(i, j)== 0.0f){
//                 circle(mask, Point(j,i), 1, Scalar(GC_BGD), 0);
//             }
//             else if (dist.at<float>(i, j)== 1.0f){
//                 circle(mask, Point(j,i), 1, Scalar(GC_FGD), 0);
//             }
//             else {
//                 circle(mask, Point(j,i), 1, Scalar(GC_PR_BGD), 0);
//             }
//         }
//     }
   

//    grabCut(image2, mask, Rect(), bgdModel, fgdModel, 1);
//     m =  ( mask == cv::GC_FGD) | ( mask == cv::GC_PR_FGD);
//     image2.copyTo(img, m);

//         for( int x = 0; x < img.rows; x++ ) {
//             for( int y = 0; y < img.cols; y++ ) {
//                 if ( img.at<Vec3b>(x, y) == Vec3b(0,0,0 )) {
//                     img.at<Vec3b>(x, y)[0] = 255;
//                     img.at<Vec3b>(x, y)[1] = 255;
//                     img.at<Vec3b>(x, y)[2] = 255;
//                 }
//             }
//         }

//         {
//             FileStorage fs("mymodels.yml", FileStorage::WRITE);
//             fs << "BgdModel" <<bgdModel;
//             fs << "FgdModel" << fgdModel;
            
//             }
    namedWindow("myWindow", WINDOW_AUTOSIZE);
    char exit_key_press = 0;
    do {
       imshow("myWindow", drawing);
        exit_key_press = cvWaitKey(1);
    }while (exit_key_press != '\x1b');

    // std::string outputName = "../SegmentedImages/" + filename.substr(10,18);
    // imwrite(outputName, img);

    return 1;
}