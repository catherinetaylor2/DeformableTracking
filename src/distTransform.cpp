#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <segmentation.h>

using namespace cv;

float thresh = 5.0f;
int contourThreshold = 100;

int SegmentImage(std::string PrevSeg, std::string filename, std::string depthName){
    if(PrevSeg.empty()||filename.empty()||depthName.empty()){
        std::cerr<<"Error:no such file in Segment image \n";
        return -1;
    }
    Mat image = imread( PrevSeg, 1 );
    Mat image2 = imread( filename, 1 );
    if(image.empty()||image2.empty()){
        std::cerr<<"Error: cannot open image in Segment image\n";
        return -1;
    }

    Mat depth;
    FileStorage fsd(depthName, FileStorage::READ);
    fsd["depth"] >> depth;


    Mat img_grey, canny_edges, edges;
    cvtColor(image, img_grey, CV_BGR2GRAY);
    blur( img_grey, img_grey, Size(3,3) );

    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    Canny( img_grey, canny_edges, contourThreshold, contourThreshold*3, 3 , true);
    int erosion_size = 1;
    Mat element = getStructuringElement( MORPH_RECT,
        Size( 2*erosion_size + 1, 2*erosion_size+1 ),
        Point( erosion_size, erosion_size ) );
        dilate( canny_edges, edges, element );
        erode( edges, edges, element );
        dilate( edges, edges, element );
        erode( edges, edges, element );

    findContours( edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
     
    Mat drawing2 = 255*Mat::ones( img_grey.size(), CV_8UC1 );
    for( int i = 0; i< contours.size(); i++ ){
      drawContours( drawing2, contours, i, Scalar(0,0,0), 1, 8, hierarchy, 0, Point() );
    }

    Mat raw_dist( image.size(), CV_32FC1 );

    for( int j = 0; j < raw_dist.rows; j++ ){ 
        for( int i = 0; i < raw_dist.cols; i++ ){ 
            raw_dist.at<float>(j,i) = pointPolygonTest( contours[0], Point2f(i,j), true ); 
        }
    }

//     double minVal; double maxVal;
//     minMaxLoc( raw_dist, &minVal, &maxVal, 0, 0, Mat() );
//     minVal = abs(minVal); maxVal = abs(maxVal);
//     Mat drawing = Mat::zeros( image.size(), CV_8UC3 );

//     for( int j = 0; j < image.rows; j++ )
//        { for( int i = 0; i < image.cols; i++ )
//             {
//               if( raw_dist.at<float>(j,i) < 0 )
//                 { drawing.at<Vec3b>(j,i)[0] = 255 - (int) abs(raw_dist.at<float>(j,i))*255/minVal; }
//               else if( raw_dist.at<float>(j,i) > 0 )
//                 { drawing.at<Vec3b>(j,i)[2] = 255 - (int) raw_dist.at<float>(j,i)*255/maxVal; }
//               else
//                 { drawing.at<Vec3b>(j,i)[0] = 255; drawing.at<Vec3b>(j,i)[1] = 255; drawing.at<Vec3b>(j,i)[2] = 255; }
//             }
//         }

    for(int x = 0; x<raw_dist.rows; x++){
        for (int y =0; y<raw_dist.cols; y++){
            if(raw_dist.at<float>(x, y)< -thresh){
                raw_dist.at<float>(x, y)= 0.0f;
            }
            else if(raw_dist.at<float>(x, y)>thresh){
                raw_dist.at<float>(x, y)= 1.0f;
            }
            else{
                raw_dist.at<float>(x, y)= 0.5f;
            }
        }
    }


    Mat bgdModel, fgdModel, img, m, imgD; 
    
    {
        FileStorage fs("mymodels.yml", FileStorage::READ);
        fs["BgdModel"] >> bgdModel;
        fs["FgdModel"] >> fgdModel;
    }

    Mat1b mask (raw_dist.rows, raw_dist.cols,  uchar(GC_PR_BGD));
    for(int i =0; i<mask.rows; ++i){
        for(int j=0; j<mask.cols; ++j){
            if (raw_dist.at<float>(i, j)== 0.0f){
                circle(mask, Point(j,i), 1, Scalar(GC_BGD), 0);
            }
            else if (raw_dist.at<float>(i, j)== 1.0f){
                circle(mask, Point(j,i), 1, Scalar(GC_FGD), 0);
            }
            else {
                circle(mask, Point(j,i), 1, Scalar(GC_PR_BGD), 0);
            }
        }
     }
   

    grabCut(image2, mask, Rect(), bgdModel, fgdModel, 1);
    m =  ( mask == cv::GC_FGD) | ( mask == cv::GC_PR_FGD);
    image2.copyTo(img, m);
    depth.copyTo(imgD, m);

        for( int x = 0; x < img.rows; x++ ) {
            for( int y = 0; y < img.cols; y++ ) {
                if ( img.at<Vec3b>(x, y) == Vec3b(0,0,0 )) {
                    img.at<Vec3b>(x, y)[0] = 255;
                    img.at<Vec3b>(x, y)[1] = 255;
                    img.at<Vec3b>(x, y)[2] = 255;
                }
            }
        }

        {
            FileStorage fs("mymodels.yml", FileStorage::WRITE);
            fs << "BgdModel" <<bgdModel;
            fs << "FgdModel" << fgdModel;
            
           }
    // namedWindow("myWindow", WINDOW_AUTOSIZE);
    // char exit_key_press = 0;
    // do {
    //    imshow("myWindow", erosion_dst);
    //     exit_key_press = cvWaitKey(1);
    // }while (exit_key_press != '\x1b');


    std::string outputName = "../SegmentedImages/" + filename.substr(10,18);
    imwrite(outputName, img);
    FileStorage fs2("../SegmentedDepth/" + depthName.substr(17,20), FileStorage::WRITE);
    fs2 << "depth" <<imgD;

    return 1;
}