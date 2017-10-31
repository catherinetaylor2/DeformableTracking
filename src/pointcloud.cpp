#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <segmentation.h>
#include "math/mathTypes.h"
#include <calibration.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>

using namespace cv;




void getPointCloud(std::string DepthMap){

    transMatrix2D K;
    transMatrix3D L;

    K(0,0) = 525.0f;
    K(0,2) = 319.5f;
    K(1,1) = 525.0f;
    K(1,2) = 239.5f;
    K(2,2) = 1.0f;

    L<<1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

    Calibration calibration(K, L);

    std::cout<<"K "<<L<<"\n";

     Mat depthMat;
    FileStorage fs(DepthMap, FileStorage::READ);
    fs["depth"] >> depthMat;

    hVec2D uv ;
    hVec3D xyz;

    Mat PointCloud(depthMat.rows, depthMat.cols, CV_64FC3);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud->width    = 480;
    cloud->height   = 640;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

        for(int i = 0; i< depthMat.rows; ++i){
            for( int j = 0; j< depthMat.cols; ++j){
                if(depthMat.at<ushort>(i,j)!=0){
                    uv<<j,i,1;
                    xyz = calibration.Unproject(uv);
                    
                   
                    // uv.at<double>(0,0) = i;
                    // uv.at<double>(1,0) = j;
                    // xyz = invProj*uv;
                    xyz *= 1.0f/xyz(2)*depthMat.at<ushort>(i,j);
                    (*cloud)[depthMat.rows*j + i].x=xyz(0);
                    (*cloud)[depthMat.rows*j + i].y=xyz(1);
                    (*cloud)[depthMat.rows*j + i].z=xyz(2);
                    (*cloud)[depthMat.rows*j + i].r=255;
                    (*cloud)[depthMat.rows*j + i].g=255;
                    (*cloud)[depthMat.rows*j + i].b=255;
                    
                    
                  PointCloud.at<Vec3f>(i,j) = Vec3f(xyz(0), xyz(1), xyz(2));
                }
                else{
                   PointCloud.at<Vec3f>(i,j) = Vec3f(0,0,0);
                }
            }
        }
       pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
       viewer.showCloud (cloud);
      
        while (!viewer.wasStopped ())
       {
        }

  
}