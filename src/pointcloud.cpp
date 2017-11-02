#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <segmentation.h>
#include "math/mathTypes.h"
#include <calib/calibration.h>
#include <calib/calibration.cpp>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <pcl/conversions.h>

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

    // std::cout<<"K "<<L<<"\n";

    //  Mat depthMat;
    // FileStorage fs(DepthMap, FileStorage::READ);
    // fs["depth"] >> depthMat;

    // hVec2D uv ;
    // hVec3D xyz;

    // Mat PointCloud(depthMat.rows, depthMat.cols, CV_64FC3);

    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    // cloud->width    = 480;
    // cloud->height   = 640;
    // cloud->is_dense = false;
    // cloud->points.resize (cloud->width * cloud->height);

    //     for(int i = 0; i< depthMat.rows; ++i){
    //         for( int j = 0; j< depthMat.cols; ++j){
    //             if(depthMat.at<ushort>(i,j)!=0){
    //                 uv<<i, j,1;
    //                 xyz = calibration.Unproject(uv);
                    
                   
    //                 // uv.at<double>(0,0) = i;
    //                 // uv.at<double>(1,0) = j;
    //                 // xyz = invProj*uv;
    //                 xyz *= 1.0f/xyz(2)*depthMat.at<ushort>(i,j);
    //                 std::cout<<"xyz "<<xyz<<"\n";
    //                 (*cloud)[depthMat.rows*j + i].x=xyz(0);
    //                 (*cloud)[depthMat.rows*j + i].y=xyz(1);
    //                 (*cloud)[depthMat.rows*j + i].z=xyz(2);
    //                 (*cloud)[depthMat.rows*j + i].r=255;
    //                 (*cloud)[depthMat.rows*j + i].g=255;
    //                 (*cloud)[depthMat.rows*j + i].b=255;
                    
                    
    //               PointCloud.at<Vec3f>(i,j) = Vec3f(xyz(0), xyz(1), xyz(2));
    //             }
    //             else{
    //                PointCloud.at<Vec3f>(i,j) = Vec3f(0,0,0);
    //             }
    //         }
    //     }
    //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //    viewer.showCloud (cloud);
      
    //     while (!viewer.wasStopped ())
    //    {
    //     }
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ("../pipe.obj",mesh);
    std::cout<<"mesh size "<<mesh.polygons.size()<<"\n";

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    std::cout<<"mesh "<<cloud.points[ mesh.polygons[0].vertices[0]].x<<"\n";

    hVec3D X;
    hVec2D x;
  Mat depthMap = Mat::zeros(480, 640, CV_64F);
    for (int i=0; i<mesh.polygons.size(); ++i){
        for(int j=0; j<3; ++j){
            X(0) = cloud.points[ mesh.polygons[i].vertices[j]].x;
            X(1) = cloud.points[ mesh.polygons[i].vertices[j]].y + 200;
            X(2) = cloud.points[ mesh.polygons[i].vertices[j]].z + 400;
            X(3) = 1;
            x = calibration.Project(X);
           // std::cout<<"x "<<X<<"\n";
            if(x(0)<480 && x(1)<640){
                if(depthMap.at<float>(x(0), x(1))!= 0){
                    if(X(2) > depthMap.at<float>((int)x(0), (int)x(1))){
                        depthMap.at<float>((int)x(0), (int)x(1)) = X(2);
                        std::cout<< X(2)<< " "<< depthMap.at<float>((int)x(0), (int)x(1))<<"\n";
                    }
                }
                else{
                    depthMap.at<float>((int)x(0), (int)x(1)) = X(2);
                }
            }
        }

    }
    


    
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->setBackgroundColor (0, 0, 0);
    // viewer->addPolygonMesh(mesh,"meshes",0);
    // viewer->addCoordinateSystem (1.0);
    // viewer->initCameraParameters ();
    // while (!viewer->wasStopped ()){
    //     viewer->spinOnce (100);
    //     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // }

    FileStorage fs2("../DEPTHMAP", FileStorage::WRITE);
    fs2 << "depth" <<depthMap;
   depthMap = depthMap!=0;
    namedWindow("myWindow", WINDOW_AUTOSIZE);
    char exit_key_press = 0;
    do {
     imshow("myWindow", depthMap);
        exit_key_press = cvWaitKey(1);
    }while (exit_key_press != '\x1b');

  
}