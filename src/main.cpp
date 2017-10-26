#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <segmentation.h>

using namespace cv;
Mat bgdModel, fgdModel; 


int main(){

   int priorSegementation =  PriorSegmentation("../Frames/frame0001.jpg");
   if(priorSegementation == -1){
       std::cerr<<"Error: Prior Segmentation has failed \n";
       return -1;
    }

   int distFun = SegmentImage("../SegmentedImages/seg0001.jpg", "../Frames/frame0002.jpg");
   if(distFun == -1){
       std::cerr<<"Error: Segmentation has failed on frame "<< 1 << "\n";
   }

  // int s = SegmentImage("../Masks/mask0001.jpg","../Frames/frame0002.jpg");

    return 0;
}