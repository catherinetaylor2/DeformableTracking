#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <segmentation.h>

int main(){

   int priorSegementation =  PriorSegmentation("../Frames/frame0001.jpg");
   if(priorSegementation == -1){
       std::cerr<<"Error: Prior Segmentation has failed \n";
       return -1;
    }

   int segmentImage = SegmentImage("../SegmentedImages/seg0001.jpg");
   if(segmentImage == -1){
       std::cerr<<"Error: Segmentation has failed on frame "<< 1 << "\n";
   }


    return 0;
}