#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <segmentation.h>

using namespace cv;
Mat bgdModel, fgdModel; 


int main(){

   int priorSegementation =  PriorSegmentation("../Frames/0000.png");
   if(priorSegementation == -1){
       std::cerr<<"Error: Prior Segmentation has failed \n";
       return -1;
    }

    for(int i =0; i<21; ++i){
        std::cout<<"frame "<<i<<"\n";
        std::string j, jPrev;
        if(i < 10){
            jPrev = "000" + std::to_string(i);
        }
        else if(i>=10&&i<100){
            jPrev = "00" + std::to_string(i);
        }
        if((i+1) < 10){
            j = "000" + std::to_string(i+1);
        }
        else if((i+1)>=10&&(i+1)<100){
            j = "00" + std::to_string(i+1);
        }
       // std::cout<<"../SegmentedImages/" + jPrev +"png"<<"\n";
        int distFun = SegmentImage("../SegmentedImages/" + jPrev +".png", "../Frames/" + j +".png");
        if(distFun == -1){
            std::cerr<<"Error: Segmentation has failed on frame "<< i << "\n";
        }
    }

    return 0;
}