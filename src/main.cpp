#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <segmentation.h>

int main(){
   int priorSegementation =  PriorSegmentation("../Frames/frame01.jpg");
   if(priorSegementation == -1){
       std::cerr<<"Error: Prior Segmentation has failed \n";
       return -1;
   }
    return 0;
}