#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <string>


cv::Point p1 = cv::Point(0,0);
cv::Point p2 = cv::Point(0,0);
int numberOfClicks = 0;
bool drawRect = false;

void mouse(int event, int x, int y){
    if(event == cv::EVENT_LBUTTONDOWN){ //draw rectangle
        if(numberOfClicks == 0){
            p1 = cv::Point(x,y);
            ++ numberOfClicks;
            return;
        }
        if(numberOfClicks == 1){
            p2 = cv::Point(x,y);
            drawRect = true;
            numberOfClicks = 0;
        }
    }
    if(event == cv::EVENT_MOUSEMOVE){
       //std::cout<<"x "<<x<<" y "<<y<<"\n";
    }
}
void CallBackFunc(int event, int x, int y, int flags, void* userData){

    mouse(event, x, y);
    
}

int main(int argc, char** argv){

    std::string imgName;
    if(argc > 1){
        imgName = argv[1];
    }
    else{
        imgName = "frame01.jpg"; // default image
    }
 
    cv::Mat img = cv::imread(imgName, cv::IMREAD_COLOR);
    cv::Mat imgCopy = img.cv::Mat::clone();
    if(img.empty()){
        std::cerr<<"Error: can not open image \n";
        return -1;
    }

    cv::namedWindow("myWindow", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("myWindow", CallBackFunc, NULL);
   // imshow("myWindow", img);
    

    char exit_key_press = 0;
    do {
        if(drawRect==true){
            std::cout<<"draw \n";
            cv::rectangle(imgCopy, p1, p2, cv::Scalar(255,0,0), 1, 0);
            drawRect = false;
        }
        imshow("myWindow", imgCopy);
        exit_key_press = cvWaitKey(1);
    }while (exit_key_press != 'q');

    return 0;
}