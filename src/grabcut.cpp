// SOURCE: https://github.com/opencv/opencv/blob/master/samples/cpp/grabcut.cpp

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <segmentation.h>
#include <iostream>

using namespace cv;
Mat res;
static void help()
{
    std::cout << "\nThis program demonstrates GrabCut segmentation -- select an object in a region\n"
            "and then grabcut will attempt to segment it out.\n"
            "Call:\n"
            "./grabcut <image_name>\n"
        "\nSelect a rectangular area around the object you want to segment\n" <<
        "\nHot keys: \n"
        "\tESC - quit the program\n"
        "\tr - restore the original image\n"
        "\ts - save image \n"
        "\tn - next iteration\n"
        "\n"
        "\tleft mouse button - set rectangle\n"
        "\n"
        "\tCTRL+left mouse button - set GC_BGD pixels\n"
        "\tSHIFT+left mouse button - set GC_FGD pixels\n"
        "\n"
        "\tCTRL+right mouse button - set GC_PR_BGD pixels\n"
        "\tSHIFT+right mouse button - set GC_PR_FGD pixels\n" << std::endl;
}

const Scalar RED = Scalar(0,0,255);
const Scalar PINK = Scalar(230,130,255);
const Scalar BLUE = Scalar(255,0,0);
const Scalar LIGHTBLUE = Scalar(255,255,160);
const Scalar GREEN = Scalar(0,255,0);

const int BGD_KEY = EVENT_FLAG_CTRLKEY; //background
const int FGD_KEY = EVENT_FLAG_SHIFTKEY; //foreground

static void getBinMask( const Mat& comMask, Mat& binMask ){
    if( comMask.empty() || comMask.type()!=CV_8UC1 )
        CV_Error( Error::StsBadArg, "comMask is empty or has incorrect type (not CV_8UC1)" );
    if( binMask.empty() || binMask.rows!=comMask.rows || binMask.cols!=comMask.cols )
        binMask.create( comMask.size(), CV_8UC1 ); //size of comMask and type uint8
    binMask = comMask & 1; //set all 0 apart from elements which are one
}

class GCApplication{
public:
    enum{ NOT_SET = 0, IN_PROCESS = 1, SET = 2 };
    static const int radius = 2;
    static const int thickness = -1;

    void reset();
    void setImageAndWinName( const Mat& _image, const std::string& _winName );
    void showImage() const;
    void mouseClick( int event, int x, int y, int flags, void* param );
    int nextIter();
    int getIterCount() const { return iterCount; }
private:
    void setRectInMask();
    void setLblsInMask( int flags, Point p, bool isPr );

    const std::string* winName;
    const Mat* image;
    Mat mask;
    Mat bgdModel, fgdModel; //matrices used within algorithm internally

    uchar rectState, lblsState, prLblsState;
    bool isInitialized;

    Rect rect;
    std::vector<Point> fgdPxls, bgdPxls, prFgdPxls, prBgdPxls; //background and foreground pixels in class
    int iterCount;
};

void GCApplication::reset(){ //clear and reset everything
    if( !mask.empty() )
        mask.setTo(Scalar::all(GC_BGD));
    bgdPxls.clear(); fgdPxls.clear();
    prBgdPxls.clear();  prFgdPxls.clear();

    isInitialized = false;
    rectState = NOT_SET;
    lblsState = NOT_SET;
    prLblsState = NOT_SET;
    iterCount = 0;
}

void GCApplication::setImageAndWinName( const Mat& _image, const std::string& _winName  ){
    if( _image.empty() || _winName.empty() )
        return;
    image = &_image;
    winName = &_winName;
    mask.create( image->size(), CV_8UC1);
    reset();
}

void GCApplication::showImage() const{
    if( image->empty() || winName->empty() ) //end if image or window are empty
        return;
    res.release(); //clears res matrix
    Mat binMask;
    if( !isInitialized ) //not initilised then copy image to new Mat
        image->copyTo( res );
    else{
        getBinMask( mask, binMask ); //all zero apart from when mask is 1
        image->copyTo( res, binMask ); //only copies elements corresponding to non-zero elements of binMask;
    }

    std::vector<Point>::const_iterator it; //draw lines where mouse has drawn
    for( it = bgdPxls.begin(); it != bgdPxls.end(); ++it )
        circle( res, *it, radius, BLUE, thickness );
    for( it = fgdPxls.begin(); it != fgdPxls.end(); ++it )
        circle( res, *it, radius, RED, thickness );
    for( it = prBgdPxls.begin(); it != prBgdPxls.end(); ++it )
        circle( res, *it, radius, LIGHTBLUE, thickness );
    for( it = prFgdPxls.begin(); it != prFgdPxls.end(); ++it )
        circle( res, *it, radius, PINK, thickness );

    if( rectState == IN_PROCESS ) //display rectangle
        rectangle( res, Point( rect.x, rect.y ), Point(rect.x + rect.width, rect.y + rect.height ), GREEN, 2);

    for( int x = 0; x < res.rows; x++ ) {
        for( int y = 0; y < res.cols; y++ ) {
            if ( res.at<Vec3b>(x, y) == Vec3b(0,0,0 )) {
                res.at<Vec3b>(x, y)[0] = 255;
                res.at<Vec3b>(x, y)[1] = 255;
                res.at<Vec3b>(x, y)[2] = 255;
            }
        }
    }
    imshow( *winName, res );
}

void GCApplication::setRectInMask(){
    CV_Assert( !mask.empty() ); //fails if mask is full
    mask.setTo( GC_BGD ); //set mask to background
    rect.x = max(0, rect.x);
    rect.y = max(0, rect.y);
    rect.width = min(rect.width, image->cols-rect.x);
    rect.height = min(rect.height, image->rows-rect.y);
    (mask(rect)).setTo( Scalar(GC_PR_FGD) ); //set area within chosen rectangle to be probable foreground
}

void GCApplication::setLblsInMask( int flags, Point p, bool isPr ){
    std::vector<Point> *bpxls, *fpxls; // Pointer to vector containg background and foreground pixels
    uchar bvalue, fvalue;
    if( !isPr ){ //if def background or foreground
        bpxls = &bgdPxls; //fill vectors from class
        fpxls = &fgdPxls;
        bvalue = GC_BGD; //definately background
        fvalue = GC_FGD;
    }
    else{
        bpxls = &prBgdPxls;
        fpxls = &prFgdPxls;
        bvalue = GC_PR_BGD;
        fvalue = GC_PR_FGD;
    }
    if( flags & BGD_KEY ){
        bpxls->push_back(p); //add point to background pixels which adds to class pixels
        circle( mask, p, radius, bvalue, thickness ); //draw
    }
    if( flags & FGD_KEY ){
        fpxls->push_back(p);
        circle( mask, p, radius, fvalue, thickness );
    }
}

void GCApplication::mouseClick( int event, int x, int y, int flags, void* ){
    switch( event ){
    case EVENT_LBUTTONDOWN: // set rect or GC_BGD(GC_FGD) labels
        {
            bool isb = (flags & BGD_KEY) != 0, //background key pressed and left or right pressed
                 isf = (flags & FGD_KEY) != 0;
            if( rectState == NOT_SET && !isb && !isf ){ //no rectangle drawn and no extra key pressed
                rectState = IN_PROCESS; //start drawing rectangle
                rect = Rect( x, y, 1, 1 );
            }
            if ( (isb || isf) && rectState == SET ) //fine tuning
                lblsState = IN_PROCESS;
        }
        break;
    case EVENT_RBUTTONDOWN: // set GC_PR_BGD(GC_PR_FGD) labels
        {
            bool isb = (flags & BGD_KEY) != 0,
                 isf = (flags & FGD_KEY) != 0;
            if ( (isb || isf) && rectState == SET ) //if rectangle drawn then fine tuning
                prLblsState = IN_PROCESS;
        }
        break;
    case EVENT_LBUTTONUP: //left button released
        if( rectState == IN_PROCESS ){ // then finish rectangle
            rect = Rect( Point(rect.x, rect.y), Point(x,y) ); //rectangle from initial clicked point to current point
            rectState = SET;
            setRectInMask(); //set rect area to probable foreground in mask
            CV_Assert( bgdPxls.empty() && fgdPxls.empty() && prBgdPxls.empty() && prFgdPxls.empty() ); //check all current back/foreground pixels are empty
            showImage(); //displays image with correct stuff drawn on it
        }
        if( lblsState == IN_PROCESS ) //if fine tuning in progress
        {
            setLblsInMask(flags, Point(x,y), false); //add new point to def background/foreground pixels and draw
            lblsState = SET;
            showImage();
        }
        break;
    case EVENT_RBUTTONUP:
        if( prLblsState == IN_PROCESS ){
            setLblsInMask(flags, Point(x,y), true); //add new possible background/foreground pixels
            prLblsState = SET;
            showImage();
        }
        break;
    case EVENT_MOUSEMOVE:
        if( rectState == IN_PROCESS ){
            rect = Rect( Point(rect.x, rect.y), Point(x,y) ); //update rectangle position
            CV_Assert( bgdPxls.empty() && fgdPxls.empty() && prBgdPxls.empty() && prFgdPxls.empty() );
            showImage();
        }
        else if( lblsState == IN_PROCESS ){
            setLblsInMask(flags, Point(x,y), false); //update definite values
            showImage();
        }
        else if( prLblsState == IN_PROCESS ){ //update probable values
            setLblsInMask(flags, Point(x,y), true);
            showImage();
        }
        break;
    }
}

int GCApplication::nextIter(){
    if( isInitialized )
        grabCut( *image, mask, rect, bgdModel, fgdModel, 1 ); //use rectangle
    else
    {
        if( rectState != SET )
            return iterCount;

        if( lblsState == SET || prLblsState == SET ) //fine tuning
            grabCut( *image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_MASK );
        else
            grabCut( *image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_RECT );

        isInitialized = true;
    }
    iterCount++;

    bgdPxls.clear(); fgdPxls.clear();
    prBgdPxls.clear(); prFgdPxls.clear();

    return iterCount;
}

GCApplication gcapp;

static void on_mouse( int event, int x, int y, int flags, void* param ){ //flags depends on which button is clicked.
    gcapp.mouseClick( event, x, y, flags, param ); //event depends on which button pressed or released
}

int PriorSegmentation(std::string filename){
    if(filename.empty()){
        std::cerr<<"Error: no such file \n";
        return -1;
    }
    Mat image = imread( filename, 1 );
    if(image.empty()){
        std::cerr<<"Error: cannot open image \n";
        return -1;
    }
    help(); //displays usage terms


    const std::string winName = "image";
    namedWindow( winName, WINDOW_AUTOSIZE ); //creates window size of image
    setMouseCallback( winName, on_mouse, 0 );

    gcapp.setImageAndWinName( image, winName );
    gcapp.showImage();
    std::string outputName = "../SegmentedImages/seg" + filename.substr(15,18);

    for(;;){
        char c = (char)waitKey(0);
        switch( c ){
            case '\x1b':
                std::cout << "Exiting ...\n" ;
                goto exit_main;
            case 'r':
                std::cout << std::endl;
                gcapp.reset();
                gcapp.showImage();
                break;
            case 's':
                std::cout << "saving image \n";
                imwrite(outputName, res);
                break;
            case 'n': //if n pressed move on to next iteration
                int iterCount = gcapp.getIterCount();
                std::cout << "<" << iterCount << "... ";
                int newIterCount = gcapp.nextIter();
                if( newIterCount > iterCount ){
                    gcapp.showImage();
                std:: cout << iterCount << "> \n";
                }
                else
                    std::cout << "rect must be determined \n";
                break;
        }
    }

    exit_main:
    destroyWindow( winName );
    return 1;
}
