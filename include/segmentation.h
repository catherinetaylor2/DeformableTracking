#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <condition_variable>
#include <iostream>

int PriorSegmentation(std::string filename);
//int SegmentImage(std::string distMapName, std::string imageName);
int SegmentImage(std::string Prevfilename, std::string filename, std::string depth);
void getPointCloud(std::string DepthMap);