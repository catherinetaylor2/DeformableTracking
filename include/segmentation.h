#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <condition_variable>
#include <iostream>
#include "math/mathTypes.h"

int PriorSegmentation(std::string filename);
//int SegmentImage(std::string distMapName, std::string imageName);
int SegmentImage(std::string Prevfilename, std::string filename, std::string depth);
std::vector<hVec3D> getPointCloud(std::string DepthMap);
int getDepthMap(std::vector<hVec3D> PointCloud, int* FaceVertices, float* Vertices, float* Normals, int NumberOfFaces, int NumberOfVertices, float* ExtraNode);