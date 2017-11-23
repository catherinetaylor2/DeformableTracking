#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <segmentation.h>
#include <vector>
#include <ReadObj.hpp>
//#include "math/mathTypes.h"


using namespace cv;
Mat bgdModel, fgdModel; 


int main(){

//Load in original mesh
    float *Vertices, *Normals, *Textures; 
    int NumberOfFaces, *FaceVertices, *FaceNormals, *FaceTextures, NumberOfVertices;
    ObjFile mesh("../c.obj"); 
    if(!mesh.doesExist()){
        std::cerr<<"Error: Object file does not exist \n";
        return -1;
    }
    mesh.getMeshData(mesh, &FaceVertices, &FaceNormals, &FaceTextures, &Textures, &Normals, &Vertices, &NumberOfFaces, &NumberOfVertices);
   
   //hVec2D ExtraNode[NumberOfFaces];
    float* ExtraNode = new float [3*NumberOfFaces];
    float  X[4], Y[4], Z[4];
    int index1, index2, index3;
    float third = 1.0f/3.0f;

    for(int i = 0; i< NumberOfFaces; ++i){ //MOVE OUTSIDE THIS FILE
        index1 = FaceVertices[3*i];
        index2 = FaceVertices[3*i +1];
        index3 = FaceVertices[3*i +2];

        X[0] = Vertices[3*index1], Y[0] = Vertices[3*index1+1], Z[0] = Vertices[3*index1+2];
        X[1]=  Vertices[3*index2], Y[1] = Vertices[3*index2+1], Z[1] = Vertices[3*index2+2];
        X[2] = Vertices[3*index3], Y[2] = Vertices[3*index3+1], Z[2] = Vertices[3*index3+2];
       
        X[3] = third*(X[0]+X[1]+X[2]);
        Y[3] = third*(Y[0]+Y[1]+Y[2]);
        Z[3] = third*(Z[0]+Z[1]+Z[2]);
        Z[3] += -1*Normals[3*index1+1];
        ExtraNode[3*i] = X[3];
        ExtraNode[3*i+1] = Y[3];
        ExtraNode[3*i+2] = Z[3];
    }

   int priorSegementation =  PriorSegmentation("../Frames/0000.png");
   if(priorSegementation == -1){
       std::cerr<<"Error: Prior Segmentation has failed \n";
       return -1;
    }

  for(int i =0; i<5; ++i){
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
    
        int distFun = SegmentImage("../SegmentedImages/" + jPrev +".png", "../Frames/" + j +".png", "../depthMatrices/" + j + ".yml");
        if(distFun == -1){
            std::cerr<<"Error: Segmentation has failed on frame "<< i << "\n";
        }
        std::vector<hVec3D> pointCloud =  getPointCloud("../SegmentedDepth/"+ j+ ".yml");
        int k = getDepthMap(pointCloud, FaceVertices, Vertices, Normals, NumberOfFaces, NumberOfVertices, ExtraNode);
 }



    ObjFile::cleanUp(Vertices,Normals, Textures, FaceVertices, FaceNormals, FaceTextures);
    delete [] ExtraNode;

    return 0;
}