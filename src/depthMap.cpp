#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <ReadObj.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/glm.hpp>
#include "shader.hpp"
#include <vector>
#include <segmentation.h>
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <math.h> 

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
#include <vector>


float ICP(std::vector<hVec3D>* VisPoints, std::vector<hVec3D> PointCloud, Eigen::MatrixXf *R, Eigen::MatrixXf* t){

    //Use kd tree to find nearest neighbours

    Eigen::MatrixXf ePointCloud(PointCloud.size(), 3);    
    for(int i = 0; i< PointCloud.size(); ++i){
       ePointCloud.row(i)<<PointCloud[i](0),PointCloud[i](1),PointCloud[i](2);
    }

    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> KDTree;
    KDTree PointCloudTree(3, ePointCloud, 10);
    PointCloudTree.index->buildIndex();
    std::vector<std::pair<int, int>> pairs;
    
    for(int i = 0; i < VisPoints->size(); ++i){
        std::vector<size_t> ret_indexes(1);
        std::vector<float>  out_dists_sqr(1);
        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
        std::vector<float> q(3); 
        q[0] = (*VisPoints)[i](0);
        q[1] = (*VisPoints)[i](1);
        q[2] = (*VisPoints)[i](2);        
        
        PointCloudTree.index->findNeighbors(resultSet, &q[0], nanoflann::SearchParams(10));
        std::pair<int,int> currentPair;
        currentPair.first = i;
        currentPair.second = ret_indexes[0];
        pairs.push_back(currentPair);
    }
    //calculate centre of mass

    hVec3D mMesh, mPointCloud;
    mMesh<<0,0,0,1;
    mPointCloud<<0,0,0,1;
    for(int i=0; i<VisPoints->size(); ++i){
        mMesh += (*VisPoints)[i];
        mPointCloud += PointCloud[pairs[i].second];
    }
    mMesh *= 1.0f/VisPoints->size();
    mPointCloud *= 1.0f/VisPoints->size();   

    Eigen::MatrixXf mM(3,1), mP(3,1);
    mM<<mMesh[0], mMesh[1], mMesh[2];
    mP<<mPointCloud[0], mPointCloud[1], mPointCloud[2];

    //get new point sets

    std::vector<hVec3D> meshPoints;
    std::vector<hVec3D> cloudPoints;
    hVec3D currentPointM, currentPointP;

    for(int i = 0; i< VisPoints->size(); ++i){
        currentPointM = (*VisPoints)[i] - mMesh;
        meshPoints.push_back(currentPointM);
        currentPointP = PointCloud[pairs[i].second] - mPointCloud;
        cloudPoints.push_back(currentPointP);
    }
    
   Eigen::MatrixXf xi(3,1), pi(3,1), U(3,3), VT(3,3); //R, t;
    Eigen::MatrixXf  W = Eigen::MatrixXf::Zero(3,3);
    Eigen::MatrixXf  D = Eigen::MatrixXf::Zero(3,3);

    for(int i =0; i< VisPoints->size(); ++i){
        pi<<meshPoints[i](0), meshPoints[i](1), meshPoints[i](2);
        xi<<cloudPoints[i](0),cloudPoints[i](1), cloudPoints[i](2);
        W += xi*pi.transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    D(0,0) = svd.singularValues()(0,0);
    D(1,1) = svd.singularValues()(1,0);
    D(2,2) = svd.singularValues()(2,0);
    U = svd.matrixU();
    VT = svd.matrixV().transpose();
    Eigen::MatrixXf RTemp, tTemp;
    RTemp = U*VT;
    tTemp = mP - RTemp*mM;
   
    *R = RTemp*(*R); //update R and t
    *t = tTemp+(*t);

    for(int i=0; i<VisPoints->size(); ++i){
        pi<<(*VisPoints)[i](0), (*VisPoints)[i](1), (*VisPoints)[i](2);
        pi = RTemp*pi+tTemp;
        currentPointM<<pi(0,0), pi(1,0), pi(2,0), 1;
        (*VisPoints)[i] = currentPointM;
    }

    float Error = 0;
    Eigen::MatrixXf E(3,1), CM(3,1), CP(3,1);
    for(int i = 0; i<VisPoints->size(); ++i){
        int index = pairs[i].second;
        CM<<(*VisPoints)[i](0,0) ,(*VisPoints)[i](1,0),(*VisPoints)[i](2,0);
        CP<<PointCloud[index](0,0) ,PointCloud[index](1,0),PointCloud[index](2,0);
        E = CP - CM; 
        Error += E(0,0)*E(0,0) + E(1,0)*E(1,0) + E(2,0)*E(2,0);
    }
    Error *= 1.0f/VisPoints->size();
    return Error;
 }

int getDepthMap(std::vector<hVec3D> PointCloud){

    float *Vertices, *Normals, *Textures; 
    int NumberOfFaces, *FaceVertices, *FaceNormals, *FaceTextures, NumberOfVertices;
    ObjFile mesh("../pipe.obj"); 
    if(!mesh.doesExist()){
        std::cerr<<"Error: Object file does not exist \n";
        return -1;
    }
    mesh.getMeshData(mesh, &FaceVertices, &FaceNormals, &FaceTextures, &Textures, &Normals, &Vertices, &NumberOfFaces, &NumberOfVertices);

    if(!glfwInit()){ // initialize GLFW
        std::cerr<<"Error: failed to initialize GLFW \n";
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES,4); // select openGL version settings 
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    int width = 1000;
    int height = 1000;

    GLFWwindow *window; //create openGL window
    window = glfwCreateWindow(width, height, "Shadow Puppet", NULL, NULL);
    
    if(window==NULL){
        std::cerr<<"Error: failed to open window";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glewExperimental = true;
    if(glewInit()!=GLEW_OK){
        std::cerr<<"Error: failed to initialize GLEW \n";
        return -1;
    }

    GLuint VertexArrayID; //Create vertex array object and set to be current
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); //set keys for window

    glm::mat4 ProjectionMatrix = glm::ortho<float>(-200,200,-200,200,-30,30);
    glm::mat4 ViewMatrix = glm::lookAt(glm::vec3(0,0, -30), glm::vec3(0,0,1), glm::vec3(0,1,0)); 
    glm::mat4 MVP = ProjectionMatrix*ViewMatrix;

    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_RED, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textureID, 0); 
    
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE){
        std::cerr << "Error: frame buffer not complete \n" ; //check depth buffer is complete
        return -1;
    } 

    GLuint puppet_vertexbuffer;
    glGenBuffers(1, &puppet_vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, puppet_vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, 3*NumberOfVertices*sizeof(float), Vertices, GL_DYNAMIC_DRAW);

    unsigned int* indices = new unsigned int [3*NumberOfFaces]; // create array containing position of vertices.
    for(int i=0; i<3*NumberOfFaces; i+=3){
        indices[i]= FaceVertices[i];
        indices[i+1]=FaceVertices[i+1];
        indices[i+2]=FaceVertices[i+2];   
    }

    GLuint IBO;
    glGenBuffers(1, &IBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3*NumberOfFaces*sizeof(unsigned int), indices, GL_DYNAMIC_DRAW); 

    GLuint programID = LoadShaders("shaders/VertexShader.glsl", "shaders/FragmentShader.glsl");
    GLuint MatrixID = glGetUniformLocation(programID, "MVP");


    //do{// glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer); //write to buffer  
        glBindTexture(GL_TEXTURE_2D, textureID);

        glUseProgram(programID);
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
        
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, puppet_vertexbuffer);
                
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO);
        GLint posAttrib = glGetAttribLocation(programID, "position");
        glEnableVertexAttribArray(posAttrib);
        glVertexAttribPointer(0, // 0  is vertex
                                3, //size of information
                                GL_FLOAT, // type of the data
                                GL_FALSE, // normalised?
                                0, // stride
                                0 // offset
                            );        
        glDrawElements(GL_TRIANGLES, 3*NumberOfFaces, GL_UNSIGNED_INT,0); // draw mesh

        float pixels [width*height];
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, pixels );

        glfwSwapBuffers(window);
        glfwPollEvents();
        
  //}while(glfwGetKey(window, GLFW_KEY_ESCAPE)!=GLFW_PRESS && glfwWindowShouldClose(window)==0);
    
  std::vector<hVec3D> RigidPoints;
  Eigen::MatrixXf currentPos(3,1), newPos;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud->width    = width;
  cloud->height   = height;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

    std::vector<int> vis;
    std::vector<hVec3D> VisPoints;
    hVec3D xyz;
    for(int i = 0; i< NumberOfVertices; ++i){
        glm::vec4 V(Vertices[3*i],Vertices[3*i+1],Vertices[3*i+2],1);
        glm::vec3 v = MVP*V;
        int vx = (v.x + 1)*width/2.0f;
        int vy = (v.y +1)*height/2.0f;
        xyz << V.x, V.y, V.z, 1;
        
        if(v.z<=pixels[vx + width*(vy)]){ 
            vis.push_back(i); //contains the index values
            VisPoints.push_back(xyz);
           
        }
    }
    //--------------------------------------------------------------------
    //ICP:
    float E = 1000000000.0f;
    float threshold =15;
    Eigen::MatrixXf R(3,3), t(3,1);
    t<<0,0,0;
    R<<1,0,0,
        0,1,0,
        0,0,1;
   while(E>threshold){
        E = ICP( &VisPoints, PointCloud, &R, &t);
 }

    //transform mesh;

    for(int i =0; i<NumberOfVertices; ++i){
        glm::vec4 V(Vertices[3*i],Vertices[3*i+1],Vertices[3*i+2],1);
        glm::vec3 v = MVP*V;
        int vx = (v.x + 1)*width/2.0f;
        int vy = (v.y +1)*height/2.0f;

        currentPos<< Vertices[3*i], Vertices[3*i+1], Vertices[3*i+2];
        newPos = R*currentPos+t;
        xyz<<newPos(0,0), newPos(1,0), newPos(2,0),1;
        RigidPoints.push_back(xyz);
        
       // if(vx + width*vy>=0 && vx + width*vy<=width*height){
            // (*cloud)[vx + width*vy].x=newPos(0,0);
            // (*cloud)[vx + width*vy].y=newPos(1,0);
            // (*cloud)[vx + width*vy].z=newPos(2,0);
            // (*cloud)[vx + width*vy].r=255;
            // (*cloud)[vx + width*vy].g=255;
            // (*cloud)[vx + width*vy].b=255;
       // }
    }
//     for(int i =0; i< PointCloud.size(); ++i){
//         glm::vec4 V(PointCloud[i](0),PointCloud[i](1),PointCloud[i](2),1);
//         glm::vec3 v = MVP*V;
//         int vx = (v.x + 1)*width/2.0f;
//         int vy = (v.y +1)*height/2.0f;
        
//        // if(vx + width*vy>=0 && vx + width*vy<=width*height){
//             (*cloud)[vx + width*vy].x=V.x;
//             (*cloud)[vx + width*vy].y=V.y;
//             (*cloud)[vx + width*vy].z=V.z;
//             (*cloud)[vx + width*vy].r=255;
//             (*cloud)[vx + width*vy].g=0;
//             (*cloud)[vx + width*vy].b=0;
//     }


 
//    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//    viewer.showCloud (cloud);
  
//     while (!viewer.wasStopped ())
//    {
//     }

    //NEAREST NEIGHBOUR CORRESPONDANCES
    //FOR FEM MESH

    Eigen::MatrixXf ePointCloud(PointCloud.size(), 3);    
    for(int i = 0; i< PointCloud.size(); ++i){
       ePointCloud.row(i)<<PointCloud[i](0),PointCloud[i](1),PointCloud[i](2);
    }

    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> KDTree;
    KDTree PointCloudTree(3, ePointCloud, 10);
    PointCloudTree.index->buildIndex();
    std::vector<std::pair<int, int>> Meshpairs;
    
    for(int i = 0; i < VisPoints.size(); ++i){
        std::vector<size_t> ret_indexes(1);
        std::vector<float>  out_dists_sqr(1);
        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
        std::vector<float> q(3); 
        q[0] = (VisPoints)[i](0);
        q[1] = (VisPoints)[i](1);
        q[2] = (VisPoints)[i](2);        
        
        PointCloudTree.index->findNeighbors(resultSet, &q[0], nanoflann::SearchParams(10));
        std::pair<int,int> currentPair;
        currentPair.first = i;
        currentPair.second = ret_indexes[0];
        Meshpairs.push_back(currentPair);
    }

    //FOR POINT CLOUD
    Eigen::MatrixXf eVisPoints(VisPoints.size(), 3);
    for(int i =0; i< VisPoints.size(); ++i){
        eVisPoints.row(i)<<VisPoints[i](0), VisPoints[i](1), VisPoints[i](2);
    }
    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> KDTreeP;
    KDTreeP MeshTree(3, eVisPoints, 10);
    MeshTree.index->buildIndex();
    std::vector<std::pair<int, int>> Cloudpairs;
    
    for(int i = 0; i < PointCloud.size(); ++i){
        std::vector<size_t> ret_indexes(1);
        std::vector<float>  out_dists_sqr(1);
        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
        std::vector<float> q(3); 
        q[0] = (PointCloud)[i](0);
        q[2] = (PointCloud)[i](2);        
        q[1] = (PointCloud)[i](1);
        
        MeshTree.index->findNeighbors(resultSet, &q[0], nanoflann::SearchParams(10));
        std::pair<int,int> currentPair;
        currentPair.first = i;
        currentPair.second = ret_indexes[0];
        Cloudpairs.push_back(currentPair);
    }

    //GET EXTERNAL FORCES

    float lambda = 1.0f;
    float k = 0.7f; //youngs modulous
    float w_i = 1.0f;
    int N [VisPoints.size()];
    hVec3D Nsum [VisPoints.size()];
    hVec3D force[NumberOfVertices];
    std::vector<hVec3D> y;
    hVec3D currentY;
 //  int y[VisPoints.size()];
    for(int i = 0; i< VisPoints.size(); ++i){
        N[i] = 0;
        Nsum[i]<< 0,0,0,0;
    }
    for(int i = 0; i<PointCloud.size(); ++i){
        N[Cloudpairs[i].second] ++; //number of times each vis point is mapped to cloud points
        Nsum[Cloudpairs[i].second] += PointCloud[Cloudpairs[i].second];
    }

    for (int i = 0; i<VisPoints.size(); ++i){
        if(Nsum[i](3)>0){
            currentY = lambda*PointCloud[Meshpairs[i].second] + (1-lambda)*1.0f/(float)N[i]*Nsum[i];
            y.push_back(currentY);
        }
        else{
           currentY = lambda*PointCloud[Meshpairs[i].second] + (1- lambda)*VisPoints[i]; 
           y.push_back(currentY);
        }
    }
    for(int i =0; i< NumberOfVertices; ++i){ // non vis points have 0 forvce
        force[i] << 0.0f, 0.0f, 0.0f, 0.0f;
    }
    for (int i = 0; i<VisPoints.size(); ++i){
        force[vis[i]] = w_i*k*(VisPoints[i] - y[i]);
    }

    //ADD IN EXTRA NODE + Find volume of each tetrahedron
    hVec2D ExtraNode[NumberOfFaces];
    float volume;
    float a[4], b[4], c[4], d[4], X[4], Y[4], Z[4];
    Eigen::MatrixXf A(3,3), B(3,3), C(3,3), D(3,3), L(6, 12);
    int index1, index2, index3, ii, jj, kk, ll;
    float third = 1.0f/3.0f;
    for(int i = 0; i< NumberOfFaces; ++i){
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

        volume = 1.0f/6.0f*((X[1]-X[0])*((Y[1]-Y[2])*(Z[2]-Z[3])-(Y[2]-Y[3])*(Z[1]-Z[2])) + (X[2]-X[1])*((Y[2]-Y[3])*(Z[0]-Z[1])-(Y[0]-Y[1])*(Z[2]-Z[3])) + (X[3]-X[2])*((Y[0]-Y[1])*(Z[1]-Z[2])-(Y[1]-Y[2])*(Z[0]-Z[1])));        

        for(int j = 0; j<4; ++j){
            ii = j;
            jj = (ii + 1)*(ii <= 2);
            kk = (jj + 1)*(jj<=2); 
            ll = (kk + 1)*(kk<=2);

            A<< X[jj], Y[jj], Z[jj],
                X[kk], Y[kk], Z[kk],
                X[ll], Y[ll], Z[ll];

            B<< 1, Y[jj], Z[jj],
                1, Y[kk], Z[kk],
                1, Y[ll], Z[ll];

            C<< Y[jj], 1,  Z[jj],
                Y[kk], 1,  Z[kk],
                Y[ll], 1,  Z[ll];

            D<< Y[jj], Z[jj], 1,
                Y[kk], Z[kk], 1,
                Y[ll], Z[ll], 1;

            a[ii] = A.determinant();
            b[ii] = -1*B.determinant();
            c[ii] = C.determinant();
            d[ii] = -1*D.determinant();

            L(0, 3*ii) = b[ii];
            L(1, 3*ii + 1) = c[ii];
            L(2, 3*ii + 2) = d[ii];
            L(3, 3*ii) = c[ii];
            L(4, 3*ii + 2) = c[ii];
            L(3, 3*ii + 1) = b[ii];
            L(5, 3*ii + 2) = b[ii];
            L(4, 3*ii + 1) = d[ii];
            L(5, 3*ii) = d[ii];
        }
        L *= 1/(2*volume);
    }



    //might want to discard some points

    ObjFile::cleanUp(Vertices,Normals, Textures, FaceVertices, FaceNormals, FaceTextures);
    glDeleteBuffers(1, &textureID);
    glDeleteFramebuffers(1, &framebuffer); 

    return 1;
}