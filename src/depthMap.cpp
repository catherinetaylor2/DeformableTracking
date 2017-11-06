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

int getDepthMap(std::vector<hVec3D> PointCloud){

    float *Vertices, *Normals, *Textures; 
    int NumberOfFaces, *FaceVertices, *FaceNormals, *FaceTextures, NumberOfVertices;
    ObjFile mesh("../pipe1.obj"); 
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
            vis.push_back(i);
            VisPoints.push_back(xyz);
        }
    }
    std::cout<<vis.size()<<"\n";

    ObjFile::cleanUp(Vertices,Normals, Textures, FaceVertices, FaceNormals, FaceTextures);
    glDeleteBuffers(1, &textureID);
    glDeleteFramebuffers(1, &framebuffer); 

    //--------------------------------------------------------------------
    //ICP:

    //calculate centre of mass

   // float mPointCloud = 0.0f , mMesh = 0.0f;
   hVec3D mMesh;
   mMesh <<0.0f, 0.0f, 0.0f,0.0f;
    for(int i = 0; i<VisPoints.size(); ++i){
        mMesh += VisPoints[i];
    }
    mMesh *= 1.0f/VisPoints.size();

    hVec3D mPointCloud;
    mPointCloud <<0.0f, 0.0f, 0.0f,0.0f;
    for(int i = 0; i<PointCloud.size(); ++i){
        mPointCloud += PointCloud[i];
    }
    mPointCloud *= 1.0f/PointCloud.size();

    Eigen::MatrixXf mM(3,1), mP(3,1);
    mM<<mMesh[0], mMesh[1], mMesh[2];
    mP<<mPointCloud[0], mPointCloud[1], mPointCloud[2];



    //get new point sets

    std::vector<hVec3D> meshPoints;
    std::vector<hVec3D> cloudPoints;
    hVec3D currentPoint;

    for(int i = 0; i< VisPoints.size(); ++i){
        currentPoint = VisPoints[i] - mMesh;
        meshPoints.push_back(currentPoint);
    }
    Eigen::MatrixXf ePointCloud(PointCloud.size(), 3);    
    for(int i = 0; i< PointCloud.size(); ++i){
        currentPoint = PointCloud[i] - mPointCloud;
        cloudPoints.push_back(currentPoint);
        ePointCloud.row(i)<<currentPoint(0),currentPoint(1),currentPoint(2);
    }

    //make eigen filled with point cloud

    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> KDTree;
    
    KDTree PointCloudTree(3, ePointCloud, 10);
    PointCloudTree.index->buildIndex();
    std::vector<std::pair<int, int>> pairs;
    
    for(int i = 0; i < VisPoints.size(); ++i){
        std::vector<size_t> ret_indexes(1);
        std::vector<float>  out_dists_sqr(1);
        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
        std::vector<float> q(3); 
        q[0] = (meshPoints[i])(0);
        q[1] = (meshPoints[i])(1);
        q[2] = (meshPoints[i])(2);
        
        
        PointCloudTree.index->findNeighbors(resultSet, &q[0], nanoflann::SearchParams(10));
        std::pair<int,int> currentPair;
        currentPair.first = i;
        currentPair.second = ret_indexes[0];
        pairs.push_back(currentPair);
    }


    Eigen::MatrixXf xi(3,1), pi(3,1), U(3,3), VT(3,3), R, t;
    Eigen::MatrixXf  W = Eigen::MatrixXf::Zero(3,3);
    Eigen::MatrixXf  D = Eigen::MatrixXf::Zero(3,3);

    for(int i =0; i< VisPoints.size(); ++i){
        int  meshIndex = pairs[i].first;
        int cloudIndex = pairs[i].second;
        //std::cout<<"mesh "<<meshIndex<<" cloud "<<cloudIndex<<"\n";
        xi<<meshPoints[meshIndex](0), meshPoints[meshIndex](1), meshPoints[meshIndex](2);
        pi<<cloudPoints[cloudIndex](0),cloudPoints[cloudIndex](1), cloudPoints[cloudIndex](2);
        W += xi*pi.transpose();
    }
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    D(0,0) = svd.singularValues()(0,0);
    D(1,1) = svd.singularValues()(1,0);
    D(2,2) = svd.singularValues()(2,0);
    U = svd.matrixU();
    VT = svd.matrixV().transpose();
   
    R = U*VT;
    t = mP - R*mM;

    for(int i=0; i<VisPoints.size(); ++i){
        
    }


        return 1;
}