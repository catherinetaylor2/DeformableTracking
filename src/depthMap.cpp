#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <ReadObj.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/glm.hpp>
#include "shader.hpp"
#include <vector>
#include "math/mathTypes.h"
#include <calib/calibration.h>
#include <calib/calibration.cpp>
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
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


int main(){

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


//   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//   cloud->width    = width;
//   cloud->height   = height;
//   cloud->is_dense = false;
//   cloud->points.resize (cloud->width * cloud->height);
    
    std::vector<int> vis;
    for(int i = 0; i< NumberOfVertices; ++i){
        glm::vec4 V(Vertices[3*i],Vertices[3*i+1],Vertices[3*i+2],1);
        glm::vec3 v = MVP*V;
        int vx = (v.x + 1)*width/2.0f;
        int vy = (v.y +1)*height/2.0f;

        if(v.z<=pixels[vx + width*(vy)]){ //hmmmm?
            // (*cloud)[vx + width*vy].x=Vertices[3*i];
            // (*cloud)[vx + width*vy].y=Vertices[3*i+1];
            // (*cloud)[vx + width*vy].z=Vertices[3*i+2];
            // (*cloud)[vx + width*vy].r=255;
            // (*cloud)[vx + width*vy].g=255;
            // (*cloud)[vx + width*vy].b=255;
            vis.push_back(i);
        }
    }
    std::cout<<vis.size()<<"\n";

    ObjFile::cleanUp(Vertices,Normals, Textures, FaceVertices, FaceNormals, FaceTextures);
    glDeleteBuffers(1, &textureID);
    glDeleteFramebuffers(1, &framebuffer); 
    //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //    viewer.showCloud (cloud);
      
    //     while (!viewer.wasStopped ())
    //    {
    //     }
}