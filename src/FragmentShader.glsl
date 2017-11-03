#version 330 core

// Ouput data
layout(location = 0) out float z_e;
  float zNear = -30.0f;
    float zFar = 30.0f;
void main(){

  
    float z_b = gl_FragCoord.z;
   
    gl_FragDepth = gl_FragCoord.z;
    float z_n = (2.0f *  gl_FragDepth) - 1.0f;
    
    z_e = 2.0 * zNear * zFar / (zFar + zNear - z_n * (zFar - zNear));
     
}