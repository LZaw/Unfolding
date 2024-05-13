#version 460 core
#ifdef GL_ES
precision highp int;
precision highp float;
#endif

uniform vec4 color;
uniform mat4 mvpMatrix;
uniform vec4 rootColor;
uniform int  rootNodeIndex;

layout(location=0) in vec4 position;

// To Fragmentshader
out vec4 col;

void main()
{
  gl_Position = mvpMatrix * position;
  if(gl_VertexID == rootNodeIndex){
    col = rootColor;
  }else{
    col = color;
  }
}
