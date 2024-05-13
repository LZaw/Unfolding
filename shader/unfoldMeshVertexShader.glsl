#version 460 core

uniform mat4 mvpMatrix;
uniform mat4 normalMatrix;
uniform vec4 uniformColor;
uniform bool useIndividualColors;

layout(location=0) in vec3 normal;
layout(location=1) in vec3 position;
layout(location=2) in vec3 individualColor;

// To Fragmentshader
out vec4 col;
out vec4 transformedNormal;

void main()
{
  gl_Position = mvpMatrix * vec4(position, 1.);
  transformedNormal = normalMatrix * vec4(normal, 0.);
  if(useIndividualColors){
    col = vec4(individualColor, 1.);
  }else{
    col = uniformColor;
  }
}
