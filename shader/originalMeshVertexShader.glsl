#version 460 core

uniform vec4 color;
uniform mat4 mvpMatrix;
uniform mat4 normalMatrix;

layout(location=0) in vec3 normal;
layout(location=1) in vec3 position;

// To Fragmentshader
out vec4 col;
out vec4 transformedNormal;

void main()
{
  gl_Position = mvpMatrix * vec4(position, 1.);
  transformedNormal = normalMatrix * vec4(normal, 0.);
  col = color;
}
