#version 460 core

// From Vertexshader
in vec4 col;
in vec4 transformedNormal;

out vec4 fragColor;

void main()
{
  fragColor = col * abs(transformedNormal.z);
}
