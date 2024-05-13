#version 460 core
#ifdef GL_ES
precision highp int;
precision highp float;
#endif

// From Vertexshader
in vec4 col;

out vec4 fragColor;

void main()
{
  fragColor = col;
}
