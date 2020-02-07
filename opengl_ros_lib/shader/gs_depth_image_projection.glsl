#version 450 core

uniform vec4 lineOrigin; // camera position in projection coordinate

layout(points) in;
layout(line_strip, max_vertices = 2) out;

in vertex
{
    vec4 position;
    float depth;
} input_vertices[1];

out float depth;    //depth value of the pixel on the line
out float hitDepth; //depth value of the end of the line

void main(void)
{
    gl_Position = lineOrigin;
    depth       = 0;
    hitDepth    = input_vertices[0].depth;
    EmitVertex();

    gl_Position = input_vertices[0].position; 
    depth       = input_vertices[0].depth;
    hitDepth    = input_vertices[0].depth;
    EmitVertex();

    EndPrimitive();
}
