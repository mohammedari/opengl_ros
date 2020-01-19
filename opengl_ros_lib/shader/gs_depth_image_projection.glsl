#version 450 core

layout(points) in;
layout(line_strip, max_vertices = 2) out;

in vertex
{
    vec4 position;
    float depth;
} input_vertices[1];

out float depth;
out float hitDepth;

void main(void)
{
    const vec4 lineOrigin = vec4(0, 1, 0, 1); //Top center of the image

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