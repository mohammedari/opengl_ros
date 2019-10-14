#version 450 core

layout(points) in;
layout(line_strip, max_vertices = 2) out;

in vertex
{
    vec4  position;
    vec2  colorUV;
    float height;
} input_vertices[1];

out vec2  colorUV;
out float height;

void main(void)
{
    const vec4 lineOrigin = vec4(0, 1, 0, 1); //Top center of the image

    gl_Position = lineOrigin;
    colorUV  = input_vertices[0].colorUV;
    height   = input_vertices[0].height;
    EmitVertex();

    gl_Position = input_vertices[0].position; 
    colorUV  = input_vertices[0].colorUV;
    height   = input_vertices[0].height;
    EmitVertex();

    EndPrimitive();
}