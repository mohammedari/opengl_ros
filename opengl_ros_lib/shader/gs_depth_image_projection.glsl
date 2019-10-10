#version 450 core

layout(points) in;
layout(lines, 2) out;

in input_vertex
{
    vec4  position;
    vec2  colorUV;
    float height;
} input[1];

out vec4  position;
out vec2  colorUV;
out float height;

void main(void)
{
    const vec2 lineOrigin = vec2(0, 1, 0, 1); //Top center of the image

    position = lineOrigin;
    colorUV  = gl_in[0].colorUV;
    height   = gl_in[0].height;
    EmitVertex();

    position = gl_in[0].position;
    colorUV  = gl_in[0].colorUV;
    height   = gl_in[0].height;
    EmitVertex();

    EndPrimitive();
}