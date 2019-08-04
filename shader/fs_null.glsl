#version 450
uniform vec2 resolution;

in vec4 gl_FragCoord;
out vec4 fragColor;

void main(void)
{
    float r = gl_FragCoord.x / resolution.x;
    float g = gl_FragCoord.y / resolution.y;
    float b = 0.0;

    fragColor = vec4(r, g, b, 1.0);
}