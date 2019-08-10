#version 450 core
uniform vec2 resolution;
uniform sampler2D texture;

in vec4 gl_FragCoord;
out vec4 fragColor;

void main(void)
{
    float u = gl_FragCoord.x / resolution.x;
    float v = gl_FragCoord.y / resolution.y;

    vec3 color = texture2D(texture, vec2(u, v)).xyz;
    fragColor = vec4(color, 1.0);
}