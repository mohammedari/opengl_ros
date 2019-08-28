#version 450 core
uniform float gridMapLayerHeight;

in vec4 gl_FragCoord;
out vec4 fragColor;

void main(void)
{
    const vec3 WHITE = vec3(1, 1, 1);
    const vec3 BLACK = vec3(1, 1, 1);

    fragColor = vec4(WHITE, 1.0);

    if (-gridMapLayerHeight / 2 < gl_FragCoord.z && gl_FragCoord.z < gridMapLayerHeight / 2)
        fragColor = vec4(WHITE, 1.0);
    else
        fragColor = vec4(BLACK, 1.0);
}