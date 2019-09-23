#version 450 core
uniform float gridMapLayerHeight;
uniform float gridMapAccumulationWeight;

out float fragmentColor;

void main(void)
{
    if (-gridMapLayerHeight / 2 < gl_FragCoord.z && gl_FragCoord.z < gridMapLayerHeight / 2)
        fragmentColor = gridMapAccumulationWeight;
    else
        fragmentColor = 0;
}