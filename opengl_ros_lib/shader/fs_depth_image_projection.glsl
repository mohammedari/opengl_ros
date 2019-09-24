#version 450 core
uniform float gridMapLayerHeight;
uniform float gridMapAccumulationWeight;

out float fragmentColor;

void main(void)
{
    //If the height is out of the range, discard the fragment.
    if (gl_FragCoord.z < -gridMapLayerHeight / 2 || gridMapLayerHeight / 2 < gl_FragCoord.z)
        discard;

    fragmentColor = gridMapAccumulationWeight;
}