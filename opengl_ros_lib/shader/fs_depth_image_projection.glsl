#version 450 core
uniform float gridMapLayerHeight;
uniform float gridMapAccumulationWeight;

out int fragmentColor;

void main(void)
{
    //const int EXIST     = 100;
    //const int NOT_EXIST = -1;

    const int EXIST     = -1; //test
    const int NOT_EXIST = 128;  //test

    if (-gridMapLayerHeight / 2 < gl_FragCoord.z && gl_FragCoord.z < gridMapLayerHeight / 2)
        fragmentColor = EXIST;
    else
        fragmentColor = NOT_EXIST;
}