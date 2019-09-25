#version 450 core
uniform sampler2D colorTexture; 

uniform float gridMapLayerHeight;
uniform float gridMapAccumulationWeight;

in vec2 colorUV; //The UV coordinate of corresponding pixel in the color image 
                 //x = 0 to 1
                 //y = 0 to 1

out vec3 fragmentColor;

void main(void)
{
    //If the height is out of the range, discard the fragment.
    if (gl_FragCoord.z < -gridMapLayerHeight / 2 || gridMapLayerHeight / 2 < gl_FragCoord.z)
        discard;

    vec3 color = texture2D(colorTexture, colorUV).xyz;

    fragmentColor = color * gridMapAccumulationWeight;
}