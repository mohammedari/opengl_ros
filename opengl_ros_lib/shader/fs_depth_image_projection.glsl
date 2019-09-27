#version 450 core
uniform sampler2D colorTexture; 

uniform float gridMapLayerHeight;
uniform float gridMapAccumulationWeight;

in vec2 colorUV; //The UV coordinate of corresponding pixel in the color image 
                 //x = 0 to 1
                 //y = 0 to 1

in float height; //The height of the point, which is 0 if the point is at the middle of the image.

out vec3 fragmentColor;

void main(void)
{
    //If the height is out of the range, discard the fragment.
    //if (height < -gridMapLayerHeight / 2 || gridMapLayerHeight / 2 < height)
    //    discard;

    vec3 color = texture2D(colorTexture, colorUV).xyz;

    //fragmentColor = color * gridMapAccumulationWeight;

    //test
    fragmentColor = color;
}