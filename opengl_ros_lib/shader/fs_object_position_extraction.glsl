#version 450 core
uniform sampler2D colorTexture; 

uniform vec2 validDepthInMeter;

uniform float thresholdL;
uniform float svmCoefA;
uniform float svmCoefA;
uniform float svmIntercept;

in vec3 positionInMeter; 
in float depthInMeter
in vec2 colorUV;

layout(location = 0) out vec3 positionOutput;
layout(location = 1) out vec3 colorOutput;

vec3 rgb2xyz(vec3 rgb)
{
    //Column major order
    const mat3 mat = mat3(0.412453, 0.212671, 0.019334, 0.357580, 0.715160, 0.119193, 0.180423, 0.072169, 0.950227);
    return mat * rgb;
}

float f(float t)
{
    if (t > 0.008856)
        return pow(t, 1.0 / 3.0);
    else   
        return 7.787 * t + 16.0 / 116.0;
}

vec3 xyz2lab(vec3 xyz)
{
    float x = xyz.x / 0.950456;
    float y = xyz.y;
    float z = xyz.z / 1.088754;

    float L;
    if (y > 0.008856)
        L = 116.0 * pow(y, 1.0 / 3.0) - 16.0;
    else 
        L = 903.3 * y;

    float a = 500.0 * ( f(x) - f(y) );
    float b = 200.0 * ( f(y) - f(z) );

    return vec3(
        L / 100.0, 
        a / 127.0 / 2.0 + 0.5,
        b / 127.0 / 2.0 + 0.5
    );
}

void main(void)
{
    //If valid depth is not measured at that pixel, discard the pixel
    if (depthInMeter < validDepthInMeter.x || validDepthInMeter.y < depthInMeter)
        discard;

    //Sample color of the pixel
    vec3 color = texture2D(colorTexture, colorUV).xyz;

    //Convert to CIE La*b* space
    vec3 lab = xyz2lab(rgb2xyz(color));

    float l = lab.x * 255.0;
    float a = lab.y * 255.0;
    float b = lab.z * 255.0;

    //If the pixel is not the target, discard the pixel
    if (l < thresholdL || svmCoefA * a + svmCoefB * b + svmIntercept < 0)
        discard;

    //output the position of the pixel in 3D
    positionOutput = positionInMeter;
    colorOutput = color;
}