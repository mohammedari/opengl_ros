#version 450 core
uniform vec2 resolution;
uniform sampler2D texture; //Assuming sRGB texture; converted to linear RGB by GPU automatically

uniform float threshold_l;
uniform float svm_coef_a;
uniform float svm_coef_b;
uniform float svm_intercept;

in vec4 gl_FragCoord;
out vec4 fragColor;

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
    float u = gl_FragCoord.x / resolution.x;
    float v = gl_FragCoord.y / resolution.y;
    vec3 color = texture2D(texture, vec2(u, v)).xyz;

    //Convert to CIE La*b* space
    vec3 lab = xyz2lab(rgb2xyz(color));

    float l = lab.x * 255.0;
    float a = lab.y * 255.0;
    float b = lab.z * 255.0;

    //Apply SVM to extract region
    if (l > threshold_l && svm_coef_a * a + svm_coef_b * b + svm_intercept > 0)
    {
        fragColor = vec4(color, 1.0);
        return;
    }

    //Return black if the pixel is not target
    fragColor = vec4(0.0, 0.0, 0.0, 1.0);
}