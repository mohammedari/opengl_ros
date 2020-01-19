#version 450 core
uniform vec2 resolution;
uniform sampler2D texture;
uniform int unknownDepthColor;

in vec4 gl_FragCoord;
out float fragmentColor;
//output 0 - 100 if valid depth is measured
//output `unknownDepthColor` if valid depth is not measured
//output 255 (unknown) if ray does not reach to the pixel

void main(void)
{
    float u = gl_FragCoord.x / resolution.x;
    float v = gl_FragCoord.y / resolution.y;

    vec3 color = texture2D(texture, vec2(u, v)).xyz;
    //x ... accumulated probability of object existance
    //y ... 1.0 if valid depth is measured in any ray
    //z ... 1.0 if any ray achieved at the pixel 

    if (color.z < 0.5)
    {
        fragmentColor = 1.0;
        return;
    }
    
    if (color.y < 0.5) {
        fragmentColor = unknownDepthColor / 255.0;
        return;
    }

    fragmentColor = color.x * 100.0 / 255.0; //rescale to 0~100
}
