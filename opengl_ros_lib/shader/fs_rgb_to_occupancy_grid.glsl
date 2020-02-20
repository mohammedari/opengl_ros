#version 450 core
uniform vec2 resolution;
uniform sampler2D texture;
uniform sampler2D accumulationTexture;
uniform int unknownDepthColor;
uniform float gridMapDecay;
uniform float gridMapResolution;
uniform mat4 mapToPreviousMap;

layout(location = 0) out float fragmentColor;
//output 0 - 100 if valid depth is measured
//output `unknownDepthColor` if valid depth is not measured
//output 255 (unknown) if ray does not reach to the pixel

layout(location = 1) out vec3 accumulationOutput;
//output for temporal accumulation

void main(void)
{
    //sample previous buffer
    vec4 previousFragCoord = mapToPreviousMap * vec4(gl_FragCoord.xy * gridMapResolution, 0, 1) / gridMapResolution;
    vec2 previousUv = previousFragCoord.xy / resolution;
    vec3 previousColor = texture2D(accumulationTexture, previousUv).xyz;

    //sample current buffer and accumulate
    vec2 uv = gl_FragCoord.xy / resolution;
    vec3 color = texture2D(texture, uv).xyz + gridMapDecay * previousColor;
    //x ... accumulated probability of object existance
    //y ... non-zero if valid depth is measured in any ray
    //z ... non-zero if any ray achieved at the pixel 

    //output for temporal acccumulation
    accumulationOutput = color;

    //output occupancy grid
    if (color.z == 0)
    {
        fragmentColor = 1.0;
        return;
    }
    if (color.y == 0) {
        fragmentColor = unknownDepthColor / 255.0;
        return;
    }
    fragmentColor = clamp(color.x, 0, 1) * 100.0 / 255.0; //rescale to 0~100
}
