#version 450 core
uniform float gridMapLayerHeight;
uniform float gridMapAccumulationWeight;
uniform float depthHitThreshold;
uniform vec2 validDepthInMeter;

out vec3 fragmentColor;
//x ... accumulated probability of object existance
//y ... 1.0 if valid depth is measured in any ray
//z ... 1.0 if any ray achieved at the pixel 

//TODO Remove entire color image process as color image is not used anymore
uniform sampler2D colorTexture; 
uniform float threshold_l;
uniform float svm_coef_a;
uniform float svm_coef_b;
uniform float svm_intercept;
//TODO Remove entire color image process as color image is not used anymore

in float depth;
in float hitDepth;

void main(void)
{
    //TODO multilayer output based on depth which represents height

    //If the height is out of the range, discard the fragment.
    if (gl_FragCoord.z < -gridMapLayerHeight / 2 || gridMapLayerHeight / 2 < gl_FragCoord.z)
        discard;

    float validDepthMeasured = 1.0;
    if (validDepthInMeter.y <= hitDepth)
        validDepthMeasured = 0.0;

    float accumulatedProbability = 0.0;
    if (0 < validDepthMeasured && hitDepth * depthHitThreshold < depth)
        accumulatedProbability = gridMapAccumulationWeight;

    //store accumulated value in R channel
    //store 1.0 in G channel if valid depth is measured at the line
    //always store 1.0 in B channel 
    fragmentColor = vec3(accumulatedProbability, validDepthMeasured, 1.0); 
}