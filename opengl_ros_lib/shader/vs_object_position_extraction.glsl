#version 450 core
uniform vec2 depthSize;
uniform float depthUnit;
uniform usampler2D depthTexture; //using unsigned integer sampler
uniform vec2 depthFocalLength;
uniform vec2 depthCenter;

uniform vec2 colorSize;
uniform vec2 colorFocalLength;
uniform vec2 colorCenter;
uniform mat4 depthToColor;

in vec3 inputPixel;     //This input position is integer pixel coordinate in the optical frame.
                        //x = 0 to (depthSize.x - 1)
                        //y = 0 to (depthSize.y - 1)
                        //z = 0
    
out vec3 positionInMeter; //3D position of the point in optical frame
out float depthInMeter;   //The depth value in meter of the point.
    
out vec2 colorUV;       //The UV coordinate of corresponding pixel in the color image.
                        //x = 0 to 1
                        //y = 0 to 1

void main(void)
{
    //Sampling depth from the texture
    //XY coordinate and texture UV mathes in OpenGL 
    ivec2 depthUV = ivec2(
        input_pixel.x,
        input_pixel.y
    );
    float depth = texelFetch(depthTexture, depthUV, 0).x * depthUnit; //convert to meter scale

    //Now, convert image pixel (ix, iy, iz) to point in the optical frame (px, py, pz).
    //First, calculate px/pz and px/pz from ix/iz and iy/iz using projection matrix. 
    //[ix]   [fx  0  cx][px]
    //[iy] = [ 0 fy  cy][py]
    //[iz]   [ 0  0   1][pz]
    vec2  pxy_z  = (input_pixel.xy - depthCenter.xy) / depthFocalLength.xy;
    float pz = depth;
    vec4  point  = vec4(pxy_z * pz, pz, 1.0);

    //Ouptut vertex coordinate
    //Just normalizing the pixel coordinate in the depth image
    gl_Position = vec4(
        (input_pixel.x - depthCenter.x) / depthSize.x * 2, //convert to -1 to 1
        (input_pixel.y - depthCenter.y) / depthSize.y * 2, //set the origin at the center
        0.0, 
        1.0
    );
    positionInMeter = point;
    depthInMeter = depth;

    //Calculate coordinate in color image
    vec4 colorPoint = depthToColor * vec4(point, 1);
    mat3 colorProjection = mat3(
        colorFocalLength.x,                  0, 0, 
                         0, colorFocalLength.y, 0, 
             colorCenter.x,      colorCenter.y, 1
    ); //column major order
    vec3 colorImagePoint = colorProjection * colorPoint.xyz;

    //Output texture coordinate
    colorUV = vec2(
        colorImagePoint.x / colorImagePoint.z / colorSize.x, 
        colorImagePoint.y / colorImagePoint.z / colorSize.y
    );
}
