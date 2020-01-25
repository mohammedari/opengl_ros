#version 450 core
uniform vec2 depthSize;
uniform float depthUnit;
uniform usampler2D depthTexture; //using unsigned integer sampler
uniform vec2 depthFocalLength;
uniform vec2 depthCenter;
uniform vec2 validDepthInMeter;

//TODO Remove entire color image process as color image is not used anymore
uniform vec2 colorSize;
uniform vec2 colorFocalLength;
uniform vec2 colorCenter;
uniform mat4 depthToColor;
//TODO Remove entire color image process as color image is not used anymore

uniform mat4 depthToMap;

uniform vec2 gridMapSize;
uniform float gridMapResolution;
uniform float gridMapLayerHeight;

in vec3 input_pixel; //This input position is integer pixel coordinate in the optical frame.
                     //x = 0 to (depthSize.x - 1)
                     //y = 0 to (depthSize.y - 1)
                     //z = 0

out vertex
{
    vec4 position;   //The vertex coordinate of the point in grid map space.
                     //x = -1 to 1; which represents left to right
                     //y = -1 to 1; which represents far to near, note this is flipped upside down
                     //z = -1 to 1; which represents low to high, 
                     //w = 1

    float depth;     //The depth value in meter of the point.
} output_vertex;

void main(void)
{
    //Sampling depth from the texture
    //XY coordinate and texture UV mathes in OpenGL 
    ivec2 depthUV = ivec2(
        input_pixel.x,
        input_pixel.y
    );
    float depth = texelFetch(depthTexture, depthUV, 0).x * depthUnit; //convert to meter scale

    if (depth < validDepthInMeter.x || validDepthInMeter.y < depth)
    {
        depth = validDepthInMeter.y; //set to the max value
    }

    //Now, convert image pixel (ix, iy, iz) to point in the optical frame (px, py, pz).
    //First, calculate px/pz and px/pz from ix/iz and iy/iz using projection matrix. 
    //[ix]   [fx  0  cx][px]
    //[iy] = [ 0 fy  cy][py]
    //[iz]   [ 0  0   1][pz]
    vec2  pxy_z  = (input_pixel.xy - depthCenter.xy) / depthFocalLength.xy;
    float pz = depth;
    vec4  point  = vec4(pxy_z * pz, pz, 1.0);

    //Transform the point along with camera pose
    vec4 p = depthToMap * point;

    //Ouptut vertex coordinate
    //Projecting the point to the occupancy grid plane, in top down orthognal projection
    output_vertex.position = vec4(
        p.x / gridMapResolution / gridMapSize.x * 2,     //convert to -1 to 1
        p.y / gridMapResolution / gridMapSize.y * 2,     //set the origin on the bottom and flip upside down
        p.z / (gridMapLayerHeight / 2),                  //mapping -height/2 to height/2, to -1 to 1
        1.0
    );
    output_vertex.depth   = depth;
}
