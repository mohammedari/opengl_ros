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

uniform float gridMapResolution;
uniform vec2  gridMapSize;

in vec3 input_pixel; //This input position is integer pixel coordinate in the optical frame.
                     //x = 0 to (depthSize.x - 1)
                     //y = 0 to (depthSize.y - 1)
                     //z = 0

out vec4 position;   //The vertex coordinate of the point in grid map space.
                     //x = -1 to 1; which represents left to right
                     //y = -1 to 1; which represents far to near, flipped upside down
                     //z = 0; TODO this value should represents the height
                     //w = 0

out vec2 colorUV;    //The UV coordinate of corresponding pixel in the color image.
                     //x = 0 to 1
                     //y = 0 to 1

out float height;    //The height of the point, which is 0 if the point is at the middle of the image.
                     //TODO should be removed and use position.z instead

void main(void)
{
    //TODO set by uniform variable
    //The values in the image from D435 is not range value.
    const bool rangeValueInDepthImage = false;

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
    //Then, calculate pz using following formula.
    //px^2 + py^2 + pz^2 = depth^2
    //-> (px/pz)^2 + (py/pz)^2 + 1 = (depth/pz)^2
    //-> pz^2 = (depth)^2 / ((px/pz)^2 + (py/pz)^2 + 1)
    vec2  pxy_z  = (input_pixel.xy - depthCenter.xy) / depthFocalLength.xy;
    float pz;
    if (rangeValueInDepthImage)
        pz = sqrt(depth * depth / (length(pxy_z) + 1));
    else
        pz = depth;
    vec3  point  = vec3(pxy_z * pz, pz);

    //TODO rotate the point along with camera pose

    //Projecting the point to the occupancy grid plane
    vec2 plane = vec2(
        point.x / gridMapResolution / gridMapSize.x * 2,     //convert to -1 to 1
        1 - point.z / gridMapResolution / gridMapSize.y * 2  //set the origin on the bottom and flip upside down
    );

    //Ouptut vertex coordinate
    position = vec4(plane, 0.0, 1.0);
    height = point.y;

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