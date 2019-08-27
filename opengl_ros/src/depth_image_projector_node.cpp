#include "depth_image_projector_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_image_projector");

    DepthImageProjectorNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
