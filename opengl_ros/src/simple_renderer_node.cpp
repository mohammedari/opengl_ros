#include "simple_renderer_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_renderer");

    SimpleRendererNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
