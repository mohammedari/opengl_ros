#include "object_position_extractor_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_position_extractor");

    ObjectPositionExtractorNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
