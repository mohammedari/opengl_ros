#include "color_extraction_cpu_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_extraction_cpu");

    ColorExtractionCpuNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
