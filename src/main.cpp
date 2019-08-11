#include <opencv2/opencv.hpp>

#include "simple_renderer.h"

int main()
{
    constexpr int WIDTH  = 1000;
    constexpr int HEIGHT = 1000;

    cgs::SimpleRenderer renderer(
        WIDTH, HEIGHT,
        "shader/vs_passthrough.glsl", 
        "shader/fs_passthrough.glsl");

    auto lobster = cv::imread("lobster.png");
    cv::Mat output(HEIGHT, WIDTH, CV_8UC3);

    renderer.render(output, lobster);

    cv::imwrite("output.png", output);

    return 0;
}