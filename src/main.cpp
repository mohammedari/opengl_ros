
#include <array>

#include <glad/glad.h>
#include <opencv2/opencv.hpp>

#include "egl/egl.h"
#include "gl/gl.h"

struct Vertex
{
    float x, y, z;
};

int main()
{
    //Context

    cgs::egl::Display display;
    cgs::egl::Context context(display);

    if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(eglGetProcAddress)))
        std::runtime_error("Failed to load OpenGL.");

    std::cout
        << "GL_VENDOR : "                  << glGetString(GL_VENDOR)                   << std::endl
        << "GL_RENDERER : "                << glGetString(GL_RENDERER)                 << std::endl
        << "GL_VERSION : "                 << glGetString(GL_VERSION)                  << std::endl
        << "GL_SHADER_LANGUAGE_VERSION : " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

    glDebugMessageCallback([](
        GLenum source, 
        GLenum type, 
        GLuint id, 
        GLenum severity, 
        GLsizei length, 
        const GLchar* message, 
        const void* userParam
    ) {
        std::cerr 
            << cgs::gl::DebugMessageUtil::source(source)     << ", " 
            << cgs::gl::DebugMessageUtil::type(type)         << ", " 
            << cgs::gl::DebugMessageUtil::severity(severity) << ", " 
            << id << ": " 
            << message << std::endl;
    }, nullptr);
    glEnable(GL_DEBUG_OUTPUT);

    constexpr auto WIDTH = 200;
    constexpr auto HEIGHT = 200;

    //Shader

    std::array<cgs::gl::Shader, 2> shaders = {
        cgs::gl::Shader(GL_VERTEX_SHADER,   "shader/vs_null.glsl"),
        cgs::gl::Shader(GL_FRAGMENT_SHADER, "shader/fs_null.glsl"),
    };
    cgs::gl::Program program(shaders);

    program.use();
    glUniform2f(glGetUniformLocation(program.get(), "resolution"), WIDTH, HEIGHT);
    glUniform1i(glGetUniformLocation(program.get(), "texture"), 0);

    //Vertex
    std::array<Vertex, 4> verticies = {
        -0.5f, -0.5f, 0.0f,
         0.5f, -0.5f, 0.0f,
         0.5f,  0.5f, 0.0f,
        -0.5f,  0.5f, 0.0f,
    };
    cgs::gl::VertexBuffer<Vertex> vbo(verticies, GL_STATIC_DRAW);

    std::array<uint, 6> indicies = {
        0, 1, 2, 
        2, 3, 0,
    };
    cgs::gl::ElementBuffer<uint> ebo(indicies, GL_STATIC_DRAW);

    cgs::gl::VertexArray vao;
    vao.mapVariable(vbo, glGetAttribLocation(program.get(), "position"), 3, GL_FLOAT, 0);
    vao.mapVariable(ebo);

    //Texture

    auto lobster = cv::imread("lobster.png");
    cgs::gl::Texture2D textureIn(GL_RGB8, lobster.cols, lobster.rows);
    textureIn.write(GL_BGR, GL_UNSIGNED_BYTE, lobster.data);

    cgs::gl::Sampler sampler(GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE);

    //FrameBuffer

    cgs::gl::Texture2D textureOut(GL_RGB8, WIDTH, HEIGHT);
    cgs::gl::FrameBuffer fbo(textureOut);

    //Render

    fbo.bind();
    glViewport(0, 0, WIDTH, HEIGHT);
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    textureIn.bindToUnit(0);
    sampler.bindToUnit(0);

    vao.bind();
    glDrawElements(GL_TRIANGLES, indicies.size(), GL_UNSIGNED_INT, nullptr);

    glFinish();

    //Save

    cv::Mat image(HEIGHT, WIDTH, CV_8UC3);
    textureOut.read(GL_BGR, GL_UNSIGNED_BYTE, image.data, image.rows * image.cols * image.channels());

    cv::imwrite("output.png", image);

    return 0;
}