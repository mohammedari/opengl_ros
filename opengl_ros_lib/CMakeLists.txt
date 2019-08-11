cmake_minimum_required(VERSION 3.0.0)
project(opengl-ros)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")

find_package(OpenCV REQUIRED)
add_subdirectory(thirdparty/glad)

include_directories(include)

add_library(opengl-ros-lib src/simple_renderer.cpp)
target_link_libraries(opengl-ros-lib GL EGL glad dl ${OpenCV_LIBS})
target_include_directories(opengl-ros-lib PUBLIC include/)

add_executable(opengl-ros src/main.cpp)
target_link_libraries(opengl-ros opengl-ros-lib)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
