cmake_minimum_required(VERSION 3.0.0)
project(opengl-ros)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")

find_package(OpenCV REQUIRED)
add_subdirectory(thirdparty/glad)

add_executable(opengl-ros main.cpp)
target_link_libraries(opengl-ros GL EGL glad dl ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
