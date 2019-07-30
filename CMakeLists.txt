cmake_minimum_required(VERSION 3.0.0)
project(opengl-ros VERSION 0.1.0)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")

add_executable(opengl-ros main.cpp)
target_link_libraries(opengl-ros GL EGL)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
