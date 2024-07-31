cmake_minimum_required(VERSION 3.9)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

project(SimpleRasterizer)

set( CMAKE_CXX_FLAGS "-std=c++17" )

set(CMAKE_BUILD_TYPE Release)

add_executable(SimpleRasterizer src/Application.cpp src/Bound3.hpp src/Intersection.hpp src/Light.hpp 
                src/Material.hpp src/Ray.hpp src/Scene.hpp src/Sphere.hpp src/Surface.hpp src/Triangle.hpp
                src/Renderer.hpp src/Transform.hpp)