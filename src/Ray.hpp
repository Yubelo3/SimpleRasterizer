#pragma once
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

//Ray described as p = orig + t * dir
struct Ray
{
    Ray(const Eigen::Vector3f &_orig, const Eigen::Vector3f &_dir) : orig(_orig), dir(_dir.normalized())
    {
        // std::cout << "Ray COPY" << std::endl;
    }
    Ray(Eigen::Vector3f &&_orig, Eigen::Vector3f &&_dir) : orig(_orig), dir(_dir.normalized())
    {
        // std::cout << "Ray MOVE" << std::endl;
    }
    Ray(){};
    //Origin of this ray.
    Eigen::Vector3f orig{0.0f, 0.0f, 0.0f};
    //Direction of the ray.
    Eigen::Vector3f dir{0.0f,0.0f,-1.0f};

    Eigen::Vector3f operator()(float t) const
    {
        return orig + t * dir;
    }
};