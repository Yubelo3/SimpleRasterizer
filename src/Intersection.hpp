#pragma once
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "Surface.hpp"
#include <memory>

struct Intersection
{
    friend std::ostream &operator<<(std::ostream &os, const Intersection &intersection)
    {
        if (!intersection.happened)
        {
            os << "No intersection" << std::endl;
            return os;
        }
        std::cout << "Intersection happend!" << std::endl;
        std::cout << "coords: " << intersection.coords.transpose() << std::endl;
        std::cout << "t: " << intersection.t << std::endl;
        return os;
    }

    // Does this intersection happened?
    bool happened = false;
    // Coords of the intersection.
    Eigen::Vector3f coords{0.0f, 0.0f, 0.0f};
    // t-param of ray-object intersection.
    float t = 0.0f;
    // If the intersection happens,which object did this ray hit?
    Surface* object;
};