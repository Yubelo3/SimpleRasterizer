#pragma once
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class Bound3
{
    friend std::ostream &operator<<(std::ostream &os, const Bound3 &bound)
    {
        os << "pMin: " << bound.pMin.transpose() << std::endl;
        os << "pMax: " << bound.pMax.transpose() << std::endl;
        return os;
    }

public:
    Eigen::Vector3f pMin{0.0f,0.0f,0.0f};
    Eigen::Vector3f pMax{0.0f,0.0f,0.0f};
};