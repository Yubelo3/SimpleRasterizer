#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Define surface coefficients. It is not necessary for every object.
struct Material
{
    Material(){};
    Material(const Eigen::Vector3f &_kd, const Eigen::Vector3f &_ks, const Eigen::Vector3f &_ka) : kd(_kd), ks(_ks), ka(_ka){};

    Eigen::Vector3f kd{0.3f, 0.3f, 0.3f};
    Eigen::Vector3f ks{0.4f, 0.4f, 0.2f};
    Eigen::Vector3f ka{0.075f, 0.075f, 0.075f};
    float n = 100.0f;
};