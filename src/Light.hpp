#pragma once
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class Light
{
private:
    //Position of current light source.
    Eigen::Vector3f _position;
    //Intensity of current light source.
    Eigen::Vector3f _intensity;

public:
    Light(const Eigen::Vector3f &p, const Eigen::Vector3f &i) : _position(p), _intensity(i){};

    inline const Eigen::Vector3f &position() const
    {
        return _position;
    }
    inline const Eigen::Vector3f &intensity() const
    {
        return _intensity;
    }
};