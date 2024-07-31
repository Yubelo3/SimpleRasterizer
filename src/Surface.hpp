#pragma once
#include "Ray.hpp"
#include "Bound3.hpp"
#include "Material.hpp"
#include <string>
#include <memory>

struct Intersection;

// Base class of various surface, like sphere, triangle, polygon, etc.
class Surface
{
protected:
    virtual void _getBound() = 0;

public:
    // Material of this object. It is not necessary for every object if you don't need.
    Material *material = new Material;

    Surface(){};
    virtual ~Surface()
    {
        delete material;
    }
    virtual bool hit(const Ray &ray, float t0, float t1, Intersection &intersection) = 0;
    virtual Bound3 bounding_box() const = 0;
    virtual Eigen::Vector3f normal(const Eigen::Vector3f &location) const = 0;

    virtual void transform(const Eigen::Matrix4f &transform) = 0;

    // For debugging
    std::string name;
};