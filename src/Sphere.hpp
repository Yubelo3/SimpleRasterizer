#pragma once
#include "Surface.hpp"
#include "Intersection.hpp"

class Sphere : public Surface
{
private:
    // Coords of the center of the sphere.
    Eigen::Vector3f _center;
    // Radius of the sphere.
    float _radius = 0.0f;
    // Bounding box.
    Bound3 _bound;

protected:
    void _getBound()
    {
        Eigen::Vector3f offset{_radius, _radius, _radius};
        _bound = {_center - offset, _center + offset};
    }

public:
    Sphere(const Eigen::Vector3f &center, const float radius) : _center(center), _radius(radius)
    {
        _getBound();
        std::cout << "Sphere COPY" << std::endl;
    }
    Sphere(Eigen::Vector3f &&center, const float radius) : _center(center), _radius(radius)
    {
        _getBound();
        std::cout << "Sphere MOVE" << std::endl;
    }

    bool hit(const Ray &ray, float t0, float t1, Intersection &intersection) override
    {
        intersection = Intersection();          // Reset intersection.
        Eigen::Vector3f x = ray.orig - _center; // x=e-c
        float d2 = ray.dir.dot(ray.dir), x2 = x.dot(x), dx = ray.dir.dot(x);
        float delta_divide_4 = dx * dx - d2 * (x2 - _radius * _radius);
        if (delta_divide_4 < 0) // No root, no intersection
            return false;
        // Else, intersection may generated. Select the nearest intersection.
        float tMin = (-ray.dir.dot(x) - sqrt(delta_divide_4)) / d2;
        float tMax = (-ray.dir.dot(x) + sqrt(delta_divide_4)) / d2;
        // If t is negative, no intersection.
        // If t is too small, regard it as the surface itself.
        if (tMax < t0 || tMin > t1)
            return false;
        if (tMin < t0)
            tMin = tMax;

        intersection = {true, ray(tMin), tMin};
        return true;
    }
    Bound3 bounding_box() const override
    {
        return _bound;
    }
    Eigen::Vector3f normal(const Eigen::Vector3f &location) const override
    {
        return (location - _center).normalized();
    }

    void transform(const Eigen::Matrix4f &transform)
    {

    }
};