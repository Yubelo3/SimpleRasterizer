#pragma once
#include "Surface.hpp"
#include "Intersection.hpp"

class Triangle : public Surface
{
private:
    // Vertices
    Eigen::Vector3f _v[3];
    // Edges: e1 = v1 - v0, e2 = v2 - v0
    Eigen::Vector3f _e[2];
    // Normal vector of the triangle
    Eigen::Vector3f _normal;
    // Bounding box.
    Bound3 _bound;

    void _getBound()
    {
        Eigen::Vector3f minVert, maxVert;
        [this, &minVert, &maxVert]() -> void
        {
            for (int dim = 0; dim < 3; dim++)
            {
                float min = _v[0](dim), max = _v[0](dim);
                for (int i = 1; i < 3; i++)
                {
                    if (min > _v[i](dim))
                        min = _v[i](dim);
                    if (max < _v[i](dim))
                        max = _v[i](dim);
                }
                minVert(dim) = min;
                maxVert(dim) = max;
            }
        }();
        _bound = {minVert, maxVert};
    }

    void _computeAttribs()
    {
        _getBound();
        _e[0] = _v[0] - _v[1];
        _e[1] = _v[0] - _v[2];
        _normal = _e[0].cross(_e[1]).normalized();
    }

public:
    std::tuple<Eigen::Vector3f&,Eigen::Vector3f&,Eigen::Vector3f&> getVertices()
    {
        return {_v[0],_v[1],_v[2]};
    }


public:
    Triangle(const Eigen::Vector3f &v0, const Eigen::Vector3f &v1, const Eigen::Vector3f &v2) : _v({v0, v1, v2})
    {
        _computeAttribs();
    }
    Triangle(Eigen::Vector3f &&v0, Eigen::Vector3f &&v1, Eigen::Vector3f &&v2) : _v({v0, v1, v2})
    {
        _computeAttribs();
    }

    // Return the vertex by index.
    const Eigen::Vector3f &operator[](int index) const
    {
        assert(index >= 0 && index < 3);
        return _v[index];
    }
    // Return normal vector.
    Eigen::Vector3f normal(const Eigen::Vector3f &location) const override
    {
        return _normal;
    }

    bool hit(const Ray &ray, float t0, float t1, Intersection &intersection) override
    {
        // beta*(a-b=v0)+gamma*(a-c=v1)+t*ray.dir=a-ray.orig
        // Rule: beta>0, gamma>0, beta+gamma<1
        Eigen::Vector3f a_minus_e = _v[0] - ray.orig;
        Eigen::Matrix3f A, temp;
        A << _e[0], _e[1], ray.dir;
        float determinantA = A.determinant();

        // Compute t
        temp = A;
        // std::cout<<"Before modify"<<std::endl<<temp<<std::endl;
        temp.col(2) = a_minus_e;
        // std::cout<<"After modify"<<std::endl<<temp<<std::endl;
        float t = temp.determinant() / determinantA;
        // std::cout<<t<<std::endl;
        if (t < t0 || t > t1)
            return false;
        // Compute gamma
        temp = A;
        temp.col(1) = a_minus_e;
        float gamma = temp.determinant() / determinantA;
        if (gamma < 0 || gamma > 1)
            return false;
        // Compute beta
        temp = A;
        temp.col(0) = a_minus_e;
        float beta = temp.determinant() / determinantA;
        // std::cout<<beta<<" "<<gamma<<std::endl;
        if (beta < 0 || beta + gamma > 1)
            return false;

        // Calculate intersection.
        intersection = {true, ray(t), t};
        return true;
    }

    Bound3 bounding_box() const override
    {
        return _bound;
    }

    // Transform
    void transform(const Eigen::Matrix4f &transform)
    {
        Eigen::Matrix<float, 4, 3> temp;
        temp.setZero();
        temp << _v[0], _v[1], _v[2],
            1.0f, 1.0f, 1.0f;
        temp = transform * temp;
        for (int i = 0; i < 3; i++)
            _v[i] = temp.block<3, 1>(0, i) / temp(3, i);
        _computeAttribs();
    }
};