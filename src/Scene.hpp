#pragma once
#include <vector>
#include <memory>
#include "Surface.hpp"
#include "Light.hpp"
#include "Intersection.hpp"
#include "Triangle.hpp"

//假定film放在xy平面，看向-z方向

// Scene with objects and lights.
class Scene
{
public:
    constexpr static float epsilon = 0.00001f;
    constexpr static float gamma = 800.0f;

public:
    // Objects in current scene.
    std::vector<Triangle *> _objects;
    // Light sources in current scene.
    std::vector<Light *> _lights;

public:
    Scene()
    {
        pixels = new Eigen::Vector3f[width * height];
        for(int i=0;i<width*height;i++)
            pixels[i]=background;
    }
    ~Scene()
    {
        for (auto p : _objects)
            delete p;
        for (auto p : _lights)
            delete p;
        delete pixels;
    }

public:
    // Drawing buffer
    Eigen::Vector3f *pixels = nullptr;

    // Ambient lighting intensity.
    Eigen::Vector3f Ia{2.0f, 2.0f, 2.0f};
    // Setting up display attributes.
    int width = 1920;
    int height = 1080;
    // Actual viewport
    int w = 640;
    int h = 360;
    // Background color.
    Eigen::Vector3f background{0.0f, 0.05f, 0.05f};

    void add(Triangle *object)
    {
        _objects.push_back(object);
    }
    void add(Light *light)
    {
        _lights.push_back(light);
    }

    bool hit(const Ray &ray, float t0, float t1, Intersection &intersection)
    {
        intersection = Intersection(); // Reset intersection.
        for (auto &object : _objects)
        {
            if (object->hit(ray, t0, t1, intersection))
            {
                intersection.object = object;
                // std::cout << "hit: " << object->name << std::endl;
                break;
            }
        }
        return intersection.happened;
    }

    // Blinn-Phong Shading
    Eigen::Vector3f shade(const Intersection &inter, const Ray &vRay,const Eigen::Vector3f& normal)
    {
        // Calculate shading for every light source.
        Material *material = inter.object->material;
        const Eigen::Vector3f &coords = inter.coords;
        // const Eigen::Vector3f normal = inter.object->normal(coords);
        const Eigen::Vector3f &v = -vRay.dir;

        // 1、Ambient Lighting
        Eigen::Vector3f color{Ia.cwiseProduct(material->ka)};
        auto max = [](float a, float b) -> float
        {
            return (a > b ? a : b);
        };
        auto min = [](float a, float b) -> float
        {
            return (a < b ? a : b);
        };
        for (const auto &light : _lights)
        {
            Eigen::Vector3f l = light->position() - coords;
            float d = l.norm();
            l.normalize();
            const Eigen::Vector3f intensity = light->intensity() / (d * d);
            // std::cout << "Intensity: " << intensity.transpose() << std::endl;
            const Eigen::Vector3f h = (-vRay.dir + l).normalized();

            // 2、Diffuse Lighting
            Eigen::Vector3f diffuse = material->kd.cwiseProduct(intensity) * max(0.0f, normal.dot(l)) * gamma;
            // std::cout << "diffuse: " << diffuse.transpose() << std::endl;
            // 3、Specular Highlight
            Eigen::Vector3f specular = material->ks.cwiseProduct(intensity) * powf(max(0.0f, normal.dot(h)), material->n) * gamma;
            // std::cout << "specular: " << specular.transpose() << std::endl;

            color += (specular + diffuse);
        }

        for (int i = 0; i < 3; i++)
            color(i) = min(1.0f, color(i));

        return color;
    }

    void draw(const Eigen::Vector2i &location, const Eigen::Vector3f &color) const
    {
        int row = height - location[1], col = location[0];
        pixels[row * width + col] = color;
    }
};