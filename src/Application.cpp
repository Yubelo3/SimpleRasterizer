#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "Ray.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Scene.hpp"
#include "Renderer.hpp"
#include "Transform.hpp"
#include "Intersection.hpp"
#include "OBJ_Loader.h"

const float PI = 3.1415926f;
float *zDepth = nullptr;

static void rasterize2DLine(Eigen::Vector2i p0, Eigen::Vector2i p1, const Scene &scene)
{
    int x0 = p0[0], y0 = p0[1], x1 = p1[0], y1 = p1[1];
    float k = ((float)y1 - y0) / ((float)x1 - x0);
    float d = (y0 - y1) * (x0 + 1.0f) + (x1 - x0) * (y0 + 0.5f) + x0 * y1 - x1 * y0;
    int y = y0;
    for (int x = x0; x <= x1; x++)
    {
        scene.draw({x, y}, {0.0f, 0.0f, 1.0f});
        if (d < 0.0f) //线在中点上方，画上侧
        {
            y++;
            d += (y0 - y1) + (x1 - x0);
        }
        else
            d += (y0 - y1);
    }
}

static void rasterize2DTriangle(Eigen::Vector2f v0, Eigen::Vector2f v1, Eigen::Vector2f v2, const Scene &scene)
{
    // Get the bounding box of current triangle.
    auto [minX, minY, maxX, maxY] = [&v0, &v1, &v2]() -> std::tuple<int, int, int, int>
    {
        float minXY[2]{v0[0], v0[1]}, maxXY[2]{v0[0], v0[1]};
        for (const Eigen::Vector2f &v : {v1, v2})
            for (int i : {0, 1})
            {
                if (v[i] < minXY[i])
                    minXY[i] = v[i];
                if (v[i] > maxXY[i])
                    maxXY[i] = v[i];
            }
        return {(int)floor(minXY[0]), (int)floor(minXY[1]), (int)ceil(maxXY[0]), (int)ceil(maxXY[1])};
    }();
    // Compute barycentric coordinates for every location(x,y) in bounding box.
    // Compute first pixel(minX,minY).
    float A[3]{v1[1] - v2[1], v2[1] - v0[1], v0[1] - v1[1]};
    float B[3]{v2[0] - v1[0], v0[0] - v2[0], v1[0] - v0[0]};
    float C[3]{v1[0] * v2[1] - v2[0] * v1[1], v2[0] * v0[1] - v0[0] * v2[1], v0[0] * v1[1] - v1[0] * v0[1]};
    float f[3]{A[0] * v0[0] + B[0] * v0[1] + C[0], A[1] * v1[0] + B[1] * v1[1] + C[1], A[2] * v2[0] + B[2] * v2[1] + C[2]};
    auto [alpha00, beta00, gamma00] = [&](int x, int y) -> std::tuple<float, float, float>
    {
        float xyz[3];
        for (int i : {0, 1, 2})
            xyz[i] = (A[i] * x + B[i] * y + C[i]) / f[i];
        return {xyz[0], xyz[1], xyz[2]};
    }(minX, minY);
    // fi(x+1,y)=f(x,y)+A[i], fi(x,y+1)=f(x,y)+B[i]
    float incrementX[3], incrementY[3];
    for (int i = 0; i < 3; i++)
    {
        incrementX[i] = A[i] / f[i];
        incrementY[i] = B[i] / f[i];
    }
    // Compute barycentric coords for (x,y)
    for (int y = minY; y <= maxY; y++)
    {
        float alpha = (y - minY) * incrementY[0] + alpha00, beta = (y - minY) * incrementY[1] + beta00, gamma = (y - minY) * incrementY[2] + gamma00;
        for (int x = minX; x <= maxX; x++)
        {
            if (alpha > 0 && beta > 0 && gamma > 0)
                // scene.draw({x, y}, {0.0f, 0.5f, 0.0f});
                alpha += incrementX[0];
            beta += incrementX[1];
            gamma += incrementX[2];
        }
    }
    return;
}

static void rasterizeAxes(const Scene &scene)
{
    for (int x = scene.width / 2; x <= scene.width; x += 1.0f)
        scene.draw({x, scene.height / 2}, {1.0f, 0.0f, 0.0f});
    for (int y = scene.height / 2; y <= scene.height; y += 1.0f)
        scene.draw({scene.width / 2, y}, {0.0f, 1.0f, 0.0});
}

// Output specification
const int image_width = 640;
const int image_height = 360;

struct Camera
{
private:
    Eigen::Vector3f pos{0.0f, 100.0f, 0.0f};
    Eigen::Vector3f look_at{0.0f, 0.0f, -1.0f};
    Eigen::Vector3f up{0.0f, 1.0f, 0.0f};

public:
    Eigen::Vector3f u, v, w, e;
    Camera()
    {
        v = up.normalized();
        w = -look_at.normalized();
        u = v.cross(w).normalized();
        e = pos;
    }
};

Eigen::Vector3f convert(objl::Vector3 vec)
{
    return {vec.X, vec.Y, vec.Z};
}

int main()
{
    zDepth = new float[1920 * 1080 + 1];
    for (int i = 0; i < 1920 * 1080; i++)
        zDepth[i] = -10000.0f;
    // Ray ray({0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f});
    // Triangle t{{-0.5f, -0.5f, 0.0f}, {0.5f, -0.5f, 0.0f}, {0.0f, 0.5f, 0.0f}};
    // Scene scene;
    // scene.add(&t);
    // Intersection inter;
    // scene.hit(ray, 0.0001f, std::numeric_limits<float>::infinity(), inter);
    // std::cout << inter << std::endl;

    Scene scene;

    std::vector<Eigen::Vector3f> normals;

    objl::Loader loader;
    bool loaded = loader.LoadFile("../teapot/teapot.obj");
    // loader.LoadFile("../bottle/bottle_model.obj");
    if (!loaded)
    {
        std::cout << "Failed to open file" << std::endl;
        return -1;
    }
    for (objl::Mesh &mesh : loader.LoadedMeshes)
    {
        for (int i = 0; i < mesh.Indices.size(); i += 3)
        {
            // assert(mesh.Indices[i]<mesh.Vertices.size()&&mesh.Indices[i]>=0);
            // assert(mesh.Indices[i+1]<mesh.Vertices.size()&&mesh.Indices[i+1]>=0);
            // assert(mesh.Indices[i]+2<mesh.Vertices.size()&&mesh.Indices[i+2]>=0);

            objl::Vector3 &v0 = mesh.Vertices[mesh.Indices[i]].Position;
            objl::Vector3 &v1 = mesh.Vertices[mesh.Indices[i + 1]].Position;
            objl::Vector3 &v2 = mesh.Vertices[mesh.Indices[i + 2]].Position;
            // std::cout<<"Triangle: "<<std::endl;
            // std::cout<<v0.X<<" "<<v0.Y<<" "<<v0.Z<<std::endl;
            // std::cout<<v1.X<<" "<<v1.Y<<" "<<v1.Z<<std::endl;
            // std::cout<<v2.X<<" "<<v2.Y<<" "<<v2.Z<<std::endl;
            Triangle *t = new Triangle{{v0.X, v0.Y, v0.Z},
                                       {v1.X, v1.Y, v1.Z},
                                       {v2.X, v2.Y, v2.Z}};
            normals.push_back(-convert(mesh.Vertices[mesh.Indices[i]].Normal).normalized());
            normals.push_back(-convert(mesh.Vertices[mesh.Indices[i + 1]].Normal).normalized());
            normals.push_back(-convert(mesh.Vertices[mesh.Indices[i + 2]].Normal).normalized());
            scene.add(t);
        }
    }
    // for(int i=0;i<mesh.Indices.size();i+=3)
    // {
    //     std::cout<<"idx: "<<mesh.Indices[i]<<" "<<mesh.Indices[i+1]<<" "<<mesh.Indices[i+2]<<std::endl;
    //     // for(int j=0;j<3;j++)
    //     // {

    //     //     objl::Vector3& pos=mesh.Vertices[i+j].Position;
    //     //     std::cout<<pos.X<<' '<<pos.Y<<' '<<pos.Z<<std::endl;
    //     // }

    // }
    // std::cout<<mesh.Vertices.size()<<std::endl;
    // for(int i=0;i<mesh.Vertices.size();i++)
    //     std::cout<<mesh.Vertices[i].Position.X<<' '<<mesh.Vertices[i].Position.Y<<' '<<mesh.Vertices[i].Position.Z<<std::endl;
    // }

    // Objects
    Triangle *t1{new Triangle{{-400.0f, 200.0f, -100.0f},
                              {400.0f, -300.0f, -100.0f},
                              {0.0f, 300.0f, -150.0f}}};
    Triangle *t2{new Triangle{{-400.0f, 200.0f, -100.0f},
                              {0.0f, -200.0f, -100.0f},
                              {400.0f, -300.0f, -100.0f}}};

    // Lights
    Light *l1{new Light{{0.0f, 200.0f, 0.0f}, {150.0f, 150.0f, 100.0f}}};
    Light *l2{new Light{{0.0f, 0.0f, -10.0f}, {0.0f, 10.0f, 6.0f}}};

    // scene.add(t1);
    // scene.add(s1);
    scene.add(l1);
    // scene.add(l2);
    // scene.add(t1);
    // scene.add(t2);

    Camera camera;
    // Build transfrom matrices
    float fovMul = tanf(65 / 2 * PI / 180);
    float dis = 180.0f;
    float upperBound = dis * fovMul;
    float rightBound = upperBound * 16.0f / 9.0f;
    Eigen::Matrix4f Mview = Transform3D::Mview(camera.e, camera.u, camera.v, camera.w);
    Eigen::Matrix4f Mpersp = Transform3D::Mortho(rightBound, -rightBound, upperBound, -upperBound, -dis, -1000.0f) * Transform3D::Mpersp(-dis, -1000.0f);
    Eigen::Matrix4f Mvp = Transform3D::Mvp(scene.width, scene.height);

    Eigen::Matrix4f Mtransform = Mvp * Mpersp * Mview;
    Eigen::Matrix4f MinverseTransform = Mtransform.inverse();

    Renderer r(scene.height, scene.width);

    int curVertex = 0;

    for (Triangle *object : scene._objects)
    {
        auto [v0, v1, v2] = object->getVertices();

        object->transform(Transform3D::rotate<0>(0.5f));
        object->transform(Transform3D::rotate<2>(0.2f));
        object->transform(Transform3D::scale(80.0f, 80.0f, 80.0f));
        object->transform(Transform3D::translate(0.0f, 0.0f, -450.0f));

        // object->transform(Transform3D::rotate<0>(PI/5));

        // std::cout<<"Before Transform"<<std::endl;
        // std::cout<<"v0: "<<v0.transpose()<<std::endl;
        // std::cout<<"v1: "<<v1.transpose()<<std::endl;
        // std::cout<<"v2: "<<v2.transpose()<<std::endl;
        // std::cout<<"--------------------------------------"<<std::endl;

        Eigen::Vector3f colors[3];
        Ray rays[3];
        Intersection inters[3];
        for (int i = 0; i < 3; i++)
        {
            rays[i].orig = camera.e;
            rays[i].dir = ((*object)[i] - camera.e).normalized();
            inters[i].happened = true;
            inters[i].coords = (*object)[i];
            inters[i].object = object;
            colors[i] = scene.shade(inters[i], rays[i], normals[curVertex]);
            curVertex++;
        }

        // std::cout<<"Transfrom: "<<std::endl;
        // std::cout<<Mtransform<<std::endl;

        // std::cout<<"transfrom"<<std::endl;
        // std::cout<<Mtransform<<std::endl;
        // std::cout<<"-------------------------------------"<<std::endl;

        object->transform(Mtransform);
        // std::cout<<"After transform"<<std::endl;
        // std::cout<<"v0: "<<v0.transpose()<<std::endl;
        // std::cout<<"v1: "<<v1.transpose()<<std::endl;
        // std::cout<<"v2: "<<v2.transpose()<<std::endl;
        // std::cout<<"-----------------------------"<<std::endl;
        // Get the bounding box of current triangle.

        auto [minX, minY, maxX, maxY] = [](Eigen::Vector3f &v0, Eigen::Vector3f &v1, Eigen::Vector3f &v2) -> std::tuple<int, int, int, int>
        {
            float minXY[2]{v0[0], v0[1]}, maxXY[2]{v0[0], v0[1]};
            for (const Eigen::Vector3f &v : {v1, v2})
                for (int i : {0, 1})
                {
                    if (v[i] < minXY[i])
                        minXY[i] = v[i];
                    if (v[i] > maxXY[i])
                        maxXY[i] = v[i];
                }
            return {(int)floor(minXY[0]), (int)floor(minXY[1]), (int)ceil(maxXY[0]), (int)ceil(maxXY[1])};
        }(v0, v1, v2);
        // Compute barycentric coordinates for every location(x,y) in bounding box.
        // Compute first pixel(minX,minY).
        float A[3]{v1[1] - v2[1], v2[1] - v0[1], v0[1] - v1[1]};
        float B[3]{v2[0] - v1[0], v0[0] - v2[0], v1[0] - v0[0]};
        float C[3]{v1[0] * v2[1] - v2[0] * v1[1], v2[0] * v0[1] - v0[0] * v2[1], v0[0] * v1[1] - v1[0] * v0[1]};
        float f[3]{A[0] * v0[0] + B[0] * v0[1] + C[0], A[1] * v1[0] + B[1] * v1[1] + C[1], A[2] * v2[0] + B[2] * v2[1] + C[2]};

        auto [alpha00, beta00, gamma00] = [&](int x, int y) -> std::tuple<float, float, float>
        {
            float xyz[3];
            for (int i : {0, 1, 2})
                xyz[i] = (A[i] * x + B[i] * y + C[i]) / f[i];
            return {xyz[0], xyz[1], xyz[2]};
        }(minX, minY);
        // fi(x+1,y)=f(x,y)+A[i], fi(x,y+1)=f(x,y)+B[i]
        float incrementX[3], incrementY[3];
        for (int i = 0; i < 3; i++)
        {
            incrementX[i] = A[i] / f[i];
            incrementY[i] = B[i] / f[i];
        }
        // Compute barycentric coords for (x,y)
        for (int y = minY; y <= maxY; y++)
        {
            float alpha = (y - minY) * incrementY[0] + alpha00, beta = (y - minY) * incrementY[1] + beta00, gamma = (y - minY) * incrementY[2] + gamma00;
            for (int x = minX; x <= maxX; x++)
            {
                if (alpha > 0 && beta > 0 && gamma > 0)
                {
                    if (x < scene.width && y < scene.height && x >= 0 && y >= 0)
                    {
                        float nz = alpha * v0.z() + beta * v1.z() + gamma * v2.z();

                        if (nz > zDepth[y * scene.width + x])
                        {
                            scene.draw({x, y}, alpha * colors[0] + beta * colors[1] + gamma * colors[2]);
                            zDepth[y * scene.width + x] = nz;
                        }
                    }
                }
                alpha += incrementX[0];
                beta += incrementX[1];
                gamma += incrementX[2];
            }
        }
    }

    r.render(scene.pixels);

    return 0;
}