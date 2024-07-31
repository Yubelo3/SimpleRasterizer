#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>


// 2D transformation
class Transform2D
{
private:
    Transform2D();

    // Return the next <offset> axis.
    template <int Axis>
    static inline int _axisOffset(int offset)
    {
        return (Axis + offset) % 2;
    }

public:
    // 2D scale
    static Eigen::Matrix3f scale(float sx, float sy)
    {
        Eigen::Matrix3f ret;
        ret.setZero();
        ret(0, 0) = sx;
        ret(1, 1) = sy;
        ret(2, 2) = 1.0f;
        return ret;
    }
    // 2D rotate
    static Eigen::Matrix3f rotate(float theta)
    {
        Eigen::Matrix3f ret;
        ret << cosf(theta), -sinf(theta), 0.0f,
            sinf(theta), cosf(theta), 0.0f,
            0.0f, 0.0f, 1.0f;
        return ret;
    }
    // 2D shear, Axis: 0->x, 1->y
    template <int Axis>
    static Eigen::Matrix3f shear(float d)
    {
        Eigen::Matrix3f ret = Eigen::Matrix3f::Identity();
        int (*axisOff)(int) = _axisOffset<Axis>;
        ret(Axis, axisOff(1)) = d;
        return ret;
    }
    // 2D reflect
    template <int Axis>
    static Eigen::Matrix3f reflect()
    {
        Eigen::Matrix3f ret = Eigen::Matrix3f::Identity();
        ret(Axis, Axis) = -1.0f;
        return ret;
    }
    // 2D translate
    static Eigen::Matrix3f translate(float tx, float ty)
    {
        Eigen::Matrix3f ret = Eigen::Matrix3f::Identity();
        ret(0, 2) = tx;
        ret(1, 2) = ty;
        return ret;
    }
    // 2D coordinate transformation
    static Eigen::Matrix3f frameToCanonical(Eigen::Vector3f u, Eigen::Vector3f v, Eigen::Vector3f e)
    {
        Eigen::Matrix3f ret;
        ret << u, v, e;
        return ret;
    }
};

// 3D transformation
class Transform3D
{
private:
    Transform3D();

    // Return the next <offset> axis.
    template <int Axis>
    static inline int _axisOffset(int offset)
    {
        return (Axis + offset) % 3;
    }

    static inline void _normalizePoint(Eigen::Vector4f& point)
    {
        for(int i=0;i<4;i++)
            point(i)/=point(3);
    }

public:
    // 3D scale
    static Eigen::Matrix4f scale(float sx, float sy, float sz)
    {
        Eigen::Matrix4f ret;
        ret.setZero();
        ret(0, 0) = sx;
        ret(1, 1) = sy;
        ret(2, 2) = sz;
        ret(3, 3) = 1.0f;
        return ret;
    }
    // 3D rotate, Axis: 0->x, 1->y, 2->z
    template <int Axis>
    static Eigen::Matrix4f rotate(float theta)
    {
        Eigen::Matrix4f ret;
        ret.setZero();
        int (*axisOff)(int) = _axisOffset<Axis>;
        ret(axisOff(1), axisOff(1)) = cosf(theta);
        ret(axisOff(1), axisOff(2)) = -sinf(theta);
        ret(axisOff(2), axisOff(1)) = sinf(theta);
        ret(axisOff(2), axisOff(2)) = cosf(theta);
        ret(Axis,Axis)=1.0f;
        ret(3, 3) = 1.0f;
        return ret;
    }
    // 3D shear, Axis: 0->x, 1->y, 2->z
    template <int Axis>
    static Eigen::Matrix4f shear(float d1, float d2)
    {
        Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();
        int (*axisOff)(int) = _axisOffset<Axis>;
        ret(Axis, axisOff(1)) = d1;
        ret(Axis, axisOff(2)) = d2;
        return ret;
    }
    // 3D reflect
    template <int Axis>
    static Eigen::Matrix4f reflect()
    {
        Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();
        ret(Axis, Axis) = -1.0f;
        return ret;
    }
    // 3D translate
    static Eigen::Matrix4f translate(float tx, float ty, float tz)
    {
        Eigen::Matrix4f ret = Eigen::Matrix4f::Identity();
        ret(0, 3) = tx;
        ret(1, 3) = ty;
        ret(2, 3) = tz;
        return ret;
    }
    // 3D coordinate system transformation
    static Eigen::Matrix4f frameToCanonical(Eigen::Vector4f u, Eigen::Vector4f v, Eigen::Vector4f w, Eigen::Vector4f e)
    {
        Eigen::Matrix4f ret;
        ret << u, v, w, e;
        return ret;
    }

    // 3D viewport transformation. Canonical view volume [-1,1]^3 to the screen.
    // The screen is [-0.5,nx-0.5]x[-0.5,ny-0.5]
    static Eigen::Matrix4f Mvp(int nx, int ny)
    {
        Eigen::Matrix4f ret;
        ret << nx / 2.0f, 0.0f, 0.0f, (nx - 1.0f) / 2.0f,
            0.0f, ny / 2.0f, 0.0f, (ny - 1.0f) / 2.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
#ifdef DEBUG
        std::cout<<"Viewport Transformation"<<std::endl;
        std::cout<<ret<<std::endl;
        std::cout<<"----------------------------------"<<std::endl;
#endif
        return ret;
    }
    //3D orthographic projection. Arbitrary box volumne to canonical view volume.
    static Eigen::Matrix4f Mortho(float r,float l,float t,float b,float n,float f)
    {
        Eigen::Matrix4f ret;
        ret<<2.0f/(r-l),0.0f,0.0f,-(r+l)/(r-l),
             0.0f,2.0f/(t-b),0.0f,-(t+b)/(t-b),
             0.0f,0.0f,2.0f/(n-f),-(n+f)/(n-f),
             0.0f,0.0f,0.0f,1.0f;
#ifdef DEBUG
        std::cout<<"Orthographic Transformation"<<std::endl;
        std::cout<<ret<<std::endl;
        std::cout<<"----------------------------------"<<std::endl;
#endif
        return ret;
    }
    //3D viewing transformation. Global coordinates to camera coordinates.
    //Please ensure e is in canonical form!(e(4)==1.0f)
    static Eigen::Matrix4f Mview(const Eigen::Vector3f& e, Eigen::Vector3f& u,const Eigen::Vector3f& v,const Eigen::Vector3f& w)
    {
        Eigen::Matrix4f ret;
        ret<<u,v,w,e,
             0.0f,0.0f,0.0f,1.0f;
#ifdef DEBUG
        std::cout<<"camera Transformation"<<std::endl;
        std::cout<<ret<<std::endl;
        std::cout<<"----------------------------------"<<std::endl;
#endif
        return ret.inverse();
    }

    //3D perspective tranform. Viewing frustum to box volumn.
    static Eigen::Matrix4f Mpersp(float n,float f)
    {
        Eigen::Matrix4f ret;
        ret<<n,0.0f,0.0f,0.0f,
             0.0f,n,0.0f,0.0f,
             0.0f,0.0f,n+f,-n*f,
             0.0f,0.0f,1.0f,0.0f;
#ifdef DEBUG
        std::cout<<"Perspective to orthograpic Transformation"<<std::endl;
        std::cout<<ret<<std::endl;
        std::cout<<"----------------------------------"<<std::endl;
#endif
        return ret;
    }

};