#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

enum BodyType{
    Rigid,
    Soft
};

class XPBDBody
{
public:
    XPBDBody(BodyType type, float Mass = 1.0f) : bodyType(type), mass(Mass){}
    virtual ~XPBDBody(){}

    virtual void setMaterial(float mu, float lambda) = 0;
    virtual void preSolve(float dt) = 0;
    virtual void solve(float dt) = 0;
    virtual void postSolve(float dt) = 0;
    virtual void endFrame() = 0;

    virtual void translatePos(Eigen::Vector3f delta){ x += delta; }

    BodyType bodyType;
    Eigen::Vector3f x;
    Eigen::Quaternionf q;
    float mass;
};
