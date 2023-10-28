#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

enum BodyType{
    Rigid,
    Soft
};

class XPBDBody
{
public:
    XPBDBody(BodyType type) : bodyType(type){}
    virtual ~XPBDBody(){}

    virtual void setMaterial(float mu, float lambda){}

    // Simulation Update
    virtual void preSolve(float dt) = 0;
    virtual void solve(float dt) = 0;
    virtual void postSolve(float dt) = 0;
    virtual void endFrame() = 0;

    virtual void translatePos(Eigen::Vector3f delta) = 0;

    virtual float getMass() { return mass; }
    virtual Eigen::Vector3f getPosition() { return x; }
    virtual void setPosition(const Eigen::Vector3f newPos) { x = newPos; }
    virtual Eigen::Quaternionf getRotation() { return q; }
    virtual void setRotation(const Eigen::Quaternionf newRot) { q = newRot; };
    virtual Eigen::Matrix3f getInertia() { return I; };
    virtual Eigen::Matrix3f getInertiaLocal() { return Ibody; };


    BodyType bodyType;
    std::ofstream logfile;
    // static int counter;

protected:
    float mass;
    
    Eigen::Vector3f x;
    Eigen::Quaternionf q;

    Eigen::Matrix3f I;        // Inertia matrix (global)
    Eigen::Matrix3f Ibody;    // Inertia in local body frame.
};
