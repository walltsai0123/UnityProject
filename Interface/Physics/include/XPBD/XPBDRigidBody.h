#pragma once

#include "XPBDBody.h"
#include <fstream>


class XPBDRigidBody : public XPBDBody
{
public:
    XPBDRigidBody(Eigen::Vector3f pos, Eigen::Quaternionf rot, Eigen::Vector3f inertia, float mass);
    ~XPBDRigidBody();

    virtual void preSolve(float dt) override;
    virtual void solve(float dt) override;
    virtual void postSolve(float dt) override; 
    virtual void endFrame() override;

    virtual void translatePos(Eigen::Vector3f delta) { x += delta; };

    virtual float getMass() override { return mass; }
    virtual Eigen::Vector3f getPosition() { return x; }
    virtual void setPosition(const Eigen::Vector3f newPos) { x = newPos; }
    virtual Eigen::Quaternionf getRotation() { return q; }
    virtual void setRotation(const Eigen::Quaternionf newRot);
    virtual Eigen::Matrix3f getInertia() { return I; }
    virtual Eigen::Matrix3f getInertiaLocal() { return Ibody; };

protected:
    Eigen::Matrix3f Iinv;            // Inertia and inverse inertia matrix (global)
    Eigen::Matrix3f IbodyInv;    // Inertia and inverse inertia in the local body frame.

    Eigen::Vector3f xprev;              // Previous position
    Eigen::Quaternionf qprev;           // Previous rotation
    Eigen::Vector3f v;                  // Linear velocity.
    Eigen::Vector3f omega;              // Angular velocity.
    Eigen::Vector3f f;                  // Linear force.
    Eigen::Vector3f tau;                // Angular force (torque).

private:
    //std::ofstream logfile;
    static int counter;

    void updateInertia();
};
