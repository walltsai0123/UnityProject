#pragma once

#include "XPBDBody.h"
#include "RigidGeometry.h"
#include <memory>


class XPBDRigidBody : public XPBDBody
{
public:
    XPBDRigidBody(Eigen::Vector3f pos, Eigen::Quaternionf rot, Eigen::Vector3f inertia, float mass);
    XPBDRigidBody(Eigen::Vector3f pos, Eigen::Quaternionf rot, Geometry* _geometry, float mass);
    ~XPBDRigidBody();

    inline Geometry* getGeometry() { return geometry.get(); }

    virtual void preSolve(float dt) override;
    virtual void solve(float dt) override;
    virtual void postSolve(float dt) override; 
    virtual void endFrame() override;

    virtual void translatePos(Eigen::Vector3f delta) { x += delta; };

protected:
    Eigen::Matrix3f Iinv;            // Inertia and inverse inertia matrix (global)
    Eigen::Matrix3f IbodyInv;    // Inertia and inverse inertia in the local body frame.

    Eigen::Vector3f xprev;              // Previous position
    Eigen::Quaternionf qprev;           // Previous rotation
    Eigen::Vector3f v;                  // Linear velocity.
    Eigen::Vector3f omega;              // Angular velocity.
    Eigen::Vector3f f;                  // Linear force.
    Eigen::Vector3f tau;                // Angular force (torque).

    std::unique_ptr<Geometry> geometry; // The geometry of the rigid body.
private:
    //std::ofstream logfile;
    static int counter;

    void updateInertia();
};
