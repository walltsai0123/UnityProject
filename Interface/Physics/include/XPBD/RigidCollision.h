#pragma once

#include "XPBDConstraint.h"

class XPBDRigidBody;

class RigidCollision : public XPBDConstraint
{
public:
    RigidCollision(XPBDRigidBody *rb, const Eigen::Vector3f& localPoint, const Eigen::Vector3f& planeN, const Eigen::Vector3f& planeP, float coef = 0.4f);
    ~RigidCollision();

    virtual void solveConstraint(float dt);
protected:
    float frictionCoef;
    float pene;
    Eigen::Vector3f r;
    Eigen::Vector3f surfaceN;
    Eigen::Vector3f surfaceP;
    Eigen::Vector3f fn;
    float vn_;
};
