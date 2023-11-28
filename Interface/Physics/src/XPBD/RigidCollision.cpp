#include "XPBD/RigidCollision.h"

#include "XPBD/XPBDRigidBody.h"

RigidCollision::RigidCollision(XPBDRigidBody *rb, const Eigen::Vector3f& localPoint, const Eigen::Vector3f& planeN, const Eigen::Vector3f& planeP, float coef = 0.4f)
    : XPBDConstraint(rb, nullptr, 0.0f),
    r(localPoint),
    surfaceN(planeN), surfaceP(planeP),
    frictionCoef(coef)
{
}

RigidCollision::~RigidCollision(){}

void RigidCollision::solveConstraint(float dt)
{
    Eigen::Vector3f p1 = body1->getPosition() + body1->getRotation() * r;
    Eigen::Vector3f p2(0,0,0);
    Eigen::Vector3f dx = (p1 - p2).dot(surfaceN) * surfaceN;
    solvePosConstraint(dt, body1, body2, r, Eigen::Vector3f(0,0,0), dx, 0.0f, compliance);
}