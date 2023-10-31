#include "XPBD/XPBDFixedJoint.h"

XPBDFixedJoint::XPBDFixedJoint(XPBDBody *B1, XPBDBody *B2)
    : XPBDJoint(B1, B2)
{
    Eigen::Vector3f b1x = b1->getPosition();
    Eigen::Vector3f b2x = b2->getPosition();
    Eigen::Quaternionf b1q = b1->getRotation();
    Eigen::Quaternionf b2q = b2->getRotation();

    anchor = b1q.inverse() * (b2x - b1x);
    compliance = 0.0f;
}

XPBDFixedJoint::~XPBDFixedJoint() {}

void XPBDFixedJoint::solveConstraint(float dt)    
{
    // solvePosConstraint(dt, anchor, Eigen::Vector3f::Zero(), 0.0f, compliance);

    // Eigen::Quaternionf dq = b1->getRotation() * b2->getRotation().inverse();
    // Eigen::Vector3f dq_fixed = 2.0f * Eigen::Vector3f(dq.x(), dq.y(), dq.z());

    // solveAngConstraint(dt, dq_fixed, 0.0f, compliance);
}