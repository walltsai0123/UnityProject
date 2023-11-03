#include "XPBD/XPBDFixedJoint.h"

XPBDFixedJoint::XPBDFixedJoint(XPBDBody *B1, XPBDBody *B2)
    : XPBDJoint(B1, B2)
{
    valid = b1->bodyType == BodyType::Rigid && b2->bodyType == BodyType::Rigid;
    if(!valid)
        return;
    Eigen::Vector3f b1x = b1->getPosition();
    Eigen::Vector3f b2x = b2->getPosition();
    Eigen::Quaternionf b1q = b1->getRotation();
    Eigen::Quaternionf b2q = b2->getRotation();

    anchor = b1q.inverse() * (b2x - b1x);
    dq0 = b1q * b2q.inverse();
    compliance = 0.0f;
}

XPBDFixedJoint::~XPBDFixedJoint() {}

void XPBDFixedJoint::solveConstraint(float dt)    
{
    if(!valid)
        return;

    solvePosConstraint(dt, anchor, Eigen::Vector3f::Zero(), 0.0f, compliance);

    Eigen::Quaternionf dq = b1->getRotation() * b2->getRotation().inverse();
    Eigen::AngleAxisf aa(dq * dq0.inverse());
    Eigen::Vector3f dq_fixed = aa.angle() * aa.axis();

    // solveAngConstraint(dt, dq_fixed, 0.0f, compliance);
}