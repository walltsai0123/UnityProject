#include "XPBD/XPBDFixedJoint.h"

XPBDFixedJoint::XPBDFixedJoint(XPBDBody *B1, XPBDBody *B2)
    : XPBDJoint(B1, B2)
{
    valid = body1->bodyType == BodyType::Rigid && body2->bodyType == BodyType::Rigid;
    if (!valid)
        return;

    compliance = 0.0f;

    Eigen::Vector3f b1x = body1->getPosition();
    Eigen::Vector3f b2x = body2->getPosition();
    Eigen::Quaternionf b1q = body1->getRotation();
    Eigen::Quaternionf b2q = body2->getRotation();
    b1q0 = b1q;
    b2q0 = b2q;

    anchor = b1q.inverse() * (b2x - b1x);

#ifndef NODEBUG
    logfile << "XPBDFixedJoint\n";
    logfile << "anchor: " << anchor.transpose() << "\n";
    logfile << "b1q0: " << b1q0 << "\n";
    logfile << "b2q0: " << b2q0 << "\n";
    logfile.flush();
#endif
}

XPBDFixedJoint::~XPBDFixedJoint() {}

void XPBDFixedJoint::solveConstraint(float dt)
{
    if (!valid)
        return;

    body1->setRotation(b1q0.conjugate() * body1->getRotation());
    body2->setRotation(b2q0.conjugate() * body2->getRotation());
    Eigen::Quaternionf dq = body1->getRotation() * body2->getRotation().conjugate();
    Eigen::Vector3f dq_fixed = 2.0f * dq.vec();
    solveAngConstraint(dt, dq_fixed, 0.0f, compliance);
    body1->setRotation(b1q0 * body1->getRotation());
    body2->setRotation(b2q0 * body2->getRotation());
    
    solvePosConstraint(dt, anchor, Eigen::Vector3f::Zero(), 0.0f, compliance);
}
