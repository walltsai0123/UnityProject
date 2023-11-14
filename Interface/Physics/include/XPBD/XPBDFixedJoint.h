#pragma once

#include "XPBD/XPBDJoint.h"

class XPBDFixedJoint : public XPBDJoint
{
public:
    XPBDFixedJoint(XPBDBody *B1, XPBDBody *B2);
    ~XPBDFixedJoint();

    virtual void solveConstraint(float dt);
protected:
    Eigen::Quaternionf b1q0, b2q0;      // Rest state rotation of body1 and body2
};
