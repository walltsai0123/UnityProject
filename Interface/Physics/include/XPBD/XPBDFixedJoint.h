#pragma once

#include "XPBD/XPBDJoint.h"

class XPBDFixedJoint : public XPBDJoint
{
public:
    XPBDFixedJoint(XPBDBody *B1, XPBDBody *B2);
    ~XPBDFixedJoint();

     void solveConstraint(float dt);
protected:
};
