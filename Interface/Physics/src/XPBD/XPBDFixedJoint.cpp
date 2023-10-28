#include "XPBD/XPBDFixedJoint.h"

XPBDFixedJoint::XPBDFixedJoint(XPBDBody *B1, XPBDBody *B2, float comp)
    : XPBDJoint(B1, B2)
{
    
}

XPBDFixedJoint::~XPBDFixedJoint() {}

void XPBDFixedJoint::solveConstraint(float dt)    
{
}