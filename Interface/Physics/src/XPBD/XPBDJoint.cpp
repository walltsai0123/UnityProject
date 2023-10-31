#include "XPBD/XPBDJoint.h"

XPBDJoint::XPBDJoint(XPBDBody *B1, XPBDBody *B2)
    : XPBDConstraint(B1, B2, 0.0f)
{
}

XPBDJoint::~XPBDJoint(){}