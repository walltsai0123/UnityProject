#include "XPBD/XPBDHingeJoint.h"

#include "XPBD/XPBDAngConstraint.h"
#include "XPBD/XPBDPosConstraint.h"

XPBDHingeJoint::XPBDHingeJoint(XPBDBody *B1, XPBDBody *B2, const Eigen::Vector3f &Anchor, const Eigen::Vector3f &Axis, float comp)
    : XPBDJoint(B1, B2)
{
    anchor = Anchor;
    a = Axis;

    // Compute b and c
    Eigen::Vector3f dir(1, 0, 0);
    b = dir - (dir.dot(a)) * a;
    if (b.norm() < 1e-5f)
    {
        // Fail-safe: use axis-aligned direction (0,0,-1)
        b = -a.cross(Eigen::Vector3f(0, 0, -1));
    }
    b.normalize();

    c = a.cross(b);
    c.normalize();
}

XPBDHingeJoint::~XPBDHingeJoint(){}
