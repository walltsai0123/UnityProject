#include "XPBD/XPBDJoint.h"

XPBDJoint::XPBDJoint(XPBDBody *B1, XPBDBody *B2)
    : XPBDConstraint(B1, B2, 0.0f)
{
}

XPBDJoint::~XPBDJoint(){}

void XPBDJoint::computePerpendicularAxis()
{
    // Compute first tangent direction.
    //
    Eigen::Vector3f dir(1,0,0);
    b = dir - (dir.dot(a)) * a;
    if (b.norm() < 1e-5f)
    {
        // Fail-safe: use axis-aligned direction (0,0,-1)
        b = -a.cross(Eigen::Vector3f(0, 0, -1));
    }
    b.normalize();

    // Compute second tangent direction.
    //
    c = a.cross(b);
    c.normalize();
}