#pragma once

#include "XPBD/XPBDJoint.h"
#include <memory>

class XPBDHingeJoint : public XPBDJoint
{
public:
    XPBDHingeJoint(XPBDBody *B1, XPBDBody *B2, const Eigen::Vector3f& anchor, const Eigen::Vector3f& Axis, float comp);
    ~XPBDHingeJoint();

};
