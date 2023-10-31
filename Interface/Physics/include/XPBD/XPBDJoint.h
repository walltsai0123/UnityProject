#pragma once

#include "XPBDConstraint.h"

#include <vector>


class XPBDJoint : public XPBDConstraint
{
public:
    XPBDJoint(XPBDBody *B1, XPBDBody *B2);
    virtual ~XPBDJoint();
protected:
    Eigen::Vector3f anchor;         // Local attachment point to body1
    Eigen::Vector3f a, b, c;        // Local perpendicular unit axis [a,b,c]
};

