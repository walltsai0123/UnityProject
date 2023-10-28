#pragma once

#include "XPBDConstraint.h"

class XPBDAngConstraint : public XPBDConstraint
{
public:
    XPBDAngConstraint(XPBDBody *B1, XPBDBody *B2, const Eigen::Vector3f& N, float Angle, float comp);
    ~XPBDAngConstraint();

    virtual void solveConstraint(float dt) override;
protected:
    Eigen::Vector3f a;  // rotation axis (local)
    float angle;
};

