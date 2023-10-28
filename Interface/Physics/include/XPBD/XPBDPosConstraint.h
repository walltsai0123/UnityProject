#pragma once

#include "XPBDConstraint.h"



class XPBDPosConstraint : public XPBDConstraint
{
public:
    XPBDPosConstraint(XPBDBody* B1, XPBDBody* B2,
                      Eigen::Vector3f R1 = Eigen::Vector3f::Zero(), Eigen::Vector3f R2 = Eigen::Vector3f::Zero(),
                      float Length = 0.0f, float comp = 0.0f);
    ~XPBDPosConstraint();

    virtual void solveConstraint(float dt) override;
protected:
    Eigen::Vector3f r1, r2;
    float dmax;

private:
    static int counter;
};

