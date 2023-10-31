#pragma once

#include "XPBD/XPBDConstraint.h"
#include <vector>

struct ParticlePos
{
    int id;
    Eigen::Vector3f pos;
};

class XPBDAttatchment : public XPBDConstraint
{
public:
    XPBDAttatchment(XPBDBody *rb, XPBDBody *sb);
    ~XPBDAttatchment();

    virtual void solveConstraint(float dt) override;
protected:
    std::vector<ParticlePos> particlePos;
};

