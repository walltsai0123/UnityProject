#pragma once

#include "XPBD/XPBDConstraint.h"
#include <vector>

struct ParticlePos
{
    int id;
    Eigen::Vector3f pos;
};

class XPBDAttachment : public XPBDConstraint
{
public:
    XPBDAttachment(XPBDBody *rb, XPBDBody *sb);
    ~XPBDAttachment();

    virtual void solveConstraint(float dt) override;
protected:
    std::vector<ParticlePos> particlePos;
};

