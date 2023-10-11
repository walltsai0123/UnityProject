#pragma once

#include "XPBDBody.h"

class XPBDConstraint
{
public:
    XPBDConstraint(XPBDBody* B1, XPBDBody* B2, float comp): b1(B1), b2(B2), compliance(comp), lambda(0.0f){}
    virtual ~XPBDConstraint(){}

    virtual void solveConstraint(float dt) = 0;

protected:
    XPBDBody* b1;
    XPBDBody* b2;
    
    float lambda;
    float compliance;
};
