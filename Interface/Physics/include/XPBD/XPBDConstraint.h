#pragma once

#include "XPBDBody.h"

#include <fstream>

class XPBDConstraint
{
public:
    XPBDConstraint(XPBDBody* B1, XPBDBody* B2, float comp);
    virtual ~XPBDConstraint();

    virtual void preSolve(){
        lambda = 0.0f;
    }
    virtual void solveConstraint(float dt) = 0;

protected:
    XPBDBody* body1;
    XPBDBody* body2;
    
    bool valid;
    float lambda;
    float compliance;
    
    std::ofstream logfile;
    static int counter;

    static void solvePosConstraint(float dt, XPBDBody* body1, XPBDBody* body2, const Eigen::Vector3f &r1, const Eigen::Vector3f &r2, const Eigen::Vector3f &dx, float dmax, float comp);
    static void solveAngConstraint(float dt, XPBDBody* body1, XPBDBody* body2, const Eigen::Vector3f &dq, float angle, float comp);
};
